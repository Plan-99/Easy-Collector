# -*- coding: utf-8 -*-
"""
TrainingScheduler — 학습 큐의 단일 진실원천(single source of truth).

책임:
  1) `Checkpoint.status` 큐 상태 머신을 소유 — 학습 진행 중 다른 누구도 status를
     mutate 하지 않는다 (호출자는 enqueue / cancel API만 쓴다).
  2) 단일 워커 스레드가 가장 오래된 `queued` 체크포인트를 picking → 한 번에 하나씩
     실행. 끝나면 다음 `queued`를 자동으로 진행.
  3) 실행 중 cancel 요청은 `threading.Event`로 신호. 워커는 적당한 폴링 지점에서
     event를 보고 self-terminate.

호출자 진입점은 모두 thread-safe:
    scheduler.enqueue(checkpoint_id, server_url, callback_url) → bool
    scheduler.cancel(checkpoint_id) → 'canceled' | 'stopping' | 'not_found'
    scheduler.snapshot() → {running, queued, recent}

스케줄러는 워크플로 자체(데이터셋 업로드, 학습 시작, 폴링)를 직접 알지 않고,
주입된 `runner_fn(checkpoint, server_url, callback_url, stop_event,
socketio_instance)` 콜러블을 호출한다. runner는 정상 종료 시 'finished',
취소된 경우 'canceled', 그 외 예외는 'failed'를 의미하는 결과로 처리되어야 하며,
이 결정은 status 갱신과 함께 스케줄러가 일괄 처리한다.
"""
from __future__ import annotations

import datetime
import threading
import traceback
from typing import Callable, Optional

from ..database.models.checkpoint_model import Checkpoint as CheckpointModel


# Runner 결과 타입 — runner_fn이 명시적으로 'canceled'를 리턴하면 status=canceled,
# 정상 완료면 'finished', 그 외 예외 발생 시 'failed' 처리.
RunnerResult = str  # 'finished' | 'failed' | 'canceled'


class TrainingScheduler:
    """단일 워커 큐. 모든 status 전이는 여기서.

    runner_fn 시그니처:
        runner_fn(checkpoint, server_url, callback_url, stop_event,
                  socketio_instance) -> 'finished' | 'failed' | 'canceled'
    """

    def __init__(self, socketio, runner_fn: Callable):
        self._socketio = socketio
        self._runner_fn = runner_fn

        self._lock = threading.RLock()
        self._cond = threading.Condition(self._lock)
        # DB 가 single source of truth — server_url 등 job 메타도 모두
        # ``checkpoint.train_settings`` 에 영속화. in-memory 큐 없음.
        self._current_id: Optional[int] = None
        self._current_stop: Optional[threading.Event] = None
        self._stop_scheduler = threading.Event()
        self._thread: Optional[threading.Thread] = None

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------
    def start(self) -> None:
        if self._thread is not None and self._thread.is_alive():
            return
        self._stop_scheduler.clear()
        self._thread = threading.Thread(
            target=self._run, name='TrainingScheduler', daemon=True)
        self._thread.start()

    def shutdown(self) -> None:
        with self._cond:
            self._stop_scheduler.set()
            if self._current_stop is not None:
                self._current_stop.set()
            self._cond.notify_all()
        if self._thread is not None:
            self._thread.join(timeout=2.0)

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------
    def enqueue(self, checkpoint_id: int, server_url: str,
                callback_url: str = '') -> bool:
        """체크포인트를 큐에 넣고 status='queued'로 표시. ``server_url`` /
        ``callback_url`` 은 ``train_settings`` 에 영속화 — backend 재시작 후에도
        worker 가 DB 만 보고 실행할 수 있게.

        Returns True if newly enqueued, False if already active.
        """
        ckpt = CheckpointModel.find(checkpoint_id)
        if ckpt is None:
            return False
        if ckpt.status in CheckpointModel.ACTIVE_STATUSES:
            # 이미 풀(waiting/queued) 또는 실행 중 — idempotent. server_url
            # 만 갱신해서 추후 worker pickup 이 가능하도록.
            ts = ckpt._get_json_field('train_settings') or {}
            ts['server_url'] = server_url
            ts['callback_url'] = callback_url
            ckpt.train_settings = ts
            ckpt.save()
            self._emit_queue_changed()
            return False

        with self._cond:
            ts = ckpt._get_json_field('train_settings') or {}
            ts['server_url'] = server_url
            ts['callback_url'] = callback_url
            ckpt.train_settings = ts
            ckpt.status = CheckpointModel.STATUS_QUEUED
            ckpt.queued_at = datetime.datetime.now()
            ckpt.started_at = None
            ckpt.finished_at = None
            ckpt.save()
            self._cond.notify_all()

        self._emit_queue_changed()
        return True

    def cancel(self, checkpoint_id: int) -> str:
        """큐 대기 중이면 즉시 'canceled'. 실행 중이면 stop_event 신호 → 'stopping'.

        Returns one of: 'canceled', 'stopping', 'not_found'.
        """
        with self._cond:
            ckpt = CheckpointModel.find(checkpoint_id)
            if ckpt is None:
                return 'not_found'

            # 1) 실행 중인 작업이 이 체크포인트라면 stop_event를 set.
            if self._current_id == checkpoint_id and self._current_stop is not None:
                self._current_stop.set()
                return 'stopping'

            # 2) 풀(waiting/queued) 대기 중이면 status=canceled 표시 후 즉시
            #    soft-delete. canceled 상태로 row 를 남기면 사용자에게 의미
            #    없는 흔적만 보이므로 바로 deleted_at 을 set 해서 모든 listing
            #    에서 사라지게 한다.
            if ckpt.status in CheckpointModel.POOL_STATUSES:
                ckpt.status = CheckpointModel.STATUS_CANCELED
                ckpt.finished_at = datetime.datetime.now()
                ckpt.save()
                try:
                    ckpt.delete_instance()  # SoftDeleteModel: deleted_at = now
                except Exception as e:
                    print(f'[TrainingScheduler] soft-delete on cancel failed: {e}')
                self._cond.notify_all()
                self._emit_queue_changed()
                return 'canceled'

            # 3) 그 외 상태(이미 finished/failed/canceled)는 not_found 취급.
            return 'not_found'

    def snapshot(self, recent_limit: int = 5) -> dict:
        """현재 큐 상태의 단순 read-only 뷰. 프론트엔드 polling/refresh용.

        floating 버튼은 active(queued/running)만 보여주지만, 다이얼로그가 학습
        종료 직후 status를 자연스럽게 finished로 전환할 수 있도록 recent도 같이
        제공한다. canceled는 cancel 시점에 soft-delete되므로 자연 제외.

        Returns:
            {
                running: <checkpoint_dict|None>,
                queued: [<checkpoint_dict>, ...],   # FIFO 순서 (oldest first)
                recent: [<checkpoint_dict>, ...],   # 최근 종료 N건 (finished/failed)
            }
        """
        # DB 가 single source of truth — in-memory 큐를 들고 있지 않는다.
        # status='running' 인 row 가 진짜 실행 중인 체크포인트.
        running = None
        try:
            running_row = (
                CheckpointModel.select()
                .where(
                    CheckpointModel.status == CheckpointModel.STATUS_RUNNING,
                    CheckpointModel.deleted_at.is_null(),
                )
                .order_by(CheckpointModel.started_at.desc())
                .first()
            )
            if running_row is not None:
                running = running_row.to_dict()
        except Exception:
            pass

        # queue 는 ``waiting`` (curriculum 자동 생성) + ``queued`` (사용자 enqueue)
        # 를 모두 포함 — 둘 다 worker 픽업 대상이며 사용자 입장에서 "학습 대기 중".
        # 정렬은 queued_at (있으면) 우선, 없으면 created_at — 같은 큐 흐름.
        queued = list(
            CheckpointModel.select()
            .where(
                CheckpointModel.status.in_(list(CheckpointModel.POOL_STATUSES)),
                CheckpointModel.deleted_at.is_null(),
            )
            .order_by(
                CheckpointModel.queued_at.asc(nulls='LAST'),
                CheckpointModel.created_at.asc(),
            )
        )

        recent = list(
            CheckpointModel.select()
            .where(
                CheckpointModel.status.in_([
                    CheckpointModel.STATUS_FINISHED,
                    CheckpointModel.STATUS_FAILED,
                ]),
                CheckpointModel.deleted_at.is_null(),
            )
            .order_by(CheckpointModel.finished_at.desc(),
                      CheckpointModel.updated_at.desc())
            .limit(recent_limit)
        )

        return {
            'running': running,
            'queued': [c.to_dict() for c in queued],
            'recent': [c.to_dict() for c in recent],
        }

    # ------------------------------------------------------------------
    # Worker thread
    # ------------------------------------------------------------------
    def _next_queued(self) -> Optional[CheckpointModel]:
        """가장 오래된 풀(waiting/queued) row 를 가져온다 (FIFO). 없으면 None.
        train_settings.server_url 이 있는 행만 픽업 — 없는 건 사용자가 아직
        설정 안 한 케이스(예: 수동 등록만 한 cp)라 그대로 둔다.
        """
        candidates = (
            CheckpointModel.select()
            .where(
                CheckpointModel.status.in_(list(CheckpointModel.POOL_STATUSES)),
                CheckpointModel.deleted_at.is_null(),
            )
            .order_by(
                CheckpointModel.queued_at.asc(nulls='LAST'),
                CheckpointModel.created_at.asc(),
            )
        )
        for ckpt in candidates:
            ts = ckpt._get_json_field('train_settings') or {}
            if ts.get('server_url'):
                return ckpt
        return None

    def _any_external_running(self) -> bool:
        """이 scheduler 인스턴스가 관리하지 않는 'running' cp 가 DB 에 있는지.

        backend 재시작 직후 ``_resume_polling`` 이 인계받은 cp 가 있을 수 있다.
        이런 cp 가 끝나기 전에 새 queued 를 픽업하면 training_server (단일 worker
        FIFO) 가 거부해서 즉시 failed 가 된다. 이 함수가 True 면 worker 루프는
        다음 픽업을 미룬다.
        """
        try:
            q = CheckpointModel.select().where(
                CheckpointModel.status == CheckpointModel.STATUS_RUNNING,
                CheckpointModel.deleted_at.is_null(),
            )
            return q.exists()
        except Exception:
            return False

    def _run(self) -> None:
        while not self._stop_scheduler.is_set():
            with self._cond:
                while not self._stop_scheduler.is_set():
                    # 외부 (resume_polling) 가 인계받은 running cp 가 있으면
                    # 그게 끝날 때까지 대기 — 동시 enqueue 로 인한 즉시 failed
                    # 방지. ``_current_id`` 는 None 이라 우리 worker 가 다른
                    # job 을 시작하는 걸 막을 게 없으므로 명시적 guard.
                    if self._any_external_running():
                        self._cond.wait(timeout=5.0)
                        continue
                    ckpt = self._next_queued()
                    if ckpt is not None:
                        break
                    # queued가 없으면 enqueue/shutdown 알림까지 대기.
                    self._cond.wait(timeout=5.0)
                if self._stop_scheduler.is_set():
                    return

                # train_settings 에서 server_url/callback_url 을 읽는다.
                # ``_jobs`` 인메모리 dict 의존 제거 — backend 재시작 후에도
                # 동일 동작 보장.
                ts = ckpt._get_json_field('train_settings') or {}
                job_meta = {
                    'server_url': ts.get('server_url', ''),
                    'callback_url': ts.get('callback_url', ''),
                }
                # waiting / queued → running 으로 atomic 전이. 풀에서 다른 곳이
                # 가로챘으면 (cancel 등) 다시 루프.
                if ckpt.status not in CheckpointModel.POOL_STATUSES:
                    continue
                ckpt.status = CheckpointModel.STATUS_RUNNING
                ckpt.started_at = datetime.datetime.now()
                ckpt.save()
                self._current_id = ckpt.id
                self._current_stop = threading.Event()

            # ── 잠금 해제 후 runner 호출 (장시간 실행) ────────────────────
            self._emit_queue_changed()
            result: RunnerResult = CheckpointModel.STATUS_FAILED
            try:
                result = self._runner_fn(
                    checkpoint=ckpt,
                    server_url=job_meta.get('server_url', ''),
                    callback_url=job_meta.get('callback_url', ''),
                    stop_event=self._current_stop,
                    socketio_instance=self._socketio,
                )
                if result not in (
                    CheckpointModel.STATUS_FINISHED,
                    CheckpointModel.STATUS_FAILED,
                    CheckpointModel.STATUS_CANCELED,
                ):
                    result = CheckpointModel.STATUS_FAILED
            except Exception as e:
                traceback.print_exc()
                print(f'[TrainingScheduler] runner raised for ckpt {ckpt.id}: {e}')
                result = CheckpointModel.STATUS_FAILED

            # 종료 처리 — fresh row를 다시 읽어 status가 외부에서 바뀐 상태(예:
            # cancel 직후 정리)이면 그대로 두고, 그 외는 결과 status로 마감.
            # canceled 결과는 soft-delete까지 — 흔적 없이 사라지게.
            final_status = None
            with self._cond:
                fresh = CheckpointModel.find(ckpt.id)
                if fresh is not None:
                    if fresh.status == CheckpointModel.STATUS_RUNNING:
                        fresh.status = result
                    fresh.finished_at = datetime.datetime.now()
                    fresh.save()
                    final_status = fresh.status
                    if result == CheckpointModel.STATUS_CANCELED:
                        try:
                            fresh.delete_instance()
                        except Exception as e:
                            print(f'[TrainingScheduler] soft-delete on cancel-end failed: {e}')
                self._current_id = None
                self._current_stop = None
            self._emit_queue_changed()

            # ── 커리큘럼 graduation trigger ─────────────────────────────────
            # 이 체크포인트가 curriculum 그룹의 training_map 에 있고 그룹의
            # 모든 cp 가 TERMINAL 이면 check_training_done 이 자동으로
            # graduate (플래너 블록 cp id 교체 + 새 stage 생성 + group 상태를
            # collecting 으로 되돌림). rollout 이 안 돌고 있는 "업그레이드"
            # 흐름에서도 동작.
            if final_status in (
                CheckpointModel.STATUS_FINISHED,
                CheckpointModel.STATUS_FAILED,
                CheckpointModel.STATUS_CANCELED,
            ):
                try:
                    from .process.curriculum_train import notify_checkpoint_finished
                    notify_checkpoint_finished(ckpt.id, self._socketio)
                except Exception as e:
                    print(f'[TrainingScheduler] curriculum graduation hook failed: {e}')

    # ------------------------------------------------------------------
    # Notifications
    # ------------------------------------------------------------------
    def _emit_queue_changed(self) -> None:
        """프런트엔드에 큐 변화 알림. 본문은 가벼운 시그널 — 자세한 데이터는
        별도 GET /api/train/queue 로 가져가도록 한다.
        """
        try:
            self._socketio.emit('train_queue_changed', {})
        except Exception:
            pass
