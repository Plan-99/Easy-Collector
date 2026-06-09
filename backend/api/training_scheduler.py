# -*- coding: utf-8 -*-
"""
TrainingScheduler — 학습 큐의 단일 진실원천(single source of truth).

책임:
  1) `Checkpoint.status` 큐 상태 머신을 소유 — 학습 진행 중 다른 누구도 status를
     mutate 하지 않는다 (호출자는 enqueue / cancel API만 쓴다).
  2) 워커 스레드가 가장 오래된 `queued` 체크포인트를 picking → **GPU 여유가 있으면
     동시에 여러 개**, 부족하면 하나씩 실행한다. 끝나면 다음 `queued`를 자동 진행.
  3) 실행 중 cancel 요청은 per-job `threading.Event`로 신호. runner는 적당한 폴링
     지점에서 event를 보고 self-terminate.

동시성(capacity-based admission):
  - 이미 실행 중인 학습이 **하나도 없으면** 다음 학습을 무조건 시작한다(최소 1개는
    항상 진행 — GPU 여유를 못 재도 멈추지 않는다).
  - 이미 실행 중인 학습이 있으면, 다음 학습을 **동시에 띄울지**를 다음으로 판단:
      a) 동시 실행 수 < ``max_concurrent`` (하드 상한)
      b) 직전 admit 으로부터 ``warmup_sec`` 경과 (방금 띄운 학습이 VRAM 을 다 잡을
         때까지 기다렸다가 다음을 판단 — ramp-up race 로 인한 OOM 방지)
      c) 대상 server_url 의 GPU 여유 VRAM ≥ ``min_free_mib`` (probe 실패 시 보수적
         으로 admit 안 함 = 직렬)
    셋을 모두 만족하면 큐에 쌓지 않고 곧바로 동시 실행한다.
  - 환경변수로 튜닝: ``EC_TRAIN_MAX_CONCURRENT`` / ``EC_TRAIN_MIN_FREE_MIB`` /
    ``EC_TRAIN_WARMUP_SEC``. ``EC_TRAIN_MAX_CONCURRENT=1`` 로 두면 과거의 엄격한
    직렬 동작으로 되돌아간다.

"실행 중" 카운트는 DB(``status='running'`` & not-deleted)를 기준으로 센다 — backend
재시작 후 ``_resume_polling`` 이 인계받은(스케줄러가 직접 소유하지 않는) 학습도
정확히 슬롯·VRAM 을 차지하므로 admission 에 함께 반영된다.

호출자 진입점은 모두 thread-safe:
    scheduler.enqueue(checkpoint_id, server_url, callback_url) → bool
    scheduler.cancel(checkpoint_id) → 'canceled' | 'stopping' | 'not_found'
    scheduler.snapshot() → {running, running_list, queued, recent}

스케줄러는 워크플로 자체(데이터셋 업로드, 학습 시작, 폴링)를 직접 알지 않고,
주입된 `runner_fn(checkpoint, server_url, callback_url, stop_event,
socketio_instance)` 콜러블을 호출한다. runner는 정상 종료 시 'finished',
취소된 경우 'canceled', 그 외 예외는 'failed'를 의미하는 결과로 처리되어야 하며,
이 결정은 status 갱신과 함께 스케줄러가 일괄 처리한다.

GPU 여유 측정은 주입된 `gpu_probe_fn(server_url) -> Optional[int]`(free MiB)로
한다. None 이면 '측정 불가'.
"""
from __future__ import annotations

import datetime
import os
import threading
import time
import traceback
from typing import Callable, Optional

from ..database.models.checkpoint_model import Checkpoint as CheckpointModel
from .gpu_estimate import estimate_gpu_mib_for_checkpoint


# Runner 결과 타입 — runner_fn이 명시적으로 'canceled'를 리턴하면 status=canceled,
# 정상 완료면 'finished', 그 외 예외 발생 시 'failed' 처리.
RunnerResult = str  # 'finished' | 'failed' | 'canceled'


def _env_int(name: str, default: int) -> int:
    try:
        return int(os.environ.get(name, '').strip() or default)
    except (TypeError, ValueError):
        return default


def _env_float(name: str, default: float) -> float:
    try:
        return float(os.environ.get(name, '').strip() or default)
    except (TypeError, ValueError):
        return default


class TrainingScheduler:
    """용량 기반 동시 학습 큐. 모든 status 전이는 여기서.

    runner_fn 시그니처:
        runner_fn(checkpoint, server_url, callback_url, stop_event,
                  socketio_instance) -> 'finished' | 'failed' | 'canceled'
    gpu_probe_fn 시그니처:
        gpu_probe_fn(server_url) -> Optional[int]   # free VRAM MiB, 또는 None
    """

    def __init__(self, socketio, runner_fn: Callable,
                 gpu_probe_fn: Optional[Callable] = None):
        self._socketio = socketio
        self._runner_fn = runner_fn
        self._gpu_probe_fn = gpu_probe_fn

        self._lock = threading.RLock()
        self._cond = threading.Condition(self._lock)
        # DB 가 single source of truth — server_url 등 job 메타도 모두
        # ``checkpoint.train_settings`` 에 영속화. in-memory 큐 없음.
        # 실행 중 job 들: checkpoint_id -> {'stop': Event, 'thread': Thread,
        #                                   'server_url': str}
        self._running: dict[int, dict] = {}
        self._last_admit_ts: float = 0.0  # warmup hold 기준
        self._stop_scheduler = threading.Event()
        self._thread: Optional[threading.Thread] = None

        # 동시성 튜닝 (env override 가능).
        self._max_concurrent = max(1, _env_int('EC_TRAIN_MAX_CONCURRENT', 2))
        self._min_free_mib = _env_int('EC_TRAIN_MIN_FREE_MIB', 4000)
        self._warmup_sec = _env_float('EC_TRAIN_WARMUP_SEC', 90.0)
        # 학습 중 오류(예: 동시 학습으로 인한 OOM) 시 fail 처리 대신 waiting 으로
        # requeue 하는 최대 횟수. 초과 시 비로소 failed. (deterministic 실패의
        # 무한 재시도 방지용 상한.)
        self._max_retries = max(0, _env_int('EC_TRAIN_MAX_RETRIES', 2))

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
            for entry in self._running.values():
                ev = entry.get('stop')
                if ev is not None:
                    ev.set()
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
            # 새 학습 시도 — requeue 재시도 카운터 초기화.
            ts.pop('_train_retries', None)
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
        """실행 중이면 stop_event 신호 → 'stopping'. 큐 대기 중이면 즉시
        'canceled'. 그 외는 'not_found'.

        주의: 이 스케줄러 인스턴스가 소유하지 않은 running(예: backend 재시작 후
        resume_polling 인계분)은 ``_running`` 에 없어 'not_found' 를 돌려준다 —
        그 경우 라우트가 직접 training_server 로 stop 을 보낸다(remote_train).
        """
        with self._cond:
            ckpt = CheckpointModel.find(checkpoint_id)
            if ckpt is None:
                return 'not_found'

            # 1) 우리 워커가 띄운 실행 중 job 이면 stop_event 를 set.
            entry = self._running.get(int(checkpoint_id))
            if entry is not None and entry.get('stop') is not None:
                entry['stop'].set()
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

            # 3) 그 외(이미 finished/failed/canceled, 또는 우리가 소유하지 않은
            #    running)는 not_found.
            return 'not_found'

    def snapshot(self, recent_limit: int = 5) -> dict:
        """현재 큐 상태의 단순 read-only 뷰. 프론트엔드 polling/refresh용.

        Returns:
            {
                running: <checkpoint_dict|None>,    # 첫 실행 항목(하위호환)
                running_list: [<checkpoint_dict>, ...],  # 동시 실행 전체
                queued: [<checkpoint_dict>, ...],   # FIFO 순서 (oldest first)
                recent: [<checkpoint_dict>, ...],   # 최근 종료 N건 (finished/failed)
            }
        """
        # DB 가 single source of truth — status='running' & not-deleted 가
        # 진짜 실행 중. (우리 워커 + resume_polling 인계분 모두 포함.)
        running_list = []
        try:
            rows = (
                CheckpointModel.select()
                .where(
                    CheckpointModel.status == CheckpointModel.STATUS_RUNNING,
                    CheckpointModel.deleted_at.is_null(),
                )
                .order_by(CheckpointModel.started_at.asc(nulls='LAST'))
            )
            running_list = [self._light(r) for r in rows]
        except Exception:
            running_list = []

        # queue 는 ``waiting`` (curriculum 자동 생성) + ``queued`` (사용자 enqueue)
        # 를 모두 포함 — 둘 다 worker 픽업 대상이며 사용자 입장에서 "학습 대기 중".
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
            'running': running_list[0] if running_list else None,
            'running_list': running_list,
            'queued': [self._light(c) for c in queued],
            'recent': [self._light(c) for c in recent],
        }

    @staticmethod
    def _light(ckpt):
        """큐 패널/다이얼로그 동기화에 필요한 최소 필드만 (id/name/status). 전체
        to_dict() 는 lineage·dataset_info·policy 까지 직렬화해 응답이 수십~수백 KB
        로 커지고(>100KB), 동시 요청 시 dev 서버(Flask-SocketIO)가 그 큰 응답을
        깨뜨리는 원인이 된다. 상세는 프론트가 조회(다이얼로그 오픈/상태변화) 시
        /checkpoint/<id> 로 따로 가져온다."""
        fa = getattr(ckpt, 'finished_at', None)
        return {
            'id': ckpt.id,
            'name': ckpt.name,
            'status': ckpt.status,
            'finished_at': fa.isoformat() if fa else None,
        }

    # ------------------------------------------------------------------
    # Worker thread
    # ------------------------------------------------------------------
    def _reap_locked(self) -> None:
        """_running 에서 이미 끝난 thread 항목 정리. (cond 잠금 보유 상태로 호출)"""
        dead = [cid for cid, e in self._running.items()
                if not e['thread'].is_alive()]
        for cid in dead:
            self._running.pop(cid, None)

    def _db_running_count(self) -> int:
        """status='running' & not-deleted 인 cp 수 (resume_polling 인계분 포함)."""
        try:
            return (
                CheckpointModel.select()
                .where(
                    CheckpointModel.status == CheckpointModel.STATUS_RUNNING,
                    CheckpointModel.deleted_at.is_null(),
                )
                .count()
            )
        except Exception:
            # DB 오류 시 보수적으로 '여러 개 실행 중'으로 간주 → 새 admit 보류.
            return self._max_concurrent

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

    def _can_admit(self, server_url: str, running_count: int,
                   estimate_mib: Optional[int] = None) -> bool:
        """이미 ``running_count`` 개가 도는 상태에서 새 학습을 **동시에** 띄워도
        될지 판단. (running_count==0 일 때는 호출 전에 무조건 admit.)

        ``estimate_mib`` 가 주어지면 "예상 사용량 > GPU 여유" 면 admit 하지 않고
        큐에 둔다(=스케줄링). 추정 불가 시 ``min_free_mib`` floor 로 판단.
        """
        # a) 하드 상한.
        if running_count >= self._max_concurrent:
            return False
        # b) warmup hold — 방금 admit 한 학습이 VRAM 을 다 잡을 시간을 준다.
        if (time.time() - self._last_admit_ts) < self._warmup_sec:
            return False
        # c) GPU 여유. probe 불가(None)면 보수적으로 admit 안 함(직렬).
        if self._gpu_probe_fn is None:
            return False
        free = self._gpu_probe_fn(server_url)
        if free is None:
            return False
        # d) 예상 사용량 vs 여유. 추정값이 있으면 그게 들어갈 만큼 여유가 있어야
        #    하고, 없으면 min_free_mib floor 로 판단. (둘 중 큰 값을 요구.)
        required = self._min_free_mib
        if estimate_mib:
            required = max(int(estimate_mib), self._min_free_mib)
        if free < required:
            return False
        return True

    def _run(self) -> None:
        while not self._stop_scheduler.is_set():
            ckpt = None
            job_meta = None
            with self._cond:
                self._reap_locked()
                if self._stop_scheduler.is_set():
                    return

                running_count = self._db_running_count()
                candidate = None
                if running_count < self._max_concurrent:
                    candidate = self._next_queued()

                admit = False
                if candidate is not None:
                    ts = candidate._get_json_field('train_settings') or {}
                    server_url = ts.get('server_url', '')
                    estimate_mib = estimate_gpu_mib_for_checkpoint(candidate)
                    # 실행 중이 하나도 없으면 무조건 시작(최소 1개 진행 보장).
                    # 그 외엔 예상 사용량 vs GPU 여유 기반 판단(예상 > 여유면 대기).
                    if running_count == 0 or self._can_admit(server_url, running_count, estimate_mib):
                        # waiting/queued → running 으로 atomic 전이. 풀에서 다른
                        # 곳이 가로챘으면(cancel 등) 다음 루프.
                        if candidate.status in CheckpointModel.POOL_STATUSES:
                            candidate.status = CheckpointModel.STATUS_RUNNING
                            candidate.started_at = datetime.datetime.now()
                            candidate.save()
                            ckpt = candidate
                            job_meta = {
                                'server_url': server_url,
                                'callback_url': ts.get('callback_url', ''),
                            }
                            stop_event = threading.Event()
                            t = threading.Thread(
                                target=self._run_one,
                                args=(ckpt, job_meta, stop_event),
                                name=f'train_job_cp{ckpt.id}',
                                daemon=True,
                            )
                            self._running[int(ckpt.id)] = {
                                'stop': stop_event,
                                'thread': t,
                                'server_url': server_url,
                            }
                            self._last_admit_ts = time.time()
                            admit = True
                            t.start()

                if not admit:
                    # 더 admit 할 게 없으면(큐 비었거나 용량 부족) enqueue /
                    # job 종료 / shutdown 알림까지 대기. 용량 부족 케이스를 위해
                    # timeout 으로 주기적 재평가(warmup 만료 등).
                    self._cond.wait(timeout=5.0)
                    continue

            # admit 한 경우, 곧바로 루프를 돌며 추가 admit 가능 여부를 다시 본다.
            self._emit_queue_changed()

    def _run_one(self, ckpt, job_meta, stop_event) -> None:
        """단일 학습 job 실행 (per-job 스레드). runner 호출 → 결과로 status 마감."""
        result: RunnerResult = CheckpointModel.STATUS_FAILED
        try:
            result = self._runner_fn(
                checkpoint=ckpt,
                server_url=job_meta.get('server_url', ''),
                callback_url=job_meta.get('callback_url', ''),
                stop_event=stop_event,
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
        #   - canceled : soft-delete (흔적 없이 사라지게).
        #   - failed   : 학습 중 오류(동시 학습 OOM 등 일시적일 수 있음)는 곧장
        #                실패로 굳히지 않고 **waiting 으로 requeue** 한다. 워커가
        #                나중에(다른 학습이 끝나 GPU 여유가 생기거나, 단독 실행
        #                차례가 되면) 다시 시도한다. ``max_retries`` 초과 시에만
        #                비로소 failed. (deterministic 실패 무한루프 방지.)
        final_status = None
        requeued = False
        with self._cond:
            fresh = CheckpointModel.find(ckpt.id)
            if fresh is not None:
                if fresh.status == CheckpointModel.STATUS_RUNNING:
                    if result == CheckpointModel.STATUS_FAILED:
                        ts = fresh._get_json_field('train_settings') or {}
                        retries = int(ts.get('_train_retries', 0) or 0)
                        if retries < self._max_retries:
                            ts['_train_retries'] = retries + 1
                            fresh.train_settings = ts
                            fresh.status = CheckpointModel.STATUS_WAITING
                            fresh.started_at = None
                            fresh.finished_at = None
                            # 큐 맨 뒤로(다른 대기 작업에 양보) — queued_at 갱신.
                            fresh.queued_at = datetime.datetime.now()
                            fresh.save()
                            requeued = True
                            print(f'[TrainingScheduler] cp{ckpt.id} 학습 실패 → '
                                  f'waiting 재투입 (retry {retries + 1}/{self._max_retries})')
                        else:
                            fresh.status = CheckpointModel.STATUS_FAILED
                            fresh.finished_at = datetime.datetime.now()
                            fresh.save()
                            print(f'[TrainingScheduler] cp{ckpt.id} 학습 실패 — '
                                  f'재시도 {self._max_retries}회 초과로 failed 처리')
                    else:
                        fresh.status = result
                        fresh.finished_at = datetime.datetime.now()
                        fresh.save()
                else:
                    fresh.finished_at = datetime.datetime.now()
                    fresh.save()
                final_status = fresh.status
                if result == CheckpointModel.STATUS_CANCELED:
                    try:
                        fresh.delete_instance()
                    except Exception as e:
                        print(f'[TrainingScheduler] soft-delete on cancel-end failed: {e}')
            self._running.pop(int(ckpt.id), None)
            # 슬롯이 비었으니 워커가 다음 admit 을 즉시 재평가하도록 깨운다.
            self._cond.notify_all()
        self._emit_queue_changed()

        # requeue 한 경우는 아직 종료(TERMINAL)가 아니므로 graduation 훅을 돌리지
        # 않는다 — 재시도 후 최종 상태에서 처리.
        if requeued:
            return

        # ── 커리큘럼 graduation trigger ─────────────────────────────────
        # 이 체크포인트가 curriculum 그룹의 training_map 에 있고 그룹의
        # 모든 cp 가 TERMINAL 이면 check_training_done 이 자동으로 graduate.
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
