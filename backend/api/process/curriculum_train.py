"""Curriculum 승급 & 자동 학습.

Stage Mission(예: 성공 데이터 N개) 달성을 감지하면:
  1. 그룹 내 체크포인트마다 (이 stage + 이전 stage success 데이터셋 + base 데이터셋)을 모아
     현재 체크포인트로부터 이어학습(load_model_id)을 큐에 넣는다.
  2. 성공률·성공 에피소드 평균 길이에 비례해 노이즈/판정조건을 업그레이드한 다음 Stage 를 생성한다.
  3. 그룹의 checkpoint_ids 를 새로 학습될 체크포인트로 갱신(계보 전진)한다.

학습은 기존 TrainingScheduler/remote_train 인프라를 그대로 재사용한다(비동기).
docs/design-docs/2026-05-29_curriculum-self-training.md 참고.
"""
import os

from ...configs.global_configs import DATASET_DIR
from ...utils.lerobot_io import get_episode_count


def _emit(socketio_instance, event, payload):
    if socketio_instance is None:
        return
    try:
        socketio_instance.emit(event, payload)
    except Exception:
        pass


def _training_datasets_for_checkpoint(group, checkpoint_id):
    """그룹의 모든 stage 에 걸친, 이 체크포인트의 학습용 데이터셋 리스트.

    포함: ``role in {'success', 'dagger'}`` (mission 승급 조건이
    ``success + dagger >= target`` 이므로 학습에도 둘 다 들어가야 일관).
    제외: 빈 데이터셋 (eps 0개) — ``meta/info.json`` 도 없어서 base 로
    선택되면 ``_merge_datasets_to_dir`` 가 FileNotFoundError 로 fail.
    """
    from ...database.models.dataset_model import Dataset as DatasetModel
    out = []
    for stage in group.stages:
        for ds in DatasetModel.all_active().where(DatasetModel.stage_id == stage.id):
            if ds.role not in ('success', 'dagger'):
                continue
            if str(ds.checkpoint_id) != str(checkpoint_id):
                continue
            # 빈 데이터셋 (LeRobot 포맷 초기화 안 됨) 은 학습 머지에서 제외.
            if not (ds.episodes or []):
                continue
            out.append(ds)
    return out


# 구 함수명 호환 (기존 호출자가 있을 수 있음).
_success_datasets_for_checkpoint = _training_datasets_for_checkpoint


def _avg_success_episode_len(datasets):
    """success 데이터셋들의 평균 에피소드 프레임 수(없으면 0)."""
    total_frames = 0
    total_eps = 0
    for ds in datasets:
        folder = os.path.join(DATASET_DIR, str(ds.id))
        if not os.path.isdir(folder):
            continue
        try:
            cnt = get_episode_count(folder)
        except Exception:
            cnt = 0
        eps = ds.episodes or []
        total_eps += max(cnt, len(eps))
    # 프레임 단위 평균은 parquet 파싱이 필요해 비용이 큼 → 에피소드 수 기반 근사 대신
    # RolloutResult 의 steps 평균을 쓰는 호출자에서 보강. 여기선 episode 수만 반환.
    return total_eps


def _avg_steps_from_results(stage_id, checkpoint_id):
    """이 stage 에서 success 로 저장된 해당 체크포인트 에피소드의 평균 step 수."""
    from ...database.models.rollout_result_model import RolloutResult as RolloutResultModel
    steps = []
    for res in RolloutResultModel.all_active().where(RolloutResultModel.stage_id == stage_id):
        for ep in (res._get_json_field('episodes') or []):
            if str(ep.get('checkpoint_id')) == str(checkpoint_id) and ep.get('role') == 'success':
                if ep.get('steps'):
                    steps.append(int(ep['steps']))
    return (sum(steps) / len(steps)) if steps else 0.0


def _escalate_criteria(group, stage):
    """다음 Stage 의 체크포인트별 길이 제한(max_steps)을 계산.

    노이즈는 더 이상 stage 별로 escalate 하지 않는다(엔진에서 success_rate × rate + offset
    으로 실시간 계산). 판정 조건만 체크포인트별로 승급:
      next_max_steps = 성공 에피소드 평균 step × (체크포인트별 length_limit_rate)

    rate 가 ``0`` 이거나 ``None`` 이면 **현재 max_steps 를 그대로 유지** (= 다음
    stage 에서도 길이 제한을 좁히지 않음 — 사용자가 "이 cp 는 길이 제한 escalation
    을 하지 마라" 라고 의도한 경우). 평균이 0 (성공 에피소드 없음) 인 경우도 동일.
    success_threshold 는 체크포인트 설정값 유지.
    """
    cp_settings = group._get_json_field('checkpoint_settings') or {}
    current = stage._get_json_field('success_criteria') or {}
    out = dict(current) if isinstance(current, dict) else {}
    for cp_id in (group._get_json_field('checkpoint_ids') or []):
        conf = cp_settings.get(str(cp_id)) or cp_settings.get(cp_id) or {}
        rate = conf.get('length_limit_rate')
        threshold = conf.get('success_threshold')
        cur = dict(out.get(str(cp_id)) or out.get(cp_id) or {})
        avg_steps = _avg_steps_from_results(stage.id, cp_id)
        # rate 가 None/0/음수 → max_steps 그대로 유지 (escalation off).
        try:
            rate_f = float(rate) if rate is not None else 0.0
        except (TypeError, ValueError):
            rate_f = 0.0
        if rate_f > 0 and avg_steps > 0:
            cur['max_steps'] = int(avg_steps * rate_f)
        if threshold is not None:
            cur['success_threshold'] = threshold
        out[str(cp_id)] = cur
    return out


def _enqueue_training(curriculum, group, stage, socketio_instance, app=None):
    """그룹 내 각 체크포인트에 대해 이어학습 체크포인트를 생성하고 큐에 넣는다.

    base 데이터셋·training 파라미터는 체크포인트별 설정(group.checkpoint_settings)에서 읽는다.
    ``app`` 은 background thread 에서 호출될 때를 위해 명시적으로 받음 — 안 받으면
    flask current_app proxy 로 시도 (request 컨텍스트가 있을 때만 동작). curriculum
    rollout 처럼 socketio.start_background_task 로 띄운 thread 에선 current_app 이
    실패하므로 ctx['app'] 을 받아서 전달해야 한다.
    반환: {old_checkpoint_id: new_checkpoint_id}
    """
    from ...database.models.checkpoint_model import Checkpoint as CheckpointModel

    cp_settings = group._get_json_field('checkpoint_settings') or {}
    if not isinstance(cp_settings, dict):
        cp_settings = {}

    scheduler = None
    if app is not None:
        scheduler = getattr(app, 'training_scheduler', None)
    if scheduler is None:
        try:
            from flask import current_app
            scheduler = current_app.training_scheduler
        except Exception:
            scheduler = None

    mapping = {}
    for cp_id in (group._get_json_field('checkpoint_ids') or []):
        old = CheckpointModel.find(cp_id)
        if not old:
            continue
        # 체크포인트별 base 데이터셋 + training 파라미터.
        conf = cp_settings.get(str(cp_id)) or cp_settings.get(cp_id) or {}
        base_ids = conf.get('base_dataset_ids') or []
        train_settings = conf.get('train_settings') or {}
        if not isinstance(train_settings, dict):
            train_settings = {}
        server_url = train_settings.get('server_url', '')
        callback_url = train_settings.get('callback_url', '')

        training_datasets = _training_datasets_for_checkpoint(group, cp_id)
        dataset_info = {}
        for ds in training_datasets:
            dataset_info[str(ds.id)] = {}
        for bid in base_ids:
            dataset_info[str(bid)] = {}
        if not dataset_info:
            print(f"[curriculum_train] cp {cp_id}: no datasets to train on, skip")
            continue

        new_ckpt = CheckpointModel.create(
            name=f"{old.name or 'cp'}_s{stage.index + 1}",
            task_id=old.task_id,
            policy_id=old.policy_id,
            dataset_info=dataset_info,
            status='waiting',
            train_settings=train_settings,
            load_model_id=cp_id,
        )
        mapping[cp_id] = new_ckpt.id
        if scheduler and server_url:
            try:
                scheduler.enqueue(int(new_ckpt.id), server_url, callback_url)
            except Exception as e:
                print(f"[curriculum_train] enqueue failed for ckpt {new_ckpt.id}: {e}")
        else:
            # 어느 쪽이 비어서 enqueue 가 스킵됐는지 정확히 노출. scheduler 가
            # None 이면 거의 항상 background thread 에서 app context 가 없어서
            # current_app proxy 가 실패한 경우 (caller 가 app= 안 넘긴 케이스).
            reason_parts = []
            if not scheduler:
                reason_parts.append('no scheduler (app context missing)')
            if not server_url:
                reason_parts.append('no server_url in train_settings')
            reason = ' + '.join(reason_parts) or 'unknown'
            print(f"[curriculum_train] checkpoint {new_ckpt.id} created (status=waiting), not enqueued: {reason}")
        _emit(socketio_instance, 'curriculum_training_started', {
            'curriculum_id': curriculum.id,
            'checkpoint_group_id': group.id,
            'old_checkpoint_id': cp_id,
            'new_checkpoint_id': new_ckpt.id,
        })
    return mapping


def check_and_promote(curriculum_id, ctx=None, socketio_instance=None):
    """collecting 그룹 중 현재 stage 가 Mission 을 달성한 그룹 → 학습 시작(training 진입).

    승급 조건은 **per-cp** ``(성공 에피소드 + 교정 에피소드) >= target`` — 그룹 안
    모든 체크포인트가 각자 목표 개수에 도달해야 한다. 한 체크포인트에서 교정이
    많이 발생하고 다른 체크포인트는 실패만 누적되면, 두 체크포인트의 (성공+교정)
    수가 달라지므로 둘 다 목표 도달 시점까지 stage 가 유지된다.
    """
    from ...database.models.curriculum_model import Curriculum as CurriculumModel
    from ...database.models.checkpoint_group_model import CheckpointGroup as CheckpointGroupModel

    curriculum = CurriculumModel.find(curriculum_id)
    if not curriculum:
        return

    for group in curriculum.checkpoint_groups:
        # 이미 학습 중인 그룹은 건너뜀.
        if group.status == CheckpointGroupModel.STATUS_TRAINING:
            continue
        stage = group.current_stage
        if stage is None or stage.status != stage.STATUS_ACTIVE:
            continue
        mission = group._get_json_field('mission') or {}
        if not isinstance(mission, dict):
            continue
        target = int(mission.get('target_success_count') or 0)
        if target <= 0:
            continue
        cp_ids = group._get_json_field('checkpoint_ids') or []
        if not cp_ids:
            continue
        # 데이터셋 episode 수 집계: (cp_id, role) → count
        ep_counts = {}
        for ds in stage.datasets:
            ep_counts[(ds.checkpoint_id, ds.role)] = len(ds.episodes or [])
        # 모든 cp 가 (success + dagger) >= target 인지.
        all_met = True
        for cp in cp_ids:
            s = ep_counts.get((cp, 'success'), 0) or ep_counts.get((str(cp), 'success'), 0)
            d = ep_counts.get((cp, 'dagger'), 0) or ep_counts.get((str(cp), 'dagger'), 0)
            if (s + d) < target:
                all_met = False
                break
        if not all_met:
            continue
        _start_group_training(curriculum, group, stage, socketio_instance, app=(ctx or {}).get('app'))


def _start_group_training(curriculum, group, stage, socketio_instance, app=None):
    """미션 달성 그룹: 현재 stage 완료 처리 + 체크포인트 순차 학습 큐잉 + status=training.

    checkpoint_ids 전진과 다음 stage 생성은 학습 완료(graduation) 시점으로 미룬다.
    학습할 데이터가 없으면(빈 mapping) 즉시 졸업(체크포인트 교체 없이 다음 stage).
    """
    from ...database.models.stage_model import Stage as StageModel
    from ...database.models.checkpoint_group_model import CheckpointGroup as CheckpointGroupModel

    print(f"[curriculum_train] group {group.id} stage {stage.index} mission met → start training")

    stage.status = StageModel.STATUS_COMPLETED
    stage.save()

    mapping = _enqueue_training(curriculum, group, stage, socketio_instance, app=app)

    fresh = CheckpointGroupModel.find(group.id)
    if not mapping:
        # 학습할 게 없으면 곧바로 졸업(교체 없이 다음 stage 진입).
        _graduate_group(curriculum.id, fresh, stage, {}, None, socketio_instance)
        return

    fresh.status = CheckpointGroupModel.STATUS_TRAINING
    fresh.training_map = {str(k): v for k, v in mapping.items()}
    fresh.save()
    _emit(socketio_instance, 'curriculum_group_training', {
        'curriculum_id': curriculum.id,
        'checkpoint_group_id': fresh.id,
        'training_map': fresh.training_map,
    })


def check_training_done(curriculum_id, plans=None, planner=None, socketio_instance=None):
    """training 그룹 중 모든 학습이 끝난 그룹을 졸업 처리. 졸업이 하나라도 있으면 True.

    plans/planner 가 주어지면 졸업 시 플래너 블록의 checkpoint_id 를 새 체크포인트로 교체한다.
    """
    from ...database.models.curriculum_model import Curriculum as CurriculumModel
    from ...database.models.checkpoint_group_model import CheckpointGroup as CheckpointGroupModel
    from ...database.models.checkpoint_model import Checkpoint as CheckpointModel

    curriculum = CurriculumModel.find(curriculum_id)
    if not curriculum:
        return False

    TERMINAL = ('finished', 'failed', 'canceled')
    graduated = False
    for group in curriculum.checkpoint_groups:
        if group.status != CheckpointGroupModel.STATUS_TRAINING:
            continue
        tmap = group._get_json_field('training_map') or {}
        if not tmap:
            continue
        statuses = {}
        for old, new in tmap.items():
            ck = CheckpointModel.find(new)
            statuses[old] = ck.status if ck else 'failed'
        if not all(s in TERMINAL for s in statuses.values()):
            continue  # 아직 학습 중인 체크포인트가 있음
        # 졸업: finished 면 새 체크포인트로, 아니면 old 유지.
        effective = {}
        for old, new in tmap.items():
            ck = CheckpointModel.find(new)
            effective[old] = new if (ck and ck.status == 'finished') else int(old)
        completed_stage = group.current_stage  # COMPLETED 상태의 마지막 stage
        _graduate_group(curriculum_id, group, completed_stage, effective, (plans, planner), socketio_instance)
        graduated = True
    return graduated


def _graduate_group(curriculum_id, group, completed_stage, mapping, plan_ctx, socketio_instance):
    """학습 완료 그룹 졸업: 체크포인트 교체 + 플래너 블록 교체 + 다음 stage 생성 + status=collecting."""
    from ...database.models.stage_model import Stage as StageModel
    from ...database.models.checkpoint_group_model import CheckpointGroup as CheckpointGroupModel
    from ..routes.curriculum import _create_stage_datasets

    fresh = CheckpointGroupModel.find(group.id)
    old_ids = fresh._get_json_field('checkpoint_ids') or []
    # mapping: {str(old): new}. 교체된 새 id 목록.
    new_ids = [mapping.get(str(cp), cp) for cp in old_ids]

    # 체크포인트별 설정 키 old→new 이전.
    cs = fresh._get_json_field('checkpoint_settings') or {}
    new_cs = {}
    for cp in old_ids:
        conf = cs.get(str(cp)) or cs.get(cp)
        if conf is not None:
            new_cs[str(mapping.get(str(cp), cp))] = conf

    # 판정 조건 승급(완료 stage 기준).
    next_criteria = _escalate_criteria(fresh, completed_stage) if completed_stage else {}
    # criteria 키도 old→new 이전.
    migrated_criteria = {}
    for cp in old_ids:
        c = (next_criteria.get(str(cp)) or next_criteria.get(cp)) if isinstance(next_criteria, dict) else None
        if c is not None:
            migrated_criteria[str(mapping.get(str(cp), cp))] = c

    fresh.checkpoint_ids = new_ids
    fresh.checkpoint_settings = new_cs
    fresh.status = CheckpointGroupModel.STATUS_COLLECTING
    fresh.training_map = {}
    fresh.save()

    # 플래너 블록의 checkpoint_id 교체(old→new) + 영속화.
    if plan_ctx and mapping:
        _replace_planner_checkpoints(plan_ctx, mapping)

    next_index = ((completed_stage.index if completed_stage else -1) or 0) + 1
    next_stage = StageModel.create(
        checkpoint_group_id=fresh.id,
        index=next_index,
        status=StageModel.STATUS_ACTIVE,
        rollout_noise={},
        success_criteria=migrated_criteria,
        success_count=0,
        failure_count=0,
    )
    _create_stage_datasets(next_stage, fresh)

    print(f"[curriculum_train] group {fresh.id} graduated → stage {next_index}, checkpoints {old_ids}→{new_ids}")
    _emit(socketio_instance, 'curriculum_group_graduated', {
        'curriculum_id': curriculum_id,
        'checkpoint_group_id': fresh.id,
        'new_stage_index': next_index,
        'checkpoint_ids': new_ids,
    })


def _replace_planner_checkpoints(plan_ctx, mapping):
    """플래너의 체크포인트 블록 checkpoint_id 를 mapping(old→new)으로 교체하고 DB 에 영속화.

    plan_ctx = (plans_list, planner_model). plans_list 를 in-place 수정해 롤아웃 루프가
    다음 패스부터 새 체크포인트를 쓰게 하고, planner.plans 에도 저장한다.
    """
    plans, planner = plan_ctx if isinstance(plan_ctx, tuple) else (plan_ctx, None)
    if not plans:
        return
    changed = False
    for grp in plans:
        for blk in grp.get('blocks') or []:
            if blk.get('type') == 'checkpoint':
                old = blk.get('checkpoint_id')
                new = mapping.get(str(old))
                if new is not None and new != old:
                    blk['checkpoint_id'] = new
                    changed = True
    if changed and planner is not None:
        try:
            planner.plans = plans
            planner.save()
        except Exception as e:
            print(f"[curriculum_train] planner.plans persist failed: {e}")
