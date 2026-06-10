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
import re

from ...configs.global_configs import DATASET_DIR
from ...utils.lerobot_io import get_episode_count


def _emit(socketio_instance, event, payload):
    if socketio_instance is None:
        return
    try:
        socketio_instance.emit(event, payload)
    except Exception:
        pass


def _checkpoint_ancestors(checkpoint_id):
    """``load_model_id`` 체인을 거슬러 올라가며 ancestor 체크포인트 id 들을
    모은다 (자기 자신 포함). graduate 마다 새 cp 가 ``load_model_id=old_cp``
    로 생성되므로 이 체인이 곧 동일 cp 의 stage-수직 계보다.
    """
    from ...database.models.checkpoint_model import Checkpoint as CheckpointModel
    seen = set()
    cur = checkpoint_id
    while cur is not None and cur not in seen:
        try:
            cur_int = int(cur)
        except (TypeError, ValueError):
            break
        seen.add(cur_int)
        ck = CheckpointModel.find(cur_int)
        if ck is None:
            break
        cur = ck.load_model_id
    return seen


def _training_datasets_for_checkpoint(group, checkpoint_id):
    """그룹의 모든 stage 에 걸친, 이 체크포인트 (및 그 ancestor 들) 의 학습용
    데이터셋 리스트.

    포함: ``role in {'success', 'dagger'}`` (mission 승급 조건이
    ``success + dagger >= target`` 이므로 학습에도 둘 다 들어가야 일관).
    체크포인트 매칭은 ancestor 체인 — graduate 시 cp id 가 바뀌므로
    (예: stage0=cp112 → graduate → stage1=cp117) 단순히 현재 cp id 로만
    필터하면 이전 stage 의 데이터가 모두 빠진다. load_model_id 를 타고
    올라가 동일 계보의 모든 cp id 에 대한 데이터를 포함시킨다.
    제외: 빈 데이터셋 (eps 0개) — ``meta/info.json`` 도 없어서 base 로
    선택되면 ``_merge_datasets_to_dir`` 가 FileNotFoundError 로 fail.
    """
    from ...database.models.dataset_model import Dataset as DatasetModel
    ancestor_ids = _checkpoint_ancestors(checkpoint_id)
    ancestor_strs = {str(a) for a in ancestor_ids}
    out = []
    for stage in group.stages:
        for ds in DatasetModel.all_active().where(DatasetModel.stage_id == stage.id):
            if ds.role not in ('success', 'dagger'):
                continue
            # ancestor 체인 안의 cp 데이터만 포함 (lateral 다른 cp 의 데이터는 제외).
            if str(ds.checkpoint_id) not in ancestor_strs:
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


def _avg_steps_from_results(stage_id, block_id):
    """이 stage 에서 success 로 저장된 해당 **블록** 에피소드의 평균 step 수."""
    from ...database.models.rollout_result_model import RolloutResult as RolloutResultModel
    steps = []
    for res in RolloutResultModel.all_active().where(RolloutResultModel.stage_id == stage_id):
        for ep in (res._get_json_field('episodes') or []):
            if ep.get('block_id') == block_id and ep.get('role') == 'success':
                if ep.get('steps'):
                    steps.append(int(ep['steps']))
    return (sum(steps) / len(steps)) if steps else 0.0


def _escalate_criteria(group, stage):
    """다음 Stage 의 **체크포인트 블록별** 길이 제한(max_steps)을 계산.

    success_criteria 는 블록 단위(키=block_id). 노이즈는 stage 별 escalate 하지 않는다
    (엔진에서 success_rate × rate + offset 실시간 계산). 판정 조건만 블록별로 승급:
      next_max_steps = 성공 에피소드 평균 step × (블록의 cp 의 length_limit_rate)

    rate/threshold 는 학습 config(checkpoint_settings, cp 단위)에서 읽는다 — 같은 cp 의
    여러 블록은 같은 rate 를 쓰되 max_steps 는 블록별 평균으로 따로 계산된다.
    rate 가 ``0``/``None`` 이거나 평균이 0 이면 현재 max_steps 유지(escalation off).
    """
    from ..routes.curriculum import _group_checkpoint_blocks
    from ...database.models.curriculum_model import Curriculum as CurriculumModel
    cp_settings = group._get_json_field('checkpoint_settings') or {}
    current = stage._get_json_field('success_criteria') or {}
    out = dict(current) if isinstance(current, dict) else {}
    curriculum = CurriculumModel.find(group.curriculum_id)
    blocks = _group_checkpoint_blocks(curriculum, group) if curriculum else []
    for b in blocks:
        bid, cp_id = b['block_id'], b['checkpoint_id']
        # checkpoint_settings 는 **블록 단위**(키=block_id). 구버전(cp 키) 데이터는
        # 마이그레이션이 재키잉하지만, 아직 안 돈 경우를 위해 cp 키도 폴백.
        conf = cp_settings.get(str(bid)) or cp_settings.get(str(cp_id)) or cp_settings.get(cp_id) or {}
        rate = conf.get('length_limit_rate')
        threshold = conf.get('success_threshold')
        cur = dict(out.get(bid) or {})
        avg_steps = _avg_steps_from_results(stage.id, bid)
        try:
            rate_f = float(rate) if rate is not None else 0.0
        except (TypeError, ValueError):
            rate_f = 0.0
        if rate_f > 0 and avg_steps > 0:
            cur['max_steps'] = int(avg_steps * rate_f)
        if threshold is not None:
            cur['success_threshold'] = threshold
        out[bid] = cur
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

    # 학습할 체크포인트는 **그룹의 cp 블록**(plan 에 실제 존재하는 체크포인트 블록)에서
    # 불러온다. checkpoint_ids 에 plan 블록이 없는 잔재 cp 가 있어도 제외되고, 같은 cp 가
    # 여러 블록이면 한 번만 학습한다(데이터는 _training_datasets_for_checkpoint 가
    # 그 cp 의 모든 블록·stage 데이터를 ancestor 체인으로 모은다).
    from ..routes.curriculum import _group_checkpoint_blocks
    blocks = _group_checkpoint_blocks(curriculum, group)
    mapping = {}
    seen_cp = set()
    for b in blocks:
        cp_id = b['checkpoint_id']
        if cp_id in seen_cp:
            continue
        seen_cp.add(cp_id)
        old = CheckpointModel.find(cp_id)
        if not old:
            continue
        # 설정은 **블록 단위**(키=block_id). 같은 cp 가 여러 블록이면 seen_cp 덕에
        # plan 순서상 **첫 블록**의 설정을 쓴다(base 데이터셋/train_settings 모두).
        # 학습은 cp 단위 1회지만 데이터는 _training_datasets_for_checkpoint 가 그 cp 의
        # 모든 블록·stage 데이터를 모은다. 구버전 cp 키 데이터는 폴백.
        bid = b['block_id']
        conf = cp_settings.get(str(bid)) or cp_settings.get(str(cp_id)) or cp_settings.get(cp_id) or {}
        base_ids = conf.get('base_dataset_ids') or []
        train_settings = conf.get('train_settings') or {}
        if not isinstance(train_settings, dict):
            train_settings = {}
        # checkpoint_settings 가 이 cp_id 의 conf 를 안 가질 수 있다(스테이지마다 새
        # checkpoint id 가 생기는데 설정 키가 안 따라온 경우). 그러면 같은 그룹의 다른
        # checkpoint conf 로 폴백해 num_epochs 등 학습 파라미터가 비지 않게 한다.
        if not train_settings:
            for _v in cp_settings.values():
                _ts = (_v or {}).get('train_settings')
                if isinstance(_ts, dict) and _ts:
                    train_settings = dict(_ts)
                    break
        # num_epochs 가 비었으면(설정 키가 새 cp id 를 못 따라와 폴백조차 못 잡은
        # 경우 등) 언더트레이닝 됨 — 합리적 기본값으로 floor. 명시값이 있으면 존중.
        if not train_settings.get('num_epochs'):
            train_settings = {**train_settings, 'num_epochs': 5000}
        server_url = train_settings.get('server_url', '') or ''
        # server_url 이 비면 로컬 training_server(127.0.0.1:5100) 가 떠 있을 때 기본값
        # 으로 채운다 — 안 그러면 enqueue 가 스킵돼 자동 학습이 영영 안 걸린다.
        if not server_url:
            import socket
            try:
                with socket.create_connection(('127.0.0.1', 5100), timeout=0.5):
                    server_url = 'http://localhost:5100'
                    train_settings = {**train_settings, 'server_url': server_url}
            except OSError:
                pass
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

        # naming: 매 graduation 마다 ``_s{N}`` 을 누적하면 ``base_s1_s2_s3``
        # 처럼 길어지므로 기존 접미사를 제거하고 현재 stage 번호로 한 번만 붙임.
        # 예: "base" → "base_s1", "base_s1" → "base_s2", "foo_bar_s3" → "foo_bar_s4".
        base_name = re.sub(r'(_s\d+)+$', '', old.name or 'cp')
        new_ckpt = CheckpointModel.create(
            name=f"{base_name}_s{stage.index + 1}",
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
        # 승급 판정은 **plan 에 실제 존재하는 체크포인트 블록** 기준으로 한다.
        # checkpoint_ids 에는 plan 블록이 없는 잔재 cp(과거 승급 흔적 등)가 남아있을
        # 수 있는데, 그런 cp 는 블록이 없어 데이터를 수집할 수 없으므로 (success+dagger)
        # 가 영원히 0 → all_met 이 절대 True 가 되지 않아 학습이 시작되지 않던 버그 방지.
        from ..routes.curriculum import _group_checkpoint_blocks
        blocks = _group_checkpoint_blocks(curriculum, group)
        if not blocks:
            continue
        # 데이터셋 episode 수 집계: (block_id, role) → count
        ep_counts = {}
        for ds in stage.datasets:
            ep_counts[(ds.block_id, ds.role)] = len(ds.episodes or [])
        # 모든 체크포인트 블록이 (success + dagger) >= target 인지.
        all_met = True
        for b in blocks:
            bid = b['block_id']
            s = ep_counts.get((bid, 'success'), 0)
            d = ep_counts.get((bid, 'dagger'), 0)
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

    # 설정·판정조건은 **블록 단위**(키=block_id)다. 졸업 시 플래너 블록의
    # checkpoint_id 만 old→new 로 바뀌고 **block_id 는 그대로**라(_replace_planner_checkpoints),
    # 키를 옮길 필요 없이 그대로 이어진다.
    cs = fresh._get_json_field('checkpoint_settings') or {}
    new_cs = dict(cs) if isinstance(cs, dict) else {}

    # 판정 조건 승급(완료 stage 기준) — 이미 block_id 키. 재키잉 불필요.
    next_criteria = _escalate_criteria(fresh, completed_stage) if completed_stage else {}
    migrated_criteria = dict(next_criteria) if isinstance(next_criteria, dict) else {}

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


def notify_checkpoint_finished(checkpoint_id, socketio_instance=None):
    """학습 끝난 체크포인트가 어느 curriculum 의 training_map 에 있는지 찾아
    있으면 ``check_training_done`` 호출. rollout 이 안 돌고 있어도 graduation
    이 진행되도록 — 업그레이드 버튼 흐름에서 필수.

    training_scheduler 가 체크포인트 종료 직후 호출 (TERMINAL status 진입 시).
    """
    from ...database.models.checkpoint_group_model import CheckpointGroup as CheckpointGroupModel
    from ...database.models.curriculum_model import Curriculum as CurriculumModel

    try:
        cp_id = int(checkpoint_id)
    except (TypeError, ValueError):
        return

    # 이 checkpoint 가 training_map 에 있는 training 그룹 찾기.
    target_group = None
    for group in CheckpointGroupModel.all_active().where(
        CheckpointGroupModel.status == CheckpointGroupModel.STATUS_TRAINING
    ):
        tmap = group._get_json_field('training_map') or {}
        try:
            if any(int(v) == cp_id for v in tmap.values()):
                target_group = group
                break
        except (TypeError, ValueError):
            continue
    if target_group is None:
        return

    curriculum = CurriculumModel.find(target_group.curriculum_id)
    if not curriculum:
        return

    # planner.plans 를 로드해 함께 넘김 — graduation 시 플래너 블록의
    # checkpoint_id 를 새 id 로 교체 + 영속화 하기 위함. plans 없이 호출하면
    # 블록 교체 없이 그룹/스테이지만 갱신돼 다음 rollout 에서도 옛 cp 가 돈다.
    planner = curriculum.planner
    plans = None
    if planner is not None:
        try:
            from ..routes.planner import _ensure_plans_loaded
            plans = _ensure_plans_loaded(planner)
        except Exception as e:
            print(f"[curriculum_train] _ensure_plans_loaded failed: {e}")

    try:
        check_training_done(curriculum.id, plans, planner, socketio_instance)
    except Exception:
        import traceback as _tb
        print(f"[curriculum_train] check_training_done failed:\n{_tb.format_exc()}")


def _replace_planner_checkpoints(plan_ctx, mapping):
    """플래너의 체크포인트 블록 checkpoint_id 를 mapping(old→new)으로 교체하고 DB 에 영속화.

    plan_ctx = (plans_list, planner_model). plans_list 를 in-place 수정해 롤아웃 루프가
    다음 패스부터 새 체크포인트를 쓰게 하고, planner.plans 에도 저장한다.

    ``checkpoint_name`` 도 함께 새 cp 이름으로 refresh — 프론트엔드는 블록 카드에
    block.checkpoint_name 을 표시하는데 이게 블록 생성 시점에 캐시되므로 graduation
    후에도 옛 이름이 노출되는 버그가 있었다.
    """
    from ...database.models.checkpoint_model import Checkpoint as CheckpointModel
    plans, planner = plan_ctx if isinstance(plan_ctx, tuple) else (plan_ctx, None)
    if not plans:
        return
    changed = False
    name_cache = {}
    for grp in plans:
        for blk in grp.get('blocks') or []:
            if blk.get('type') == 'checkpoint':
                old = blk.get('checkpoint_id')
                new = mapping.get(str(old))
                if new is not None and new != old:
                    blk['checkpoint_id'] = new
                    # cached checkpoint_name 도 새 cp 이름으로 갱신.
                    if new not in name_cache:
                        ck = CheckpointModel.find(new)
                        name_cache[new] = (ck.name if ck else None) or ''
                    blk['checkpoint_name'] = name_cache[new]
                    changed = True
    if changed and planner is not None:
        try:
            planner.plans = plans
            planner.save()
        except Exception as e:
            print(f"[curriculum_train] planner.plans persist failed: {e}")
