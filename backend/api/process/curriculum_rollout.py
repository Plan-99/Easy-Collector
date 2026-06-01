"""Curriculum 롤아웃 엔진.

플래너를 베이스로, 타겟 체크포인트 그룹에 대해 롤아웃을 반복하며 데이터를 자가 수집한다.
한 번의 롤아웃 = 플래너 1회 실행. 각 타겟 그룹에 대해:
  - 그룹 내 체크포인트마다 판정 조건(max_steps, success_threshold)으로 성공/실패 판정.
  - 그룹 성공 = 그룹 내 전 체크포인트 성공.
  - 그룹 성공 → 녹화 에피소드를 success 데이터셋에 저장.
  - 그룹 실패 → 진행된 체크포인트의 에피소드만 저장 확률에 따라 failure 데이터셋에 저장.
  - 성공/실패 카운트는 저장 확률과 무관하게 항상 갱신.

planner_run 의 모션 핸들러(joint_position / move_relative_ee / replay_episode /
timesleep / query_pose / sync)를 재사용하고, checkpoint 블록만 녹화/판정 버전으로 교체한다.
docs/design-docs/2026-05-29_curriculum-self-training.md 참고.
"""
import copy
import os
import random
import threading
import time
import traceback

from ...configs.global_configs import DATASET_DIR
from ...utils.lerobot_io import append_episode
from ...utils.image_parser import fetch_image_with_config

from .planner_run import (
    _run_joint_position,
    _run_move_relative_ee,
    _run_replay_episode,
    _run_timesleep,
    _run_query_pose,
    _run_sync,
    _preload_checkpoints,
    _compute_sync_barriers,
    _emit,
    _thread_local,
)
from .checkpoint_test import checkpoint_test

# action_type → append_episode 의 action_key 정규화.
_ACTION_KEY_ALIAS = {
    'qaction': 'joint', 'joint': 'joint',
    'ee_delta_action': 'ee_delta', 'ee_delta': 'ee_delta',
    'relative_ee_pos': 'relative_ee_pos',
    'relative_joint_pos': 'relative_joint_pos',
}

DEFAULT_MAX_STEPS = 600
DEFAULT_SUCCESS_THRESHOLD = 0.5


# ── 노이즈 주입 ────────────────────────────────────────────────────────────

_NOISE_AXES = ('x', 'y', 'z', 'ax', 'ay', 'az')


def _sample_delta(spec, success_rate=0.0):
    """노이즈 스펙 → 샘플된 6-벡터 [dx,dy,dz,drx,dry,drz].

    spec: { rate: {x,y,z,ax,ay,az}, offset: {x,y,z,ax,ay,az} }
    축별 delta = uniform(-(success_rate × |rate| + |offset|),
                          +(success_rate × |rate| + |offset|))

    - rate   : 성공률에 비례해 커지는 일반화 강도(랜덤 범위 확장).
    - offset : 항상 더해지는 기본 랜덤 범위 (성공률 무관).
    rate, offset 모두 절댓값으로 사용하며 둘의 합이 그 축의 ± 최댓값.
    상수 변위가 아니라 매 rollout 마다 다시 샘플되는 랜덤 변위.
    """
    if not spec:
        return [0.0] * 6
    rate = spec.get('rate') or {}
    offset = spec.get('offset') or {}
    out = []
    for ax in _NOISE_AXES:
        try:
            r = abs(float(rate.get(ax, 0) or 0))
        except (TypeError, ValueError):
            r = 0.0
        try:
            o = abs(float(offset.get(ax, 0) or 0))
        except (TypeError, ValueError):
            o = 0.0
        mag = success_rate * r + o
        out.append(random.uniform(-mag, mag) if mag > 0 else 0.0)
    return out


def _apply_noise_to_plan(groups, block_specs):
    """플래너 그룹(블록)들을 deep copy 후 모션 블록에 노이즈를 주입한 새 그룹 리스트 반환.

    block_specs: { block_id: { 'spec': {rate, offset}, 'success_rate': float } }
    - move_relative_ee: 각 로봇 delta 벡터(6)에 노이즈 가산.
    - joint_position / query_pose: '_ee_noise'(샘플된 6-벡터) 메타를 달아 런타임 EE nudge.
    """
    out = []
    for grp in groups:
        g = copy.deepcopy(grp)
        for blk in g.get('blocks') or []:
            entry = block_specs.get(blk.get('id'))
            if not entry:
                continue
            spec = entry.get('spec')
            sr = entry.get('success_rate', 0.0)
            btype = blk.get('type')
            if btype == 'move_relative_ee':
                deltas = blk.get('deltas') or {}
                for rid, vec in list(deltas.items()):
                    noise = _sample_delta(spec, sr)
                    if isinstance(vec, (list, tuple)) and len(vec) == 6:
                        deltas[rid] = [float(vec[i]) + noise[i] for i in range(6)]
                blk['deltas'] = deltas
            elif btype in ('joint_position', 'query_pose'):
                # 런타임에 도달 후 EE nudge 로 적용 (arm 만).
                blk['_ee_noise'] = _sample_delta(spec, sr)
        out.append(g)
    return out


def _nudge_arms_ee(block, ctx, task_control):
    """joint_position/query_pose 도달 후, 블록의 _ee_noise 만큼 arm EE 를 상대 이동.

    'joint pos 의 ee pos 에 노이즈' 요구사항 — 현재 EE 를 읽어 delta 를 더한 EE 로 move_ee_to.
    tool agent 는 EE/IK 가 없으므로 건너뛴다.
    """
    d = block.get('_ee_noise')  # 이미 샘플된 6-벡터 [dx,dy,dz,drx,dry,drz]
    if not d or not any(d):
        return
    workspace = ctx['workspaces_by_id'].get(block.get('workspace_id'))
    if workspace is None:
        return
    robots = (workspace.get('assembly') or {}).get('robots') or []
    moved = []
    for robot in robots:
        agent = ctx['agents'].get(int(robot['id']))
        if agent is None or getattr(agent, 'ik_solver', None) is None or getattr(agent, 'role', None) == 'tool':
            continue
        try:
            current_ee = agent.get_ee_position()
        except Exception:
            continue
        if not current_ee:
            continue
        target_ee = {}
        for ee_name, cur in current_ee.items():
            new_pose = list(cur)
            for i in range(min(6, len(new_pose))):
                new_pose[i] = float(cur[i]) + d[i]
            target_ee[ee_name] = new_pose
        try:
            agent.move_ee_to(target_ee, duration=2.0)
            moved.append(agent)
        except Exception as e:
            print(f"[curriculum] ee nudge failed for robot {robot.get('id')}: {e}")
    deadline = time.time() + 8.0
    while time.time() < deadline and moved:
        if task_control['stop']:
            break
        if not any(a.is_moving for a in moved):
            break
        time.sleep(0.1)


# ── 녹화/판정 체크포인트 핸들러 ─────────────────────────────────────────────

def _checkpoint_action_key(checkpoint, policy):
    raw = (
        (checkpoint.get('train_settings') or {}).get('action_type')
        or (policy.get('settings') or {}).get('action_type')
        or 'qaction'
    )
    return _ACTION_KEY_ALIAS.get(raw, 'joint')


def _run_checkpoint_recorded(block, ctx, task_control, group_id):
    """체크포인트 블록을 녹화하며 실행하고, 성공/실패 + 녹화 버퍼를 ctx 에 적재.

    성공 = max_steps 내에 succeed token > success_threshold (checkpoint_test 의 done 신호).
    실패 = max_steps 도달까지 done 미발생. 실패 시 fallback_block 이 있으면 실행.
    """
    from ...database.models.checkpoint_model import Checkpoint as CheckpointModel

    cp_id = block.get('checkpoint_id')
    workspace = ctx['workspaces_by_id'].get(block.get('workspace_id'))
    checkpoint_model = CheckpointModel.find(cp_id)
    if workspace is None or checkpoint_model is None:
        raise RuntimeError(f"checkpoint block invalid: cp={cp_id} ws={block.get('workspace_id')}")

    checkpoint = checkpoint_model.to_dict()
    policy = checkpoint.get('policy')
    if policy is None:
        raise RuntimeError(f"checkpoint '{checkpoint.get('name')}' has no policy")

    robots = (workspace.get('assembly') or {}).get('robots') or []
    sensors = workspace.get('sensors') or []
    agents = []
    for robot in robots:
        agent = ctx['agents'].get(int(robot['id']))
        if agent is None:
            raise RuntimeError(f"robot '{robot.get('name')}' is not running — turn it on first")
        agents.append(agent)

    crit = (ctx['criteria_by_cp'] or {}).get(cp_id) or (ctx['criteria_by_cp'] or {}).get(str(cp_id)) or {}
    try:
        max_steps = int(crit.get('max_steps') or DEFAULT_MAX_STEPS)
    except (TypeError, ValueError):
        max_steps = DEFAULT_MAX_STEPS
    try:
        threshold = float(crit.get('success_threshold') if crit.get('success_threshold') is not None else DEFAULT_SUCCESS_THRESHOLD)
    except (TypeError, ValueError):
        threshold = DEFAULT_SUCCESS_THRESHOLD

    # curriculum 의 rollout 흐름에선 cp 진입 직전 모션 블록 (home/joint_position/
    # move_relative_ee 등) 이 이미 로봇을 적절한 위치로 옮겨놓은 상태다. cp
    # 시작 시 home 으로 다시 점프하면 직전 모션이 무의미해지고 흐름이 깨진다.
    # → planner_run 과 달리 default 를 False 로 두고, 사용자가 PlannerPage 에서
    # 특정 cp 블록에 move_homepose=True 를 명시한 경우에만 home 이동.
    move_homepose = bool(block.get('move_homepose', False))
    record = {}
    sub_control = {'stop': False, 'done': False, 'done_threshold': threshold}
    preloaded = (ctx.get('preloaded') or {}).get(cp_id)

    def _runner():
        try:
            checkpoint_test(
                node=ctx['app'].node,
                checkpoint=checkpoint,
                task=workspace,
                policy_obj=policy,
                agents=agents,
                sensors=sensors,
                socketio_instance=ctx['socketio'],
                task_control=sub_control,
                max_timesteps=max_steps,
                move_homepose=move_homepose,
                move_homepose_duration=float(block.get('move_homepose_duration') or 5.0),
                move_homepose_settle_sec=float(block.get('move_homepose_settle_sec') or 0.0),
                hz=block.get('hz', 10),
                re_inference_steps=block.get('re_inference_steps', 1),
                temporal_ensemble_coeff=block.get('temporal_ensemble_coeff', 0.01),
                action_type=block.get('action_type'),
                preloaded=preloaded,
                record=record,
            )
        except Exception:
            print(f"[curriculum] checkpoint_test failed: {traceback.format_exc()}")

    thread = threading.Thread(target=_runner, name=f"curric_ckpt_{cp_id}", daemon=True)
    thread.start()

    success = False
    user_stopped_block = False
    socketio = ctx.get('socketio')
    last_emit_step = -1
    block_id = block.get('id')
    try:
        while True:
            if task_control['stop']:
                break
            # 사용자가 Monitor 다이얼로그에서 Stop 을 누르면 ``:stop_current_block``
            # API 가 task_control 에 키를 박는다 (ctx 는 process_manager 가
            # 외부에서 못 잡으므로 task_control 채널을 빌려쓰는 구조).
            # 이는 max_steps 초과와 같은 의미의 "실패" 로 처리.
            if task_control.get('stop_current_block'):
                user_stopped_block = True
                task_control['stop_current_block'] = False  # 플래그 소비
                break
            if sub_control.get('done'):
                success = True
                break
            cur_step = int(record.get('steps') or 0)
            # UI 가 체크포인트 블록의 step 진행 (n/max_steps) 을 보여줄 수 있도록
            # planner_run 과 동일 이벤트 이름·payload 로 emit.
            if cur_step != last_emit_step:
                last_emit_step = cur_step
                _emit(socketio, 'planner_block_progress', {
                    'group_id': group_id,
                    'block_id': block_id,
                    'step': cur_step,
                    'max_steps': max_steps,
                })
            if cur_step >= max_steps:
                break
            if not thread.is_alive():
                success = bool(sub_control.get('done'))
                break
            time.sleep(0.1)
    finally:
        sub_control['stop'] = True
        thread.join(timeout=15)

    # 실패 (max_steps OR 사용자 Stop) 이면 프론트에 "교정?" 다이얼로그를 띄울
    # 수 있도록 알린다. dagger 데이터셋 id, stage_id, workspace_id 까지 같이
    # 보내서 프론트가 별도 조회 없이 record_episode 호출을 바로 준비할 수 있게.
    if (not success) and (not task_control['stop']):
        from ...database.models.dataset_model import Dataset as DatasetModel
        stage = (ctx.get('stage_by_group') or {}).get(group_id)
        stage_id = stage.id if stage is not None else None
        dagger_ds_id = None
        if stage_id is not None:
            for d in DatasetModel.all_active().where(
                (DatasetModel.stage_id == stage_id)
                & (DatasetModel.checkpoint_id == cp_id)
                & (DatasetModel.role == 'dagger')
            ):
                dagger_ds_id = d.id
                break
        _emit(socketio, 'curriculum_checkpoint_failed', {
            'curriculum_id': ctx.get('curriculum_id'),
            'group_id': group_id,
            'block_id': block_id,
            'checkpoint_id': cp_id,
            'stage_id': stage_id,
            'dagger_dataset_id': dagger_ds_id,
            'workspace_id': block.get('workspace_id'),
            'max_steps': max_steps,
            'final_step': int(record.get('steps') or 0),
            'reason': 'user_stop' if user_stopped_block else 'max_steps',
        })

    steps = int(record.get('steps') or 0)
    timesteps = record.get('timesteps') or []
    succeed_flags = record.get('succeed_flags') or []
    action_key = _checkpoint_action_key(checkpoint, policy)

    with ctx['lock']:
        ctx['cp_records'].setdefault(group_id, []).append({
            'checkpoint_id': cp_id,
            'success': success,
            'steps': steps,
            'timesteps': timesteps,
            'succeed_flags': succeed_flags,
            'agents': agents,
            'sensors': sensors,
            'task': workspace,
            'action_key': action_key,
            'language_instruction': workspace.get('name') or '',
        })

    print(f"[curriculum] [{group_id}] checkpoint {cp_id} → {'SUCCESS' if success else 'FAIL'} ({steps} steps)")

    # 실패 시 곧바로 fallback 으로 점프하지 않고 **사용자 결정 대기 (pause)**.
    # 프론트엔드 다이얼로그에서 사용자가 다음 중 하나를 선택:
    #   - 건너뛰기 → /:resume_after_failure {action:'fallback'} → fallback 점프
    #   - 교정 시작 + teleop 설정 → 현재 자세에서 record_episode 시작 →
    #     record_episode 종료 시 프론트엔드가 동일 API 로 fallback 점프 트리거
    # 신호는 task_control 에 박힌다 (API thread 와 rollout thread 간 공유 채널).
    if not success and not task_control['stop']:
        fb_id = block.get('fallback_block_id')
        fb = (ctx.get('block_by_id') or {}).get(fb_id) if fb_id else None
        if fb is None:
            print(
                f"[curriculum] [{group_id}] checkpoint {cp_id} failed (no fallback "
                f"resolvable, fallback_block_id={block.get('fallback_block_id')!r})"
            )
        else:
            print(
                f"[curriculum] [{group_id}] checkpoint {cp_id} failed → waiting for "
                f"user decision (fallback id={fb_id})"
            )
            # task_control['correction_decisions'][block_id] 가 set 될 때까지 대기.
            # 0.5s 간격 polling — task_control['stop'] 체크 같이.
            decisions = task_control.setdefault('correction_decisions', {})
            while block_id not in decisions:
                if task_control['stop']:
                    print(f"[curriculum] [{group_id}] pause aborted by stop")
                    return
                time.sleep(0.5)
            action = decisions.pop(block_id, 'fallback')
            print(f"[curriculum] [{group_id}] user decision: {action!r}")
            if action == 'fallback':
                _thread_local.jump_to_block_id = fb_id
            # action in ('abort', 'next') → 점프 안 함, _run_group_once 가 다음
            # 블록으로 순차 진행. 'next' 는 교정 record_episode 후 사용자 Done
            # 선택의 alias. 둘 다 동일 의미 ("fallback 으로 가지 마라").


# ── 블록 디스패치 / 단일 롤아웃 ─────────────────────────────────────────────

_MOTION_HANDLERS = {
    'joint_position': _run_joint_position,
    'move_relative_ee': _run_move_relative_ee,
    'replay_episode': _run_replay_episode,
    'timesleep': _run_timesleep,
    'query_pose': _run_query_pose,
}


def _dispatch_block(block, ctx, task_control, group_id, record_checkpoints=True):
    btype = block.get('type')
    if btype == 'sync':
        _run_sync(block, ctx, task_control, group_id)
        return
    if btype == 'checkpoint':
        cp_id = block.get('checkpoint_id')
        target_cps = ctx['target_cp_by_group'].get(group_id, set())
        if record_checkpoints and cp_id in target_cps:
            _run_checkpoint_recorded(block, ctx, task_control, group_id)
        else:
            # 타겟이 아닌 체크포인트 — 판정/녹화 없이 일반 실행.
            from .planner_run import _run_checkpoint
            _run_checkpoint(block, ctx, task_control)
        return
    handler = _MOTION_HANDLERS.get(btype)
    if handler is None:
        raise RuntimeError(f"unknown block type '{btype}'")
    handler(block, ctx, task_control)
    # 모션 블록의 EE 노이즈 nudge (joint_position / query_pose).
    if block.get('_ee_noise') and btype in ('joint_position', 'query_pose'):
        _nudge_arms_ee(block, ctx, task_control)


def _run_group_once(group, ctx, task_control):
    """한 그룹의 플랜을 이 스레드에서 1회 순차 실행.

    UI 가 현재 어느 블록이 도는지 표시할 수 있도록 ``planner_block_start`` /
    ``planner_block_end`` 이벤트를 emit (planner_run 과 동일 이름·payload 형식).
    체크포인트의 step 진행 emit (``planner_block_progress``) 은 _run_checkpoint /
    _run_checkpoint_recorded 내부에서 처리.
    """
    socketio = ctx.get('socketio')
    group_id = group['id']
    # 핸들러 (체크포인트) 가 jump 신호를 남기면 _run_group_once 가 그 위치로
    # 옮겨 다음 step 에서 fallback 을 정상 블록처럼 실행한다. thread-local
    # 사용으로 그룹 간 race 방지.
    _thread_local.group_id = group_id
    _thread_local.jump_to_block_id = None
    blocks = group.get('blocks') or []
    total = len(blocks)
    id_to_index = {b.get('id'): i for i, b in enumerate(blocks) if b.get('id')}

    # for 가 아닌 while — fallback jump 처리 시 index 자유 이동.
    index = 0
    while index < total:
        if task_control['stop']:
            return
        block = blocks[index]
        block_summary = {
            'group_id': group_id,
            'index': index,
            'total': total,
            'block_id': block.get('id'),
            'type': block.get('type'),
            'name': block.get('name'),
        }
        _emit(socketio, 'planner_block_start', block_summary)
        status = 'finished'
        error = None
        try:
            _dispatch_block(block, ctx, task_control, group_id)
            if task_control['stop']:
                status = 'stopped'
        except Exception as e:
            status = 'error'
            error = str(e)
            print(f"[curriculum] [{group_id}] block '{block.get('name')}' failed: {traceback.format_exc()}")
            _emit(socketio, 'planner_block_end', {**block_summary, 'status': status, 'error': error})
            return
        _emit(socketio, 'planner_block_end', {**block_summary, 'status': status, 'error': error})

        # 핸들러가 fallback jump 신호를 남겼는지 확인. 있으면 그 블록의 plan
        # 내 index 로 이동 → 다음 iteration 에서 fallback 이 planner_block_start
        # 와 함께 정상 블록처럼 실행되고, 끝나면 그 뒤 블록부터 자연스럽게 진행.
        jump_id = getattr(_thread_local, 'jump_to_block_id', None)
        if jump_id:
            _thread_local.jump_to_block_id = None
            target = id_to_index.get(jump_id)
            if target is not None:
                index = target
                continue
            print(
                f"[curriculum] [{group_id}] jump target block_id={jump_id!r} not in "
                f"this group's plan — 순차 진행"
            )
        index += 1


def _run_single_rollout(groups, ctx, task_control):
    """타겟 그룹들의 플랜을 병렬로 1회 실행. ctx['cp_records'] 가 채워진다."""
    ctx['cp_records'] = {}
    # fallback 블록 조회용 id→block 맵(이번 롤아웃의 noised 블록 기준).
    block_by_id = {}
    for grp in groups:
        for blk in grp.get('blocks') or []:
            block_by_id[blk.get('id')] = blk
    ctx['block_by_id'] = block_by_id
    sync_barriers = _compute_sync_barriers(groups)
    ctx['sync_barriers'] = sync_barriers

    threads = []
    for group in groups:
        t = threading.Thread(
            target=_run_group_once, args=(group, ctx, task_control),
            name=f"curric_group_{group['id']}", daemon=True,
        )
        t.start()
        threads.append(t)
    for t in threads:
        t.join()


# ── 판정 / 저장 ────────────────────────────────────────────────────────────

def _datasets_for_stage(stage_id):
    """{(checkpoint_id, role): Dataset} 매핑."""
    from ...database.models.dataset_model import Dataset as DatasetModel
    out = {}
    for ds in DatasetModel.all_active().where(DatasetModel.stage_id == stage_id):
        out[(ds.checkpoint_id, ds.role)] = ds
    return out


SUCCESS_TAIL_PAD = 10  # 성공 에피소드 저장 시 종료 프레임을 몇 개 복제해 뒤에 이어붙일지


def _save_episode(dataset, rec, mark_success=False):
    """녹화된 timesteps 를 dataset 폴더에 lerobot 에피소드로 append.

    mark_success=True (성공 에피소드)이면 종료 프레임의 succeed 라벨을 1 로 두고,
    종료 프레임을 SUCCESS_TAIL_PAD(10)개 복제해 마지막에 이어붙인 뒤 저장한다
    (succeed token 학습용 — 종료 상태를 강조).
    """
    if dataset is None or not rec.get('timesteps'):
        return None
    dataset_dir = os.path.join(DATASET_DIR, str(dataset.id))
    os.makedirs(dataset_dir, exist_ok=True)

    timesteps = list(rec['timesteps'])
    succeed_flags = list(rec.get('succeed_flags') or [])
    # 길이 정합(혹시 succeed_flags 가 짧으면 0 으로 패딩).
    while len(succeed_flags) < len(timesteps):
        succeed_flags.append(0.0)

    if mark_success and timesteps:
        # 종료 프레임 succeed=1.
        succeed_flags[-1] = 1.0
        # 종료 프레임 10개 복제해 뒤에 이어붙임(succeed=1). 동일 ts 참조 재사용 —
        # append_episode 는 각 timestep 을 읽기만 하므로 같은 프레임이 10번 직렬화된다.
        last_ts = timesteps[-1]
        for _ in range(SUCCESS_TAIL_PAD):
            timesteps.append(last_ts)
            succeed_flags.append(1.0)

    try:
        # fetch_image_fn 을 넘겨야 lerobot_append_episode 가 task 의
        # sensor_img_size / sensor_cropped_area / sensor_rotate 를 view 단위로
        # 적용해서 저장한다. 안 넘기면 raw 카메라 프레임이 그대로 저장돼
        # 학습/replay 시 crop 영역이 학습 데이터와 안 맞는 버그가 생긴다
        # (record_episode 경로는 이미 fetch_image_with_config 를 넘기고 있음).
        append_episode(
            dataset_dir=dataset_dir,
            timesteps=timesteps,
            agents=rec['agents'],
            sensors=rec['sensors'],
            task=rec['task'],
            language_instruction=rec.get('language_instruction', ''),
            action_key=rec.get('action_key', 'joint'),
            succeed_flags=succeed_flags,
            fetch_image_fn=fetch_image_with_config,
        )
        return True
    except Exception:
        print(f"[curriculum] append_episode failed: {traceback.format_exc()}")
        return None


def _judge_and_store(rollout, ctx, save_probabilities):
    """ctx['cp_records'] 를 그룹 단위로 판정하고 데이터셋에 저장 + 카운트 갱신.

    save_probabilities: {group_id: float} 실패 데이터 저장 확률.
    반환: {group_id: success_bool}
    """
    from ...database.models.stage_model import Stage as StageModel
    from ...database.models.rollout_result_model import RolloutResult as RolloutResultModel

    socketio = ctx.get('socketio')
    curriculum_id = ctx.get('curriculum_id')
    results = {}
    for group_id, recs in (ctx['cp_records'] or {}).items():
        stage = ctx['stage_by_group'].get(group_id)
        if stage is None:
            continue
        ds_map = _datasets_for_stage(stage.id)
        saved_episodes = []
        prob = float(save_probabilities.get(group_id, 1.0))
        # 카운트는 **그룹 rollout 단위** — 1 파이프라인 한 바퀴 당 한 쪽만 +1.
        # group_success = 이 rollout 의 모든 cp 가 성공했는지. 한 cp 라도 실패면
        # rollout 전체가 실패. 사용자 멘탈모델 "1 trial = 1 파이프라인 한 번".
        group_success = bool(recs) and all(r['success'] for r in recs)
        # _save_episode 의 lerobot_append_episode 가 mp4 인코딩으로 카메라 수
        # × episode 길이에 비례한 sync 인코딩 시간을 메인 thread 에서 잡아먹는다.
        # 사용자에게 "지금 저장 중" 임을 알리려고 시작/종료에 socket emit.
        def _emit_saving(saving, role=None, dataset_id=None, checkpoint_id=None, frames=None):
            _emit(socketio, 'curriculum_saving_episode', {
                'curriculum_id': curriculum_id,
                'group_id': group_id,
                'saving': bool(saving),
                'role': role,
                'dataset_id': dataset_id,
                'checkpoint_id': checkpoint_id,
                'frames': frames,
            })
        if group_success:
            for r in recs:
                ds = ds_map.get((r['checkpoint_id'], 'success')) or ds_map.get((str(r['checkpoint_id']), 'success'))
                # 성공 에피소드: 종료 프레임 succeed=1 + 종료 프레임 10개 후미 패딩.
                if ds is not None:
                    _emit_saving(True, role='success', dataset_id=ds.id,
                                 checkpoint_id=r['checkpoint_id'],
                                 frames=len(r.get('timesteps') or []))
                if _save_episode(ds, r, mark_success=True):
                    saved_episodes.append({'checkpoint_id': r['checkpoint_id'], 'dataset_id': ds.id, 'role': 'success', 'steps': r['steps']})
                if ds is not None:
                    _emit_saving(False)
        else:
            for r in recs:
                # 진행된(녹화된) 체크포인트 에피소드만, 저장 확률 당첨 시 저장.
                if random.random() <= prob:
                    ds = ds_map.get((r['checkpoint_id'], 'failure')) or ds_map.get((str(r['checkpoint_id']), 'failure'))
                    if ds is not None:
                        _emit_saving(True, role='failure', dataset_id=ds.id,
                                     checkpoint_id=r['checkpoint_id'],
                                     frames=len(r.get('timesteps') or []))
                    if _save_episode(ds, r):
                        saved_episodes.append({'checkpoint_id': r['checkpoint_id'], 'dataset_id': ds.id, 'role': 'failure', 'steps': r['steps']})
                    if ds is not None:
                        _emit_saving(False)

        # 카운트는 저장 확률과 무관하게 항상 갱신. 그룹 단위로 정확히 한 번.
        fresh = StageModel.find(stage.id)
        if fresh:
            if group_success:
                fresh.success_count = (fresh.success_count or 0) + 1
            else:
                fresh.failure_count = (fresh.failure_count or 0) + 1
            fresh.save()
            ctx['stage_by_group'][group_id] = fresh
        RolloutResultModel.create(
            rollout_id=rollout.id,
            checkpoint_group_id=group_id_to_int(group_id, ctx),
            stage_id=stage.id,
            success=group_success,
            episodes=saved_episodes,
        )
        results[group_id] = group_success

    return results


def group_id_to_int(group_id, ctx):
    """plan group id(문자열 토큰) → CheckpointGroup.id 매핑. 둘이 다르므로 ctx 매핑 사용."""
    return ctx['ckpt_group_id_by_plan_group'].get(group_id)


# ── 타겟팅(매 iteration 재계산) ──────────────────────────────────────────────

def _build_targeting(target_group_ids, plans):
    """현재 collecting 상태인 타겟 그룹들에 대한 롤아웃 타겟팅 맵을 만든다.

    training 중인 그룹은 제외(수집 중단). 반환 maps + run_plan_groups(실행할 plan group).
    """
    from ...database.models.checkpoint_group_model import CheckpointGroup as CheckpointGroupModel

    criteria_by_cp = {}
    block_specs = {}
    stage_by_plan_group = {}
    save_prob_by_plan_group = {}
    ckpt_group_id_by_plan_group = {}
    target_cp_by_plan_group = {}

    for gid in target_group_ids:
        group = CheckpointGroupModel.find(gid)
        if not group or group.status == CheckpointGroupModel.STATUS_TRAINING:
            continue  # 학습 중 그룹은 수집 제외
        stage = group.current_stage
        if stage is None or stage.status != stage.STATUS_ACTIVE:
            continue
        cps = group._get_json_field('checkpoint_ids') or []
        criteria = stage._get_json_field('success_criteria') or {}
        for cp_id, crit in (criteria.items() if isinstance(criteria, dict) else []):
            try:
                criteria_by_cp[int(cp_id)] = crit
            except (TypeError, ValueError):
                criteria_by_cp[cp_id] = crit
        succ = stage.success_count or 0
        fail = stage.failure_count or 0
        sr = succ / (succ + fail) if (succ + fail) > 0 else 0.0
        block_noise = group._get_json_field('block_noise') or {}
        if isinstance(block_noise, dict):
            for bid, spec in block_noise.items():
                block_specs[bid] = {'spec': spec, 'success_rate': sr}
        mission = group._get_json_field('mission') or {}
        save_prob = float(mission.get('failure_save_prob', 1.0)) if isinstance(mission, dict) else 1.0
        for plan_group in plans:
            blocks = plan_group.get('blocks') or []
            plan_cps = {b.get('checkpoint_id') for b in blocks if b.get('type') == 'checkpoint'}
            if plan_cps & set(cps):
                pg_id = plan_group['id']
                stage_by_plan_group[pg_id] = stage
                save_prob_by_plan_group[pg_id] = save_prob
                ckpt_group_id_by_plan_group[pg_id] = group.id
                target_cp_by_plan_group.setdefault(pg_id, set()).update(set(cps) & plan_cps)

    run_plan_groups = [g for g in plans if g['id'] in stage_by_plan_group and g.get('blocks')]
    return {
        'criteria_by_cp': criteria_by_cp,
        'block_specs': block_specs,
        'stage_by_plan_group': stage_by_plan_group,
        'save_prob_by_plan_group': save_prob_by_plan_group,
        'ckpt_group_id_by_plan_group': ckpt_group_id_by_plan_group,
        'target_cp_by_plan_group': target_cp_by_plan_group,
        'run_plan_groups': run_plan_groups,
    }


def _any_target_group_training(target_group_ids):
    from ...database.models.checkpoint_group_model import CheckpointGroup as CheckpointGroupModel
    for gid in target_group_ids:
        g = CheckpointGroupModel.find(gid)
        if g and g.status == CheckpointGroupModel.STATUS_TRAINING:
            return True
    return False


# ── 엔트리포인트 ───────────────────────────────────────────────────────────

def curriculum_rollout(curriculum_id, target_group_ids, repeat_count, app,
                       socketio_instance, task_control):
    """ProcessManager function task 엔트리.

    target_group_ids: CheckpointGroup.id 리스트 (타겟 체크포인트 그룹).
    repeat_count: 롤아웃 반복 횟수 (<=0 이면 무한, stop 까지).
    """
    from ...database.models.curriculum_model import Curriculum as CurriculumModel
    from ...database.models.checkpoint_group_model import CheckpointGroup as CheckpointGroupModel
    from ...database.models.rollout_model import Rollout as RolloutModel
    from ..routes.planner import _ensure_plans_loaded, _load_workspaces_for_blocks

    curriculum = CurriculumModel.find(curriculum_id)
    if not curriculum:
        print(f"[curriculum] curriculum {curriculum_id} not found")
        return
    planner = curriculum.planner
    if not planner:
        print(f"[curriculum] planner for curriculum {curriculum_id} not found")
        return

    plans = _ensure_plans_loaded(planner)
    if not plans:
        print("[curriculum] planner has no plans")
        return

    # 워크스페이스/preload 는 전체 플랜 기준으로 한 번만(졸업 후 새 체크포인트는 lazy load).
    all_blocks = [b for g in plans for b in (g.get('blocks') or [])]
    workspaces = _load_workspaces_for_blocks(all_blocks)
    preloaded = _preload_checkpoints(plans, socketio_instance)

    ctx = {
        'app': app,
        'agents': getattr(app, 'agents', {}) or {},
        'socketio': socketio_instance,
        'workspaces_by_id': {ws['id']: ws for ws in workspaces},
        'preloaded': preloaded,
        'lock': threading.Lock(),
        'cp_records': {},
        # curriculum_checkpoint_failed 이벤트 emit 용
        'curriculum_id': curriculum_id,
    }

    infinite = (repeat_count is None) or (int(repeat_count) <= 0)
    total = None if infinite else int(repeat_count)
    _emit(socketio_instance, 'curriculum_rollout_start', {
        'curriculum_id': curriculum_id,
        'target_group_ids': target_group_ids,
        'total_rollouts': total,
    })

    from .curriculum_train import check_and_promote, check_training_done

    iteration = 0
    try:
        while infinite or iteration < total:
            if task_control['stop']:
                break

            # 1) 학습 완료 그룹 졸업(체크포인트/플래너 블록 교체 + 다음 stage).
            try:
                check_training_done(curriculum_id, plans, planner, socketio_instance)
            except Exception:
                print(f"[curriculum] graduation check failed: {traceback.format_exc()}")

            # 2) collecting 타겟 그룹으로 타겟팅 재계산(training 그룹 제외).
            tg = _build_targeting(target_group_ids, plans)
            run_plan_groups = tg['run_plan_groups']

            # 3) 수집할 collecting 그룹이 없으면 플래너 일시정지.
            if not run_plan_groups:
                if not _any_target_group_training(target_group_ids):
                    print("[curriculum] no collecting groups and none training — ending")
                    break
                # 일부/전부 학습 중 → 졸업 폴링하며 대기(플래너 멈춤).
                _emit(socketio_instance, 'curriculum_rollout_paused', {'curriculum_id': curriculum_id})
                for _ in range(30):
                    if task_control['stop']:
                        break
                    time.sleep(1)
                    try:
                        if check_training_done(curriculum_id, plans, planner, socketio_instance):
                            break
                    except Exception:
                        pass
                continue  # iteration 증가 없이 재평가

            iteration += 1
            _emit(socketio_instance, 'curriculum_rollout_iteration', {
                'curriculum_id': curriculum_id, 'iteration': iteration, 'total': total,
            })

            # ctx 를 이번 iteration 의 타겟팅으로 갱신.
            ctx['criteria_by_cp'] = tg['criteria_by_cp']
            ctx['stage_by_group'] = tg['stage_by_plan_group']
            ctx['target_cp_by_group'] = tg['target_cp_by_plan_group']
            ctx['ckpt_group_id_by_plan_group'] = tg['ckpt_group_id_by_plan_group']

            noised_groups = _apply_noise_to_plan(run_plan_groups, tg['block_specs'])
            rollout = RolloutModel.create(
                curriculum_id=curriculum_id,
                target_group_ids=target_group_ids,
                noise_snapshot={k: v.get('spec') for k, v in tg['block_specs'].items()},
                status=RolloutModel.STATUS_RUNNING,
            )

            _run_single_rollout(noised_groups, ctx, task_control)
            store_results = _judge_and_store(rollout, ctx, tg['save_prob_by_plan_group'])

            rollout.status = (
                RolloutModel.STATUS_STOPPED if task_control['stop'] else RolloutModel.STATUS_FINISHED
            )
            rollout.save()

            _emit(socketio_instance, 'curriculum_rollout_result', {
                'curriculum_id': curriculum_id, 'iteration': iteration,
                'results': {str(k): v for k, v in store_results.items()},
            })

            # Stage Mission 달성 검사 → 학습 시작(training 진입).
            if not task_control['stop']:
                try:
                    check_and_promote(curriculum_id, ctx, socketio_instance)
                except Exception:
                    print(f"[curriculum] promotion check failed: {traceback.format_exc()}")
    finally:
        try:
            preloaded.clear()
        except Exception:
            pass
        try:
            import gc, torch
            gc.collect()
            torch.cuda.empty_cache()
        except Exception:
            pass
        from ...database.models.curriculum_model import Curriculum as _CM
        _c = _CM.find(curriculum_id)
        if _c:
            _c.status = _CM.STATUS_STOPPED if task_control.get('stop') else _CM.STATUS_IDLE
            _c.save()
        _emit(socketio_instance, 'curriculum_rollout_end', {
            'curriculum_id': curriculum_id,
            'rollouts_done': iteration,
        })
