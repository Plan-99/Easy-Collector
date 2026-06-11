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
    _run_visual_reach,
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


def _prev_stage_success_rate(group, stage):
    """현재 stage 직전(index-1) stage 의 **최종** 성공률 (없으면 0.0).

    노이즈 강도(일반화 난이도)는 직전 stage 의 성공률로 정한다 — 직전 stage 가
    잘 됐을수록 이번 stage 의 peg 위치 랜덤 범위를 넓혀 난이도를 올린다.
    현재 stage 의 실시간 성공률을 쓰면 한 stage 안에서 성공이 쌓일수록 노이즈가
    점점 커지는 버그가 되므로, **현재 stage 동안 불변인 직전 stage 값**을 쓴다.
    """
    cur_idx = stage.index if stage.index is not None else 0
    prev = None
    for s in group.stages:  # all_active + index 오름차순
        si = s.index if s.index is not None else -1
        if si == cur_idx - 1 and s.id != stage.id:
            prev = s  # 같은 index 가 여럿이면 가장 최근(뒤) 것
    if prev is None:
        return 0.0
    sc = prev.success_count or 0
    fc = prev.failure_count or 0
    return sc / (sc + fc) if (sc + fc) > 0 else 0.0


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


def _rewind_to_step(timesteps, agents, k, duration=0.4):
    """되감기(rewind): ``timesteps[k]`` 의 관절 qpos 로 각 agent 를 보간 이동.

    v1 범위 — **로봇 팔(관절)만** 되돌린다. 시뮬 물체(free joint) 상태는 건드리지
    않으므로 교정은 "현재 물체 상태 + 되감은 팔 위치"에서 시작한다. move_to 는 비동기
    보간이라 여기서 is_moving 폴링으로 정착까지 대기한 뒤 리턴한다.
    """
    if not timesteps or k < 0 or k >= len(timesteps):
        return
    ts = timesteps[k]
    robot_states = (getattr(ts, 'observation', None) or {}).get('robot_states') or {}
    moved = []
    for agent in agents:
        st = robot_states.get(agent.id)
        if st is None:
            st = robot_states.get(str(agent.id))
        if not st or st.get('qpos') is None:
            continue
        try:
            agent.move_to(list(st['qpos']), duration=duration)
            moved.append(agent)
        except Exception as e:
            print(f"[curriculum] rewind move_to failed (agent {agent.id}): {e}")
    # 보간 정착 대기 — is_moving(_move_deadline 기반, **property**) 가 모두 끝날
    # 때까지. planner_run 과 동일하게 괄호 없이 속성으로 접근한다.
    _settle_end = time.time() + duration + 0.3
    while time.time() < _settle_end:
        if not any(a.is_moving for a in moved):
            break
        time.sleep(0.02)


# ── 동시 진행 플랜 교정 coordination ───────────────────────────────────────
# 두 플랜(그룹)이 병렬 실행될 때, 한쪽이 체크포인트 교정(teleop)에 들어가면
# 작업자는 한 번에 한 팔만 다룰 수 있으므로 교정은 직렬화돼야 한다. 교정 중인
# 그룹을 ``task_control['correcting_groups']`` (set of group_id) 로 표시하고,
# 다른 그룹의 체크포인트 추론 폴링이 이를 보고 자기 추론을 pause(현재 자세 유지)
# 한다. 교정이 끝나 set 에서 빠지면 멈췄던 지점에서 그대로 추론을 재개한다.
# correcting_groups 는 여러 그룹 스레드가 동시에 만지므로 ctx['lock'] 으로 보호.
def _set_correcting(ctx, task_control, group_id, active):
    with ctx['lock']:
        s = task_control.setdefault('correcting_groups', set())
        if active:
            s.add(group_id)
        else:
            s.discard(group_id)


def _others_correcting(ctx, task_control, group_id):
    """이 그룹이 아닌 다른 그룹이 현재 교정 중이면 True → 추론 pause 대상."""
    with ctx['lock']:
        s = task_control.get('correcting_groups') or ()
        return any(g != group_id for g in tuple(s))


def _clear_group_correction_state(ctx, task_control, group_id):
    """그룹의 교정 pause 상태(awaiting/rewind/correcting)를 한 번에 정리."""
    with ctx['lock']:
        (task_control.get('awaiting_correction_by_group') or {}).pop(group_id, None)
        (task_control.get('rewind_seek_by_group') or {}).pop(group_id, None)
        (task_control.get('rewind_step_by_group') or {}).pop(group_id, None)
        s = task_control.get('correcting_groups')
        if s is not None:
            s.discard(group_id)


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

    # 판정조건(success_criteria)은 **블록 단위** — 같은 cp 가 여러 블록이어도 따로.
    crit = (ctx.get('criteria_by_block') or {}).get(block.get('id')) or {}
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
                # 커리큘럼은 success_criteria(블록 단위)에서 우선 읽고, 없으면 plan
                # block 값, 그래도 없으면 기본 3.
                succeed_done_frames=int(crit.get('succeed_done_frames') or block.get('succeed_done_frames') or 3),
                preloaded=preloaded,
                record=record,
            )
        except Exception:
            print(f"[curriculum] checkpoint_test failed: {traceback.format_exc()}")

    success = False
    user_stopped_block = False
    socketio = ctx.get('socketio')
    last_emit_step = -1
    block_id = block.get('id')

    # 즉시 DAgger 모드: 체크포인트 진입 즉시 모델 실행을 건너뛰고 교정(expert
    # 데모 수집)으로 직행한다. 모델이 아직 매우 약해 어차피 100% 실패하므로 모델
    # 롤아웃 시간을 낭비하지 않고 바로 expert 데모를 모은다. 직전 wv 블록이 arm 을
    # 타겟 위로 옮겨놓았으므로 expert(run_grasp/run_insert)가 그 자리에서 시작.
    force_dagger = bool(task_control.get('force_dagger'))
    thread = None
    if not force_dagger:
        thread = threading.Thread(target=_runner, name=f"curric_ckpt_{cp_id}", daemon=True)
        thread.start()
    try:
        while not force_dagger:
            if task_control['stop']:
                break
            # 동시 진행 플랜 coordination — 다른 그룹이 교정 중이면 이 추론을
            # 일시정지(로봇 현재 자세 유지). pause 동안은 done/max_steps 판정도
            # 멈춰, 교정이 끝나면 멈췄던 그 지점에서 그대로 이어서 추론한다.
            if _others_correcting(ctx, task_control, group_id):
                if not sub_control.get('pause'):
                    sub_control['pause'] = True
                    _emit(socketio, 'curriculum_block_paused', {
                        'curriculum_id': ctx.get('curriculum_id'),
                        'group_id': group_id, 'block_id': block_id, 'paused': True,
                    })
                time.sleep(0.1)
                continue
            if sub_control.get('pause'):
                sub_control['pause'] = False
                _emit(socketio, 'curriculum_block_paused', {
                    'curriculum_id': ctx.get('curriculum_id'),
                    'group_id': group_id, 'block_id': block_id, 'paused': False,
                })
            # 사용자가 Monitor 의 "실패처리" 를 누르면 ``:stop_current_block`` API 가
            # task_control 에 키를 박는다 (ctx 는 process_manager 가 외부에서 못
            # 잡으므로 task_control 채널을 빌려쓰는 구조). 동시 플랜에선 그룹별 키
            # (stop_current_block_by_group)로 해당 플랜만 정확히 겨냥하고, group_id
            # 없이 온 legacy 전역 신호(단일 플랜)는 fallback 으로 같이 본다.
            # 이는 max_steps 초과와 같은 의미의 "실패" 로 처리.
            _scb = task_control.get('stop_current_block_by_group') or {}
            if _scb.get(group_id) or task_control.get('stop_current_block'):
                user_stopped_block = True
                if group_id in _scb:
                    _scb[group_id] = False  # 그룹별 플래그 소비
                if task_control.get('stop_current_block'):
                    task_control['stop_current_block'] = False  # legacy 전역 소비
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
        if thread is not None:
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
            # dagger 데이터셋도 **블록 단위** 로 찾는다.
            for d in DatasetModel.all_active().where(
                (DatasetModel.stage_id == stage_id)
                & (DatasetModel.block_id == block_id)
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
            # 블록별 교정 expert 서비스 (예: cp_grasp→/tutorial/run_grasp,
            # cp_insert→/tutorial/run_insert). 프론트가 motion_planning 교정 시
            # 이 값을 ros2_service 로 자동 선택해 grasp/insert 데이터가 각자
            # 체크포인트의 dagger 데이터셋에 따로 쌓이게 한다.
            'correction_service': block.get('correction_service'),
            'max_steps': max_steps,
            'final_step': int(record.get('steps') or 0),
            'reason': ('forced_dagger' if force_dagger
                       else 'user_stop' if user_stopped_block else 'max_steps'),
        })

    steps = int(record.get('steps') or 0)
    timesteps = record.get('timesteps') or []
    succeed_flags = record.get('succeed_flags') or []
    action_key = _checkpoint_action_key(checkpoint, policy)
    # 성공 데이터 다운샘플(블록 단위) — enabled 면 stride rate, 아니면 1(무다운샘플).
    _sd_rate = 1
    if crit.get('success_downsample'):
        try:
            _sd_rate = max(1, int(crit.get('success_downsample_rate') or 1))
        except (TypeError, ValueError):
            _sd_rate = 1

    with ctx['lock']:
        ctx['cp_records'].setdefault(group_id, []).append({
            'checkpoint_id': cp_id,
            'block_id': block_id,
            'success': success,
            'steps': steps,
            'timesteps': timesteps,
            'succeed_flags': succeed_flags,
            'success_downsample_rate': _sd_rate,
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
            # 동시 진행 플랜 coordination — 이 그룹이 교정에 들어감을 표시한다. 다른
            # 그룹의 체크포인트 추론 폴링이 이를 보고 자기 추론을 pause(현재 자세
            # 유지)하고, 이 교정이 끝나면(아래 정리에서 set 해제) 그대로 재개한다.
            _set_correcting(ctx, task_control, group_id, True)
            # resume_after_failure 가 "지금 이 블록의 결정을 기다리는 중"인지 알 수
            # 있도록 마커 노출(그룹별). 이중 클라이언트의 stale/중복 신호를 라우트에서
            # 걸러내고, 동시 플랜에서 두 그룹의 교정 대기가 충돌하지 않게 한다.
            task_control.setdefault('awaiting_correction_by_group', {})[group_id] = block_id
            # Rewind(되감기): 실패 자세에서 사용자가 ←/→ 로 체크포인트가 실행한 과거
            # step 으로 팔을 되감을 수 있게 한다. 프론트가 절대 step K 를 :rewind_seek
            # 로 보내면 여기서 timesteps[K] 의 qpos 로 각 agent 를 move_to 한다. 교정
            # 레코딩은 forceNoHomepose 라 되감은 자세에서 그대로 시작된다.
            rewind_total = len(timesteps)
            task_control.setdefault('rewind_step_by_group', {})[group_id] = max(0, rewind_total - 1)
            _emit(socketio, 'curriculum_rewind_ready', {
                'curriculum_id': ctx.get('curriculum_id'),
                'group_id': group_id,
                'block_id': block_id,
                'total_steps': rewind_total,
            })
            while block_id not in decisions:
                if task_control['stop']:
                    _clear_group_correction_state(ctx, task_control, group_id)
                    print(f"[curriculum] [{group_id}] pause aborted by stop")
                    return
                seek = (task_control.get('rewind_seek_by_group') or {}).pop(group_id, None)
                if seek is not None and rewind_total > 0:
                    # 되감기 처리 중 어떤 오류가 나도 pause 를 유지해야 한다. 예외가
                    # 밖으로 새면 _dispatch_block 이 블록 실패로 보고 fallback 으로
                    # 떨어뜨려, 화살표 한 번에 롤아웃이 망가진다 → 반드시 가둔다.
                    try:
                        try:
                            k = max(0, min(int(seek), rewind_total - 1))
                        except (TypeError, ValueError):
                            k = (task_control.get('rewind_step_by_group') or {}).get(group_id, rewind_total - 1)
                        _rewind_to_step(timesteps, agents, k)
                        task_control.setdefault('rewind_step_by_group', {})[group_id] = k
                        _emit(socketio, 'curriculum_rewind_position', {
                            'curriculum_id': ctx.get('curriculum_id'),
                            'group_id': group_id,
                            'block_id': block_id,
                            'step': k,
                            'total_steps': rewind_total,
                        })
                    except Exception:
                        print(f"[curriculum] [{group_id}] rewind seek failed: {traceback.format_exc()}")
                    continue
                time.sleep(0.1)
            # 첫 신호 도착 후 짧은 grace — 이중 클라이언트에서 교정을 안 몬 쪽이 쏜
            # 기본 'fallback' 이 정상 'next' 보다 먼저 도착해도, grace 동안 들어온
            # 'next' 가 'fallback' 을 이긴다(라우트의 non-override 규칙과 짝). 단일
            # 클라이언트면 추가 신호가 없어 grace 만 잠깐 기다리고 그대로 진행.
            _grace_end = time.time() + 1.5
            while time.time() < _grace_end and not task_control['stop']:
                time.sleep(0.1)
            action = decisions.pop(block_id, 'fallback')
            # 교정 종료 — 그룹별 awaiting/rewind 정리 + correcting set 에서 빠진다.
            # 이 시점에 다른 그룹의 추론 pause 가 자동 해제되어 그대로 재개된다.
            _clear_group_correction_state(ctx, task_control, group_id)
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
    # Wrist View Reach: move to an observe pose, locate the target in the wrist
    # RGB-D, and reach to it. Same planner-block handler the Planner page uses —
    # a curriculum group can include it just like any other motion block.
    'visual_reach': _run_visual_reach,
}


def _dispatch_block(block, ctx, task_control, group_id, record_checkpoints=True):
    btype = block.get('type')
    if btype == 'sync':
        _run_sync(block, ctx, task_control, group_id)
        return
    if btype == 'checkpoint':
        # 직전 블록 실행/교정 도중에 들어온 stale ``stop_current_block`` 신호가
        # 이 블록을 0스텝에서 즉시 중단시키는 것을 방지 — 블록 시작 시점에 플래그를
        # 비운다. 이 블록이 시작된 이후(추론 중)에 눌린 stop 만 set 되어 살아남는다.
        # (예: 교정 중 Space 연타로 포커스된 'stop' 버튼이 우발적으로 눌려 신호가
        #  남아도, 다음 체크포인트가 그걸 물려받아 곧바로 실패하던 문제 차단.)
        task_control['stop_current_block'] = False
        task_control.setdefault('stop_current_block_by_group', {})[group_id] = False
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
            # visual_reach 검출 실패 = peg 가 관찰 뷰 밖(예: 직전 사이클이 insert 후
            # put_peg 복원 전에 중단돼 peg 가 hole 에 고아로 남음)일 가능성이 크다.
            # 그대로 두면 다음 iteration wv1 도 또 0px → 무한 cascade 로 막힌다.
            # randomize_peg 로 peg 를 home±0.05(랜덤·직립)에 되돌려 자가복구한다.
            # 매 에피소드 리셋이 아니라 **검출 실패 시에만** 호출하고, 랜덤 위치라
            # 데이터 다양성도 유지된다(plan 의 put_peg 정상복원과 충돌하지 않음).
            if block.get('type') == 'visual_reach':
                _recover_peg(block, group_id)
            # 비-checkpoint 블록(visual_reach 등)이 throw 했을 때, fallback_block_id 가
            # 지정돼 있으면 그룹 전체를 abort(→ 바깥 루프가 첫 home 부터 재시작)하지
            # 않고 fallback 블록으로 점프해 복구한다. 체크포인트 실패의 fallback 점프와
            # 동일한 의미. fallback 이 없거나 이 그룹 plan 에 없으면 종전대로 abort.
            fb_id = block.get('fallback_block_id')
            fb_index = id_to_index.get(fb_id) if fb_id else None
            if fb_index is not None:
                print(f"[curriculum] [{group_id}] block '{block.get('name')}' "
                      f"→ fallback block_id={fb_id} (index {fb_index})")
                index = fb_index
                continue
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
    """{(block_id, role): Dataset} 매핑 — 기록은 체크포인트 블록 단위."""
    from ...database.models.dataset_model import Dataset as DatasetModel
    out = {}
    for ds in DatasetModel.all_active().where(DatasetModel.stage_id == stage_id):
        out[(ds.block_id, ds.role)] = ds
    return out


SUCCESS_TAIL_PAD = 10  # 성공 에피소드 저장 시 종료 프레임을 몇 개 복제해 뒤에 이어붙일지


def _save_episode(dataset, rec, mark_success=False, downsample_rate=1):
    """녹화된 timesteps 를 dataset 폴더에 lerobot 에피소드로 append.

    mark_success=True (성공 에피소드)이면 종료 프레임의 succeed 라벨을 1 로 두고,
    종료 프레임을 SUCCESS_TAIL_PAD(10)개 복제해 마지막에 이어붙인 뒤 저장한다
    (succeed token 학습용 — 종료 상태를 강조).

    downsample_rate > 1 이면 timesteps 를 stride(rate) 로 솎아서 저장한다(성공 데이터
    다운샘플). 종료 프레임 보존을 위해 마지막 프레임은 항상 포함하고, succeed tail
    패딩은 다운샘플 이후에 적용한다.
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

    # 다운샘플(stride). rate 프레임마다 1프레임만 유지하되, 마지막(종료) 프레임은
    # 반드시 포함해 종료 상태가 빠지지 않게 한다.
    try:
        rate = max(1, int(downsample_rate or 1))
    except (TypeError, ValueError):
        rate = 1
    if rate > 1 and len(timesteps) > 1:
        idx = list(range(0, len(timesteps), rate))
        if idx[-1] != len(timesteps) - 1:
            idx.append(len(timesteps) - 1)
        timesteps = [timesteps[i] for i in idx]
        succeed_flags = [succeed_flags[i] for i in idx]

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
        # group_success = 이 그룹의 **모든 타겟 체크포인트 블록**이 이번 rollout 에서
        # 성공 기록을 남겼는지. 단순히 ``all(recs 가 성공)`` 으로 보면, 사용자가 중간에
        # 중단해서 뒤쪽 블록이 아예 실행/기록되지 않은 경우 recs 에 성공한 앞 블록만
        # 남아 잘못 성공 처리된다. → 기대 타겟 블록 집합이 성공 블록 집합에 모두
        # 포함되어야 성공 (하나라도 실패하거나 미실행이면 실패). 사용자 멘탈모델
        # "1 trial = 파이프라인 전체 1회, 전부 성공해야 성공".
        expected_blocks = (ctx.get('target_block_by_group') or {}).get(group_id) or set()
        succeeded_blocks = {r.get('block_id') for r in recs if r['success']}
        if expected_blocks:
            group_success = expected_blocks.issubset(succeeded_blocks)
        else:
            # 타겟 블록 정보가 없으면(구버전 ctx 등) 기존 동작으로 폴백.
            group_success = bool(recs) and all(r['success'] for r in recs)
        # 데이터셋 저장은 **cp 별** — 각 cp 가 자기 자신의 성공/실패 여부에 따라
        # 해당 cp 의 success / failure 데이터셋으로 저장. 그룹이 실패해도 그
        # rollout 안에서 성공한 cp 가 있으면 그 cp 의 success 데이터로 수집.
        # 저장 확률은 **실패** 데이터에만 적용 (성공 데이터는 항상 저장).
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
        for r in recs:
            # 데이터셋 lookup 은 **블록 단위**(block_id). checkpoint_id 는 라벨·학습용 보존.
            bid = r.get('block_id')
            if r['success']:
                ds = ds_map.get((bid, 'success'))
                # 성공 에피소드: 종료 프레임 succeed=1 + 종료 프레임 10개 후미 패딩.
                if ds is not None:
                    _emit_saving(True, role='success', dataset_id=ds.id,
                                 checkpoint_id=r['checkpoint_id'],
                                 frames=len(r.get('timesteps') or []))
                # 성공 데이터만 블록 설정의 rate 로 다운샘플(설정 시).
                if _save_episode(ds, r, mark_success=True, downsample_rate=int(r.get('success_downsample_rate') or 1)):
                    saved_episodes.append({'checkpoint_id': r['checkpoint_id'], 'block_id': bid, 'dataset_id': ds.id, 'role': 'success', 'steps': r['steps']})
                if ds is not None:
                    _emit_saving(False)
            else:
                # 실패 cp — 저장 확률 당첨 시에만 failure 데이터셋에 저장.
                if random.random() <= prob:
                    ds = ds_map.get((bid, 'failure'))
                    if ds is not None:
                        _emit_saving(True, role='failure', dataset_id=ds.id,
                                     checkpoint_id=r['checkpoint_id'],
                                     frames=len(r.get('timesteps') or []))
                    if _save_episode(ds, r):
                        saved_episodes.append({'checkpoint_id': r['checkpoint_id'], 'block_id': bid, 'dataset_id': ds.id, 'role': 'failure', 'steps': r['steps']})
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

    criteria_by_block = {}
    block_specs = {}
    stage_by_plan_group = {}
    save_prob_by_plan_group = {}
    ckpt_group_id_by_plan_group = {}
    target_cp_by_plan_group = {}
    target_block_by_plan_group = {}  # pg_id → set(block_id) — 이 그룹이 성공하려면
                                     # 반드시 성공 기록이 있어야 하는 타겟 체크포인트 블록들.

    for gid in target_group_ids:
        group = CheckpointGroupModel.find(gid)
        if not group or group.status == CheckpointGroupModel.STATUS_TRAINING:
            continue  # 학습 중 그룹은 수집 제외
        stage = group.current_stage
        if stage is None or stage.status != stage.STATUS_ACTIVE:
            continue
        cps = group._get_json_field('checkpoint_ids') or []
        # success_criteria 는 이제 **블록 단위**(키=block_id) — 그대로 머지.
        criteria = stage._get_json_field('success_criteria') or {}
        if isinstance(criteria, dict):
            criteria_by_block.update(criteria)
        # 노이즈 강도는 **직전 stage 의 성공률**로 고정 (현재 stage 의 실시간
        # 성공률을 쓰면 한 stage 안에서 성공할수록 노이즈가 커지는 버그). 최종
        # 노이즈 = success_rate × rate(spec) + offset 이므로, spec 의 rate 가 사용자의
        # "이전 stage 성공률 × N%" 에서 N% 에 해당한다.
        sr = _prev_stage_success_rate(group, stage)
        block_noise = group._get_json_field('block_noise') or {}
        if isinstance(block_noise, dict):
            for bid, spec in block_noise.items():
                block_specs[bid] = {'spec': spec, 'success_rate': sr}
        mission = group._get_json_field('mission') or {}
        save_prob = float(mission.get('failure_save_prob', 1.0)) if isinstance(mission, dict) else 1.0
        for plan_group in plans:
            blocks = plan_group.get('blocks') or []
            plan_cps = {b.get('checkpoint_id') for b in blocks if b.get('type') == 'checkpoint'}
            matched_cps = set(cps) & plan_cps
            if matched_cps:
                pg_id = plan_group['id']
                stage_by_plan_group[pg_id] = stage
                save_prob_by_plan_group[pg_id] = save_prob
                ckpt_group_id_by_plan_group[pg_id] = group.id
                target_cp_by_plan_group.setdefault(pg_id, set()).update(matched_cps)
                target_block_by_plan_group.setdefault(pg_id, set()).update(
                    b.get('id') for b in blocks
                    if b.get('type') == 'checkpoint'
                    and b.get('checkpoint_id') in matched_cps and b.get('id')
                )

    run_plan_groups = [g for g in plans if g['id'] in stage_by_plan_group and g.get('blocks')]
    return {
        'criteria_by_block': criteria_by_block,
        'block_specs': block_specs,
        'stage_by_plan_group': stage_by_plan_group,
        'save_prob_by_plan_group': save_prob_by_plan_group,
        'ckpt_group_id_by_plan_group': ckpt_group_id_by_plan_group,
        'target_cp_by_plan_group': target_cp_by_plan_group,
        'target_block_by_plan_group': target_block_by_plan_group,
        'run_plan_groups': run_plan_groups,
    }


def _any_target_group_training(target_group_ids):
    from ...database.models.checkpoint_group_model import CheckpointGroup as CheckpointGroupModel
    for gid in target_group_ids:
        g = CheckpointGroupModel.find(gid)
        if g and g.status == CheckpointGroupModel.STATUS_TRAINING:
            return True
    return False


def _detect_scene_reset_service(plans):
    """visual_reach 블록의 rgbd_service 로 sim 컨텍스트를 감지해 scene reset
    서비스를 고른다. /tutorial/wrist_rgbd → /tutorial/reset.

    DAgger 롤아웃은 매 iteration 마다 씬을 알려진(관찰 뷰 안) 초기 상태로
    되돌려야 한다 — 안 그러면 직전 iteration 이 peg 를 hole 에 넣어버려(또는
    교정이 집어 든 채로) 다음 wv1 이 peg 를 못 찾고 막힌다. reset(Trigger)은
    arm 을 home keyframe 으로, peg/hole 을 기본 위치로 스냅한다."""
    for g in (plans or []):
        for b in (g.get('blocks') or []):
            if b.get('type') == 'visual_reach':
                svc = (b.get('rgbd_service') or '').strip()
                if svc.startswith('/tutorial'):
                    return '/tutorial/reset'
                prefix = svc.rsplit('/', 1)[0] if '/' in svc else ''
                if prefix:
                    return f'{prefix}/reset'
    return None


def _reset_scene(reset_service):
    """씬 reset 서비스를 gRPC ROSProxy 로 호출. 실패는 무시(비-sim 환경)."""
    if not reset_service:
        return
    try:
        from ...bridge.client import get_bridge_client
        from ...bridge.generated import robot_bridge_pb2 as pb
        client = get_bridge_client()
        client.ros_proxy.CallService(pb.ROSServiceRequest(
            service_type='std_srvs/srv/Trigger', service_name=reset_service,
            request_json=''))
    except Exception as e:
        print(f"[curriculum] scene reset ({reset_service}) skipped: {e}", flush=True)


def _recover_peg(block, group_id):
    """visual_reach 검출 실패 시 peg 를 home±0.05(랜덤·직립)로 되돌린다.

    sim 의 ``<prefix>/randomize_peg`` (Trigger) 를 gRPC ROSProxy 로 호출. prefix 는
    블록의 ``rgbd_service`` 에서 유추(/tutorial/wrist_rgbd → /tutorial). 실패는 무시.
    (peg 가 hole 에 고아로 남아 wv1 이 영구 0px 가 되는 cascade 방지용 자가복구.)"""
    svc = (block.get('rgbd_service') or '/tutorial/wrist_rgbd').strip()
    prefix = svc.rsplit('/', 1)[0] if '/' in svc else '/tutorial'
    service_name = f'{prefix}/randomize_peg'
    try:
        from ...bridge.client import get_bridge_client
        from ...bridge.generated import robot_bridge_pb2 as pb
        client = get_bridge_client()
        resp = client.ros_proxy.CallService(pb.ROSServiceRequest(
            service_type='std_srvs/srv/Trigger', service_name=service_name,
            request_json=''))
        print(f"[curriculum] [{group_id}] visual_reach 실패 → {service_name} "
              f"(peg 자가복구, success={getattr(resp, 'success', '?')})", flush=True)
    except Exception as e:
        print(f"[curriculum] [{group_id}] peg recover ({service_name}) skipped: {e}", flush=True)


# ── 엔트리포인트 ───────────────────────────────────────────────────────────

def curriculum_rollout(curriculum_id, target_group_ids, repeat_count, app,
                       socketio_instance, task_control, force_dagger=False):
    """ProcessManager function task 엔트리.

    target_group_ids: CheckpointGroup.id 리스트 (타겟 체크포인트 그룹).
    repeat_count: 롤아웃 반복 횟수 (<=0 이면 무한, stop 까지).
    force_dagger: True 면 체크포인트 진입 즉시 모델 실행을 건너뛰고 교정(expert
        데모 수집) 단계로 직행 — 약한 모델 부트스트랩용 즉시 DAgger 수집 모드.
    """
    # 체크포인트 실행부(_run_checkpoint_block)가 task_control 채널로 읽는다.
    if force_dagger:
        task_control['force_dagger'] = True
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
    # force_dagger 모드는 체크포인트 모델을 **실행하지 않는다**(진입 즉시 expert 교정).
    # 그런데도 모든 정책을 GPU 에 프리로드하면 8GB GPU 를 YOLOE(visual_reach 검출)·
    # sim 렌더·(곧) 학습과 나눠 써서 CUDA 세그폴트(Ultralytics)로 백엔드가 죽는다.
    # force_dagger 면 프리로드를 건너뛰어 GPU 압박을 줄인다(모델은 어차피 안 씀).
    preloaded = {} if force_dagger else _preload_checkpoints(plans, socketio_instance)

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
            ctx['criteria_by_block'] = tg['criteria_by_block']
            ctx['stage_by_group'] = tg['stage_by_plan_group']
            ctx['target_cp_by_group'] = tg['target_cp_by_plan_group']
            ctx['target_block_by_group'] = tg['target_block_by_plan_group']
            ctx['ckpt_group_id_by_plan_group'] = tg['ckpt_group_id_by_plan_group']

            # iteration 사이 씬 리셋 안 함 — 플래너의 복원 블록(put_peg 등)이 peg 를
            # 되돌리므로 환경이 연속으로 흐른다. peg 가 넘어진 경우에만 standalone
            # peg-watchdog 가 randomize_peg(±0.05, hole 고정) 로 복구한다.
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
