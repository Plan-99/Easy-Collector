"""Planner runner: executes one or more group plans, optionally in parallel.

Each group plan is a list of blocks (joint_position / checkpoint / timesleep).
Groups are partitioned by robot-overlap on the route layer, so parallel
execution is safe — different groups never touch the same robot.

Block types:
    - joint_position: move each robot in a workspace to a given joint configuration
    - checkpoint:     run an inference checkpoint for `duration` seconds
    - timesleep:      pause for `duration` seconds

Stop semantics:
    The runner receives a shared ``task_control`` dict from ProcessManager. When
    the user clicks Stop, ProcessManager sets ``task_control['stop'] = True``.
    Every group worker thread polls this flag and aborts.

WebSocket events (all carry ``group_id`` so the UI can route to the right column):
    - planner_run_start       {total_groups, total_iterations}
    - planner_preload_start   {total}
    - planner_preload_done    {count}
    - planner_group_start     {group_id, total}
    - planner_iteration_start {group_id, iteration, total_iterations}
    - planner_block_start     {group_id, index, total, block_id, type, name, iteration}
    - planner_block_end       {group_id, ..., status, error}
    - planner_group_end       {group_id, status, error}
    - planner_run_end         {status, error}
"""

import os
import time
import threading
import traceback

from .checkpoint_test import checkpoint_test


# 그룹 스레드 별 현재 실행 중인 group_id 를 보관. 핸들러가 ctx 가 아닌 thread-local
# 로 그룹 id 를 읽어가도록 해서 다른 그룹의 동시 실행과 race 가 안 나도록.
_thread_local = threading.local()


def _emit(socketio_instance, event, payload):
    if socketio_instance is None:
        return
    try:
        socketio_instance.emit(event, payload)
    except Exception as e:
        print(f"[ERROR] socketio emit failed ({event}): {e}")


class SyncBarrier:
    """Generation-counted barrier across groups for the planner sync block.

    - ``expected`` is the set of group_ids whose plan contains this sync_id.
    - When the last expected group arrives, generation increments and all
      waiters wake. This lets the same barrier reset cleanly for repeat loops.
    - Waits poll every 200 ms so a global ``task_control['stop']`` releases
      everyone.
    """

    def __init__(self, expected_group_ids):
        self.expected = set(expected_group_ids)
        self.arrived = set()
        self.generation = 0
        self.cond = threading.Condition()

    def wait(self, group_id, task_control):
        if group_id not in self.expected:
            return
        with self.cond:
            self.arrived.add(group_id)
            my_gen = self.generation
            if self.arrived >= self.expected:
                self.arrived = set()
                self.generation += 1
                self.cond.notify_all()
                return
            while self.generation == my_gen and not task_control.get('stop'):
                self.cond.wait(timeout=0.2)

    def drop(self, group_id):
        """Called when a group's worker exits — drop it from this barrier so
        any other group still waiting at this sync_id can be released."""
        with self.cond:
            if group_id not in self.expected:
                return
            self.expected.discard(group_id)
            self.arrived.discard(group_id)
            if self.expected and self.arrived >= self.expected:
                self.arrived = set()
                self.generation += 1
            self.cond.notify_all()


def _compute_sync_barriers(groups):
    """For each sync_id used anywhere, the set of groups whose plan contains it."""
    sync_groups = {}
    for grp in groups:
        for blk in grp.get('blocks') or []:
            if blk.get('type') != 'sync':
                continue
            sid = (blk.get('sync_id') or '').strip()
            if not sid:
                continue
            sync_groups.setdefault(sid, set()).add(grp['id'])
    return {sid: SyncBarrier(gids) for sid, gids in sync_groups.items()}


def _preload_checkpoints(groups, socketio_instance):
    """Load every unique checkpoint referenced across all groups into CPU memory.

    The actual GPU transfer (.cuda()) happens inside ``checkpoint_test`` when
    each block runs; this just front-loads disk read + model construction so
    no group worker stalls mid-plan.

    When 전체 재생 runs N groups in parallel, two policies may share the GPU.
    PyTorch arbitrates kernel scheduling on the default stream, which is fine
    for typical 10Hz ACT/Diffusion inference; if VRAM gets tight, swap in
    explicit ``torch.cuda.Stream`` per group.
    """
    from ...database.models.checkpoint_model import Checkpoint as CheckpointModel
    from ...configs.global_configs import resolve_checkpoint_dir
    from ...policies.utils import make_easytrainer_processors
    from lerobot.policies.act.modeling_act import ACTPolicy
    from lerobot.policies.diffusion.modeling_diffusion import DiffusionPolicy
    import numpy as np
    import torch

    seen = set()
    cache = {}
    ckpt_blocks = []
    for grp in groups:
        for b in grp.get('blocks') or []:
            if b.get('type') == 'checkpoint':
                ckpt_blocks.append(b)
    if not ckpt_blocks:
        return cache

    print(f'[PRELOAD] Found {len(ckpt_blocks)} checkpoint block(s); preloading unique models to CPU')
    _emit(socketio_instance, 'planner_preload_start', {'total': len(ckpt_blocks)})

    for block in ckpt_blocks:
        cid = block.get('checkpoint_id')
        if cid is None or cid in seen:
            continue
        seen.add(cid)

        try:
            ckpt_model = CheckpointModel.find(cid)
            if ckpt_model is None:
                print(f'[PRELOAD][WARN] checkpoint_id={cid} not found, skipping')
                continue
            ckpt = ckpt_model.to_dict()
            policy_obj = ckpt.get('policy')
            if policy_obj is None:
                print(f'[PRELOAD][WARN] checkpoint {cid} has no policy, skipping')
                continue
            ptype = policy_obj.get('type')
            ckpt_dir = resolve_checkpoint_dir(cid)
            print(f'[PRELOAD] Loading checkpoint {cid} ({ptype}) from {ckpt_dir}')
            t0 = time.time()

            if ptype == 'ACT':
                policy = ACTPolicy.from_pretrained(ckpt_dir)
            elif ptype == 'Diffusion':
                policy = DiffusionPolicy.from_pretrained(ckpt_dir)
            elif ptype == 'PI05':
                from lerobot.policies.pi05.modeling_pi05 import PI05Policy
                policy = PI05Policy.from_pretrained(ckpt_dir)
            else:
                print(f'[PRELOAD][WARN] unsupported policy type {ptype} for checkpoint {cid}, skipping')
                continue
            policy.cpu()
            policy.eval()

            preprocessor, postprocessor = make_easytrainer_processors(
                policy_type=ptype,
                cfg=policy.config,
                pretrained_path=ckpt_dir,
            )

            ood = {}
            ood_path = os.path.join(ckpt_dir, 'ood_features.npz')
            if os.path.exists(ood_path) and hasattr(policy, 'enable_feature_caching'):
                ood_data = np.load(ood_path)
                if 'image_features' in ood_data:
                    ood['image_feats'] = torch.from_numpy(ood_data['image_features']).float()
                if 'state_features' in ood_data:
                    ood['state_feats'] = torch.from_numpy(ood_data['state_features']).float()
                if 'image_dist_sorted' in ood_data:
                    ood['image_dist_sorted'] = ood_data['image_dist_sorted']
                if 'state_dist_sorted' in ood_data:
                    ood['state_dist_sorted'] = ood_data['state_dist_sorted']

            cache[cid] = {
                'policy': policy,
                'preprocessor': preprocessor,
                'postprocessor': postprocessor,
                'ood': ood,
            }
            print(f'[PRELOAD] checkpoint {cid} ready ({time.time() - t0:.1f}s)')
        except Exception:
            print(f'[PRELOAD][ERROR] failed to preload checkpoint {cid}: {traceback.format_exc()}')

    _emit(socketio_instance, 'planner_preload_done', {'count': len(cache)})
    return cache


def _resolve_position(positions, robot_id):
    if positions is None:
        return None
    return positions.get(str(robot_id), positions.get(robot_id))


def _run_joint_position(block, ctx, task_control):
    workspace = ctx['workspaces_by_id'].get(block.get('workspace_id'))
    if workspace is None:
        raise RuntimeError(f"workspace_id={block.get('workspace_id')} not found")

    robots = (workspace.get('assembly') or {}).get('robots') or []
    positions = block.get('positions') or {}

    targets = []
    for robot in robots:
        pos = _resolve_position(positions, robot['id'])
        if pos is None:
            continue
        agent = ctx['agents'].get(int(robot['id']))
        if agent is None:
            raise RuntimeError(f"robot '{robot.get('name')}' (id={robot['id']}) is not running — turn it on first")
        targets.append((agent, pos))

    if not targets:
        print(f"[WARN] block '{block.get('name')}' has no joint targets, skipping")
        return

    duration = float(block.get('duration', 5.0))
    for agent, pos in targets:
        agent.move_to(pos, duration=duration)

    timeout_at = time.time() + max(30.0, float(block.get('timeout', 30)))
    agents_only = [a for a, _ in targets]
    try:
        while time.time() < timeout_at:
            if task_control['stop']:
                print('[NOTICE] joint_position interrupted by stop signal')
                for a in agents_only:
                    try:
                        a.cancel_move_to()
                    except Exception as e:
                        print(f"[WARN] cancel_move_to failed: {e}")
                break
            if not any(a.is_moving for a in agents_only):
                break
            time.sleep(0.1)
        else:
            print('[WARNING] joint_position timed out — moving on')
            for a in agents_only:
                try:
                    a.cancel_move_to()
                except Exception:
                    pass
    finally:
        if task_control.get('stop'):
            for a in agents_only:
                try:
                    a.cancel_move_to()
                except Exception:
                    pass


def _is_tool_agent(agent):
    """Tool agents (e.g. standalone gripper) don't have EE / IK — they take an
    absolute joint position instead of an EE delta.
    """
    return getattr(agent, 'role', None) == 'tool' or getattr(agent, 'ik_solver', None) is None


def _run_move_relative_ee(block, ctx, task_control):
    """Move arms by ``deltas[<arm_id>]`` (6-DOF EE delta from current EE pose),
    and tools by ``tool_positions[<tool_id>]`` (absolute joint position).

    Two separate dicts because the semantics differ: arms accept a relative
    EE delta (delta is added to ``get_ee_position()`` at runtime), tools take
    an absolute joint command (no EE — just open/close to a target).

    ``deltas``: per-arm 6 floats [dx, dy, dz, drx, dry, drz].
    ``tool_positions``: per-tool list of absolute joint values (length matches
    the tool's ``joint_names``).

    is_moving polling + cancel pattern mirrors joint_position.
    """
    workspace = ctx['workspaces_by_id'].get(block.get('workspace_id'))
    if workspace is None:
        raise RuntimeError(f"workspace_id={block.get('workspace_id')} not found")

    robots = (workspace.get('assembly') or {}).get('robots') or []
    deltas = block.get('deltas') or {}
    tool_positions = block.get('tool_positions') or {}

    arm_targets = []   # list of (agent, target_ee_dict)
    tool_targets = []  # list of (agent, abs_joint_position)
    for robot in robots:
        agent = ctx['agents'].get(int(robot['id']))
        if agent is None:
            raise RuntimeError(
                f"robot '{robot.get('name')}' (id={robot['id']}) is not running — turn it on first"
            )

        # Diagnostic: 첫 실행 시 한 번씩 agent 의 분기 판정 근거 출력.
        print(
            f"[MoveRelEE] block='{block.get('name')}' robot={robot.get('id')} "
            f"({robot.get('name')}) agent.role={getattr(agent, 'role', None)!r} "
            f"agent.ik_solver={getattr(agent, 'ik_solver', None)!r} "
            f"is_tool={_is_tool_agent(agent)} "
            f"deltas_has={str(robot['id']) in (deltas or {})} "
            f"tool_pos_has={str(robot['id']) in (tool_positions or {})}",
            flush=True,
        )

        if _is_tool_agent(agent):
            pos = tool_positions.get(str(robot['id'])) or tool_positions.get(robot['id'])
            if pos is None:
                continue
            if not isinstance(pos, (list, tuple)):
                raise RuntimeError(
                    f"block '{block.get('name')}' tool_positions for robot "
                    f"{robot['id']} must be a list"
                )
            tool_targets.append((agent, [float(v) for v in pos]))
            continue

        # Arm path
        delta_vec = deltas.get(str(robot['id'])) or deltas.get(robot['id'])
        if delta_vec is None:
            continue
        if not isinstance(delta_vec, (list, tuple)) or len(delta_vec) != 6:
            raise RuntimeError(
                f"block '{block.get('name')}' deltas for robot {robot['id']} "
                f"must be a 6-element list [dx,dy,dz,drx,dry,drz]"
            )
        current_ee = agent.get_ee_position()
        print(
            f"[MoveRelEE] arm robot={robot.get('id')} current_ee={current_ee}",
            flush=True,
        )
        if not current_ee:
            raise RuntimeError(
                f"robot '{robot.get('name')}' returned empty EE pose — make sure IK is configured"
            )
        target_ee = {}
        d = [float(v) for v in delta_vec]
        for ee_name, cur in current_ee.items():
            cur = list(cur)
            # cur 가 [x,y,z,rx,ry,rz] (6) 또는 [x,y,z,rx,ry,rz,gripper] (7) 일 수
            # 있음. 앞 6 dim 에만 delta 더하고 뒤는 그대로 유지.
            new_pose = list(cur)
            for i in range(min(6, len(new_pose))):
                new_pose[i] = float(cur[i]) + d[i]
            target_ee[ee_name] = new_pose
        print(
            f"[MoveRelEE] arm robot={robot.get('id')} delta={delta_vec} "
            f"target_ee={target_ee}",
            flush=True,
        )
        arm_targets.append((agent, target_ee))

    if not arm_targets and not tool_targets:
        print(f"[WARN] block '{block.get('name')}' has no targets, skipping")
        return

    duration = float(block.get('duration', 3.0))
    print(
        f"[MoveRelEE] dispatching arm_targets={len(arm_targets)} "
        f"tool_targets={len(tool_targets)} duration={duration}s",
        flush=True,
    )
    for agent, target in arm_targets:
        agent.move_ee_to(target, duration=duration)
    for agent, pos in tool_targets:
        agent.move_to(pos, duration=duration)

    timeout_at = time.time() + max(30.0, duration + 5.0)
    agents_only = [a for a, _ in arm_targets] + [a for a, _ in tool_targets]
    try:
        while time.time() < timeout_at:
            if task_control['stop']:
                print('[NOTICE] move_relative_ee interrupted by stop signal')
                for a in agents_only:
                    try:
                        a.cancel_move_to()
                    except Exception as e:
                        print(f"[WARN] cancel_move_to failed: {e}")
                break
            if not any(a.is_moving for a in agents_only):
                break
            time.sleep(0.1)
        else:
            print('[WARNING] move_relative_ee timed out — moving on')
            for a in agents_only:
                try:
                    a.cancel_move_to()
                except Exception:
                    pass
    finally:
        if task_control.get('stop'):
            for a in agents_only:
                try:
                    a.cancel_move_to()
                except Exception:
                    pass


def _run_replay_episode(block, ctx, task_control):
    """Replay a recorded episode by streaming its ``action.joint`` (qaction) frame
    by frame to every robot in the workspace. No policy inference — just a
    deterministic playback of the captured trajectory.

    Behavior:
      1. (optional) ``agent.move_to(first_qpos)`` for each robot, wait until
         ``is_moving`` clears, then sleep ``settle_sec``.
      2. Iterate frames 1..N at ``hz`` calling ``agent.move_joint_step(qaction)``.
      3. Stop check every frame; on stop, cancel any in-flight ``move_to``.

    Block keys:
      workspace_id, dataset_id, episode_index, hz, move_to_first, settle_sec.
    """
    from ...utils.lerobot_io import read_episode
    from ...configs.global_configs import DATASET_DIR

    workspace = ctx['workspaces_by_id'].get(block.get('workspace_id'))
    if workspace is None:
        raise RuntimeError(f"workspace_id={block.get('workspace_id')} not found")

    robots = (workspace.get('assembly') or {}).get('robots') or []
    agents_by_id = {}
    for robot in robots:
        agent = ctx['agents'].get(int(robot['id']))
        if agent is None:
            raise RuntimeError(f"robot '{robot.get('name')}' (id={robot['id']}) is not running")
        agents_by_id[int(robot['id'])] = agent

    dataset_id = block.get('dataset_id')
    if dataset_id is None:
        raise RuntimeError("replay_episode: dataset_id is required")
    episode_index = int(block.get('episode_index'))
    hz = float(block.get('hz') or 20)
    if hz <= 0:
        raise RuntimeError(f"replay_episode: invalid hz={hz}")
    move_to_first = bool(block.get('move_to_first', True))
    settle_sec = float(block.get('settle_sec') or 0)

    dataset_dir = os.path.join(DATASET_DIR, str(dataset_id))
    if not os.path.isdir(dataset_dir):
        raise RuntimeError(f"replay_episode: dataset dir not found: {dataset_dir}")

    ep_data = read_episode(dataset_dir, episode_index)
    states_by_robot = ep_data["states"]    # {robot_name: np.array (T, D)}
    actions_by_robot = ep_data["actions"]  # {robot_name: np.array (T, D)}
    num_frames = ep_data["num_frames"]

    # Resolve agents for the dataset's robot_names. "robot_<id>" → agent id.
    # Robots present in the dataset but not in the workspace are skipped (rather
    # than failing) — common when datasets were captured with a superset of
    # robots and the user just wants the matching subset to replay.
    name_to_agent = {}
    for robot_name in actions_by_robot.keys():
        try:
            aid = int(robot_name.replace("robot_", ""))
        except ValueError:
            continue
        if aid in agents_by_id:
            name_to_agent[robot_name] = agents_by_id[aid]

    if not name_to_agent:
        raise RuntimeError(
            "replay_episode: no overlap between episode's robots "
            f"({list(actions_by_robot.keys())}) and workspace's robots "
            f"({list(agents_by_id.keys())})"
        )

    # 1) Move to first qpos.
    if move_to_first:
        for robot_name, agent in name_to_agent.items():
            first_qpos = states_by_robot[robot_name][0].tolist()
            agent.move_to(first_qpos, duration=5.0)
        movers = list(name_to_agent.values())
        timeout = 30.0
        start_wait = time.time()
        while time.time() - start_wait < timeout:
            if task_control['stop']:
                for a in movers:
                    try:
                        a.cancel_move_to()
                    except Exception:
                        pass
                return
            if not any(a.is_moving for a in movers):
                break
            time.sleep(0.05)
        if settle_sec > 0:
            t0 = time.time()
            while time.time() - t0 < settle_sec:
                if task_control['stop']:
                    return
                time.sleep(0.05)

    # 2) Replay frames at hz. i=0 은 record_episode 의 reset 프레임(qaction 의미
    # 없음) 이라 1 부터 시작 — read_dataset.py 와 동일한 규칙.
    period = 1.0 / hz
    next_tick = time.time()
    for i in range(1, num_frames):
        if task_control['stop']:
            return
        for robot_name, agent in name_to_agent.items():
            qaction = actions_by_robot[robot_name][i]
            agent.move_joint_step(qaction)
        next_tick += period
        remaining = next_tick - time.time()
        if remaining > 0.002:
            time.sleep(remaining - 0.002)
        if remaining > 0:
            while time.time() < next_tick:
                pass
        else:
            next_tick = time.time()


def _run_checkpoint(block, ctx, task_control):
    from ...database.models.checkpoint_model import Checkpoint as CheckpointModel

    workspace = ctx['workspaces_by_id'].get(block.get('workspace_id'))
    if workspace is None:
        raise RuntimeError(f"workspace_id={block.get('workspace_id')} not found")

    checkpoint_model = CheckpointModel.find(block.get('checkpoint_id'))
    if checkpoint_model is None:
        raise RuntimeError(f"checkpoint_id={block.get('checkpoint_id')} not found")

    checkpoint = checkpoint_model.to_dict()
    policy = checkpoint.get('policy')
    if policy is None:
        raise RuntimeError(f"checkpoint '{checkpoint.get('name')}' has no associated policy")

    robots = (workspace.get('assembly') or {}).get('robots') or []
    sensors = workspace.get('sensors') or []
    agents = []
    for robot in robots:
        agent = ctx['agents'].get(int(robot['id']))
        if agent is None:
            raise RuntimeError(f"robot '{robot.get('name')}' is not running — turn it on first")
        agents.append(agent)

    duration = float(block.get('duration') or 30)
    until_done = bool(block.get('until_done'))
    # block 단위 home pose 토글: 기본 True (frontend default 와 일치).
    # 켜져 있으면 inference 시작 직전에 한 번 home 으로 이동. 이전엔 episode_len
    # 마다 강제로 home 복귀하는 부수 효과도 있었으나 제거됨 — 사용자가 명시적으로
    # 멈출 때까지 inference 가 자유롭게 흐른다.
    move_homepose = bool(block.get('move_homepose', True))
    try:
        move_homepose_duration = float(block.get('move_homepose_duration') or 5.0)
    except (TypeError, ValueError):
        move_homepose_duration = 5.0
    try:
        move_homepose_settle_sec = float(block.get('move_homepose_settle_sec') or 0.0)
    except (TypeError, ValueError):
        move_homepose_settle_sec = 0.0
    try:
        done_threshold = float(block.get('done_threshold') if block.get('done_threshold') is not None else 0.5)
    except (TypeError, ValueError):
        done_threshold = 0.5
    # 실패 판정 step 한도. 0/None 이면 비활성 (실패 판정 없음). until_done 모드에서
    # done 신호가 안 떨어진 채 이 step 수에 도달하면 실패로 간주.
    max_steps = block.get('max_steps')
    try:
        max_steps = int(max_steps) if max_steps is not None else None
        if max_steps is not None and max_steps <= 0:
            max_steps = None
    except (TypeError, ValueError):
        max_steps = None
    sub_control = {'stop': False, 'done': False, 'done_threshold': done_threshold if until_done else None}
    preloaded = (ctx.get('preloaded') or {}).get(block.get('checkpoint_id'))
    # record 를 넘기면 checkpoint_test 가 매 step 마다 ``record['steps']`` 를 갱신.
    # step-based 실패 판정용.
    record: dict = {}

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
                max_timesteps=int(duration * 100),
                move_homepose=move_homepose,
                move_homepose_duration=move_homepose_duration,
                move_homepose_settle_sec=move_homepose_settle_sec,
                hz=block.get('hz', 10),
                re_inference_steps=block.get('re_inference_steps', 1),
                temporal_ensemble_coeff=block.get('temporal_ensemble_coeff', 0.01),
                action_type=block.get('action_type'),
                preloaded=preloaded,
                record=record,
            )
        except Exception:
            print(f"[ERROR] checkpoint_test failed: {traceback.format_exc()}")

    thread = threading.Thread(target=_runner, name=f"planner_ckpt_{block.get('id')}", daemon=True)
    thread.start()

    deadline = None if until_done else time.time() + duration
    failed = False
    last_emit_step = -1
    block_id = block.get('id')
    group_id = getattr(_thread_local, 'group_id', None)
    socketio = ctx['socketio']
    try:
        while True:
            if task_control['stop']:
                break
            if sub_control.get('done'):
                print(f"[PLANNER] checkpoint block '{block.get('name')}' done signal triggered (threshold={done_threshold})")
                break
            if deadline is not None and time.time() >= deadline:
                break
            cur_step = int(record.get('steps') or 0)
            # UI 가 checkpoint 블록 카드에 "현재 스텝 / max_steps" 뱃지를 보여줄 수
            # 있도록 step 변동분만 emit (불필요한 트래픽 억제).
            if cur_step != last_emit_step:
                last_emit_step = cur_step
                _emit(socketio, 'planner_block_progress', {
                    'group_id': group_id,
                    'block_id': block_id,
                    'step': cur_step,
                    'max_steps': max_steps,
                })
            if max_steps is not None and cur_step >= max_steps:
                failed = True
                print(
                    f"[PLANNER] checkpoint block '{block.get('name')}' failed: reached max_steps={max_steps} "
                    f"without done signal"
                )
                break
            time.sleep(0.1)
    finally:
        sub_control['stop'] = True
        thread.join(timeout=15)
        if thread.is_alive():
            print('[WARNING] checkpoint_test thread did not exit within 15s')

    # 실패 + fallback 블록 지정 → ``_run_group`` 에 "이 id 의 블록으로 jump" 신호.
    # 핸들러 안에서 직접 fallback 을 돌리지 않고 _run_group 이 다음 블록으로
    # 정상 실행하게 해야:
    #   1) UI 가 planner_block_start 를 받아 fallback 블록이 포커스됨
    #   2) fallback 이 끝나면 그 뒤 블록부터 plan 이 자연스럽게 계속 진행됨
    # 사용자가 stop 누른 경우엔 jump 도 트리거하지 않는다.
    if failed and not task_control.get('stop'):
        fb_id = block.get('fallback_block_id')
        fb = (ctx.get('block_by_id') or {}).get(fb_id) if fb_id else None
        if fb:
            print(
                f"[PLANNER] block '{block.get('name')}' failed → jumping to fallback "
                f"'{fb.get('name')}' (id={fb_id})"
            )
            _thread_local.jump_to_block_id = fb_id
        else:
            print(
                f"[WARNING] checkpoint block '{block.get('name')}' failed but no fallback "
                f"resolvable (fallback_block_id={block.get('fallback_block_id')!r})"
            )


def _run_timesleep(block, ctx, task_control):
    duration = float(block.get('duration') or 0)
    if duration <= 0:
        return
    deadline = time.time() + duration
    while time.time() < deadline:
        if task_control['stop']:
            return
        time.sleep(0.1)


ASSEMBLY_SLOTS = ('left_arm', 'right_arm', 'left_tool', 'right_tool', 'mobile_base')


def _assembly_slot_map(workspace):
    """Map filled assembly slot names → robot dict for a workspace.

    A workspace's assembly is a *combination* of robots in named slots
    (left_arm, left_tool, ...), so query_pose targets are keyed by slot rather
    than being a single flat list.
    """
    assembly = workspace.get('assembly') or {}
    slot_map = {}
    for slot in ASSEMBLY_SLOTS:
        robot = assembly.get(slot)
        if robot:
            slot_map[slot] = robot
    return slot_map


def _run_query_pose(block, ctx, task_control):
    """Query a ROS service at execution time for target poses, then drive the
    workspace's assembly robots to those poses over ``duration`` seconds.

    Service: ``std_srvs/srv/Trigger`` (stock ROS — no custom build needed on
    the external node side). The server JSON-encodes the target in the
    response ``message`` field, keyed by assembly slot
    (left_arm / right_arm / left_tool / right_tool / mobile_base). Only the
    slots present in the response are moved.

    Joint mode payload (response.message):
        {"positions": {
            "left_arm":  [j1, j2, ..., gripper],   # every joint of that robot
            "left_tool": [g1]
        }}

    End-effector mode payload (response.message):
        {"poses": {
            "left_arm": {"position": [x, y, z],
                         "orientation": [rx, ry, rz],   # Euler radians
                         "gripper": <value>}            # optional
        }}
    """
    import json
    from ...bridge.client import get_bridge_client
    from ...bridge.generated import robot_bridge_pb2 as pb

    workspace = ctx['workspaces_by_id'].get(block.get('workspace_id'))
    if workspace is None:
        raise RuntimeError(f"workspace_id={block.get('workspace_id')} not found")

    service_name = (block.get('service_name') or '').strip()
    if not service_name:
        raise RuntimeError("service_name is empty")

    pose_type = block.get('pose_type')
    if pose_type not in ('joint_position', 'end_effector_position'):
        raise RuntimeError(f"invalid pose_type: {pose_type}")

    duration = float(block.get('duration') or 5.0)
    slot_map = _assembly_slot_map(workspace)
    if not slot_map:
        print(f"[WARN] query_pose block '{block.get('name')}' has no robots in workspace, skipping")
        return

    def _resolve(slot_key):
        """slot name from the service response → (robot dict, agent)."""
        robot = slot_map.get(slot_key)
        if robot is None:
            raise RuntimeError(
                f"service '{service_name}' returned slot '{slot_key}', but the workspace "
                f"assembly has no such slot (available: {sorted(slot_map)})"
            )
        agent = ctx['agents'].get(int(robot['id']))
        if agent is None:
            raise RuntimeError(
                f"robot '{robot.get('name')}' (slot '{slot_key}') is not running"
            )
        return robot, agent

    client = get_bridge_client()
    print(f"[QUERY_POSE] calling service '{service_name}' (pose_type={pose_type})")
    resp = client.ros_proxy.CallService(pb.ROSServiceRequest(
        service_type='std_srvs/srv/Trigger',
        service_name=service_name,
        request_json='',
    ))
    if not resp.success:
        raise RuntimeError(f"service '{service_name}' transport failure: {resp.response_json}")

    # ros_proxy wraps the srv response as {"success": ..., "message": ...}.
    try:
        outer = json.loads(resp.response_json)
    except (TypeError, ValueError) as e:
        raise RuntimeError(f"service '{service_name}' response is not valid JSON: {e}")

    if isinstance(outer, dict) and outer.get('success') is False:
        raise RuntimeError(
            f"service '{service_name}' reported failure: {outer.get('message') or 'no message'}"
        )

    message_str = outer.get('message') if isinstance(outer, dict) else None
    if not isinstance(message_str, str) or not message_str:
        raise RuntimeError(
            f"service '{service_name}' response.message is empty — expected a JSON-encoded pose"
        )
    try:
        payload = json.loads(message_str)
    except (TypeError, ValueError) as e:
        raise RuntimeError(f"service '{service_name}' response.message is not valid JSON: {e}")
    if not isinstance(payload, dict):
        raise RuntimeError(f"service '{service_name}' payload is not an object: {payload!r}")

    if pose_type == 'joint_position':
        positions = payload.get('positions')
        if not isinstance(positions, dict) or not positions:
            raise RuntimeError(
                f"service '{service_name}' must return "
                '{"positions": {"<slot>": [j1, j2, ...]}} keyed by assembly slot '
                f"(got: {payload!r})"
            )

        targets = []
        for slot_key, joints in positions.items():
            if not isinstance(joints, list) or not joints:
                raise RuntimeError(
                    f"service '{service_name}' slot '{slot_key}' has no joint values"
                )
            robot, agent = _resolve(slot_key)
            # The response must carry every joint the robot has (including the
            # gripper); a count mismatch makes MoveTo a silent no-op.
            current = agent.get_joint_states()
            if current is not None and len(current) != len(joints):
                raise RuntimeError(
                    f"service '{service_name}' slot '{slot_key}' returned {len(joints)} joint "
                    f"values, but robot '{robot.get('name')}' has {len(current)} joints — "
                    f"include every joint (gripper included)"
                )
            targets.append((agent, [float(j) for j in joints]))

        for agent, pos in targets:
            agent.move_to(pos, duration=duration)

        agents_only = [a for a, _ in targets]
        timeout_at = time.time() + max(duration + 5.0, 30.0)
        try:
            while time.time() < timeout_at:
                if task_control['stop']:
                    print('[NOTICE] query_pose interrupted by stop signal')
                    for a in agents_only:
                        try:
                            a.cancel_move_to()
                        except Exception as e:
                            print(f"[WARN] cancel_move_to failed: {e}")
                    break
                if not any(a.is_moving for a in agents_only):
                    break
                time.sleep(0.1)
            else:
                print('[WARNING] query_pose timed out — moving on')
                for a in agents_only:
                    try:
                        a.cancel_move_to()
                    except Exception:
                        pass
        finally:
            if task_control.get('stop'):
                for a in agents_only:
                    try:
                        a.cancel_move_to()
                    except Exception:
                        pass
        return payload

    # end_effector_position: per-slot duration-based move_ee_to (IK + interpolated).
    poses = payload.get('poses')
    if not isinstance(poses, dict) or not poses:
        raise RuntimeError(
            f"service '{service_name}' must return "
            '{"poses": {"<slot>": {"position": [...], "orientation": [...]}}} '
            f"keyed by assembly slot (got: {payload!r})"
        )

    ee_targets = []
    for slot_key, pose in poses.items():
        if not isinstance(pose, dict):
            raise RuntimeError(
                f"service '{service_name}' slot '{slot_key}' pose is not an object: {pose!r}"
            )
        position = pose.get('position') or []
        orientation = pose.get('orientation') or []
        if len(position) != 3:
            raise RuntimeError(
                f"service '{service_name}' slot '{slot_key}' position must have 3 values "
                f"(got {position!r})"
            )
        if len(orientation) != 3:
            raise RuntimeError(
                f"service '{service_name}' slot '{slot_key}' orientation must have 3 values "
                f"(got {orientation!r})"
            )
        robot, agent = _resolve(slot_key)

        # move_ee_to expects {<ee_name>: [x, y, z, r, p, y, tool]} keyed by the
        # robot's IK end-effector name (e.g. 'L_ee'/'ee'), NOT a {x, y, z, ...}
        # dict. Discover the actual key from the live EE state.
        ee_state = agent.get_ee_position()
        if not isinstance(ee_state, dict) or not ee_state:
            raise RuntimeError(
                f"robot '{robot.get('name')}' (slot '{slot_key}') does not support "
                f"end-effector control"
            )
        if len(ee_state) > 1:
            raise RuntimeError(
                f"robot '{robot.get('name')}' (slot '{slot_key}') has multiple "
                f"end-effectors ({sorted(ee_state)}) — EE-mode Query Pose supports "
                f"single-EE robots only"
            )
        ee_key = next(iter(ee_state))
        target_ee = {ee_key: [
            float(position[0]), float(position[1]), float(position[2]),
            float(orientation[0]), float(orientation[1]), float(orientation[2]),
            float(pose.get('gripper', 0.0)),
        ]}
        ee_targets.append((robot, agent, slot_key, target_ee))

    for robot, agent, slot_key, target_ee in ee_targets:
        try:
            agent.move_ee_to(target_ee, duration=duration)
        except Exception as e:
            raise RuntimeError(
                f"move_ee_to for '{robot.get('name')}' (slot '{slot_key}') failed: {e}"
            )

    agents_only = [a for _, a, _, _ in ee_targets]
    timeout_at = time.time() + max(duration + 5.0, 30.0)
    try:
        while time.time() < timeout_at:
            if task_control['stop']:
                print('[NOTICE] query_pose interrupted by stop signal')
                for a in agents_only:
                    try:
                        a.cancel_move_to()
                    except Exception as e:
                        print(f"[WARN] cancel_move_to failed: {e}")
                break
            if not any(a.is_moving for a in agents_only):
                break
            time.sleep(0.1)
        else:
            print('[WARNING] query_pose timed out — moving on')
            for a in agents_only:
                try:
                    a.cancel_move_to()
                except Exception:
                    pass
    finally:
        if task_control.get('stop'):
            for a in agents_only:
                try:
                    a.cancel_move_to()
                except Exception:
                    pass
    return payload


def _run_sync(block, ctx, task_control, group_id):
    sync_id = (block.get('sync_id') or '').strip()
    if not sync_id:
        return
    barrier = (ctx.get('sync_barriers') or {}).get(sync_id)
    # 단일 그룹만 이 sync_id를 가지면 (또는 워크스페이스가 1개라 그룹이 하나뿐이면)
    # barrier.expected 크기가 1 → wait()이 즉시 통과.
    if barrier is None:
        return
    print(f"[PLANNER] [{group_id}] sync '{sync_id}' waiting "
          f"(expected={sorted(barrier.expected)})")
    barrier.wait(group_id, task_control)
    print(f"[PLANNER] [{group_id}] sync '{sync_id}' passed")


def _visual_reach_decode_rgb(jpeg_b64):
    """Decode a base64 JPEG to an RGB ndarray (H,W,3). Tries cv2 then PIL."""
    import base64
    import numpy as np
    raw = base64.b64decode(jpeg_b64)
    try:
        import cv2
        arr = cv2.imdecode(np.frombuffer(raw, np.uint8), cv2.IMREAD_COLOR)  # BGR
        return arr[..., ::-1].copy()  # -> RGB
    except Exception:
        import io
        from PIL import Image
        return np.asarray(Image.open(io.BytesIO(raw)).convert("RGB"))


# color word → representative RGB (for the no-SAM3 color fallback)
_VR_COLORS = {
    'white': (235, 235, 235), 'red': (200, 40, 40), 'blue': (40, 80, 210),
    'green': (40, 200, 80), 'yellow': (220, 210, 40), 'black': (25, 25, 25),
    'gray': (140, 140, 140), 'grey': (140, 140, 140), 'orange': (230, 120, 30),
    'purple': (150, 40, 180),
}
# geometric primitives tried (prefixed by the prompt's color word) when SAM3
# misses the literal prompt — synthetic/abstract objects often read as a shape
# (e.g. the tutorial's flat white plate reads as a "circle" to SAM3).
_VR_SHAPES = ['circle', 'disc', 'object', 'disk', 'ring', 'box', 'square', 'plate', 'ball']


def _vr_sam3_mask(runner, rgb, text_list, boxes, H, W):
    import numpy as np
    m = runner.detect_one(rgb[..., ::-1].copy(),
                          {'text': list(text_list), 'boxes': [list(b) for b in boxes]})
    m = np.asarray(m).astype(bool)
    return m if (m.shape == (H, W) and m.any()) else None


def _visual_reach_mask(rgb, text_prompts, boxes, block):
    """Return a bool mask (H,W) of the target. Wrist View Reach uses YOLOE only
    (SAM3 is not used here): a dragged box exemplar (view-independent cross-view
    match) or an open-vocabulary text prompt. Falls back to a color threshold
    only when YOLOE isn't installed. `rgb` is HxWx3 uint8 RGB."""
    import numpy as np
    H, W = rgb.shape[:2]
    joined = ' '.join(text_prompts).lower() if text_prompts else ''
    colors_in = [c for c in _VR_COLORS if c in joined]

    refer_b64 = block.get('exemplar_image')
    refer_box = block.get('exemplar_box')
    if (refer_b64 and refer_box) or text_prompts:
        from ...utils import yoloe_helper
        if not yoloe_helper.is_extension_installed():
            raise RuntimeError(
                "Wrist View Reach 검출에는 YOLOE 확장이 필요합니다. "
                "모듈 관리에서 'YOLOE Visual-Prompt Detection'을 설치하세요.")
        # box exemplar takes precedence (it pins a specific instance); else text.
        if refer_b64 and refer_box:
            refer_rgb = _visual_reach_decode_rgb(refer_b64)
            return yoloe_helper.detect_exemplar(rgb, refer_rgb, refer_box)
        return yoloe_helper.detect_text(rgb, text_prompts)

    # No YOLOE prompt set → manual box union, else color threshold (legacy fallback).
    if boxes:
        mask = np.zeros((H, W), bool)
        for b in boxes:
            x1, y1, x2, y2 = [int(round(v)) for v in b[:4]]
            mask[max(0, min(y1, y2)):max(0, max(y1, y2)),
                 max(0, min(x1, x2)):max(0, max(x1, x2))] = True
        if mask.any():
            return mask
    r, g, bl = rgb[..., 0].astype(int), rgb[..., 1].astype(int), rgb[..., 2].astype(int)
    col = block.get('target_color')
    if not col and colors_in:
        col = list(_VR_COLORS[colors_in[0]])
    if col and len(col) == 3:
        cr, cg, cb = col
        return (np.abs(r - cr) + np.abs(g - cg) + np.abs(bl - cb)) < 90
    return (r > 110) & (r - g > 50) & (r - bl > 50)


def _resolve_rgbd_service(block):
    """Resolve the wrist RGB-D Trigger service for a visual_reach block:
      1) explicit block.rgbd_service (sim/custom), else
      2) the selected sensor's settings.rgbd_service (e.g. tutorial_wrist_cam →
         /tutorial/wrist_rgbd), else
      3) a real use_depth RealSense → /ec_sensor_<id>/wrist_rgbd, else
      4) the sim default."""
    svc = (block.get('rgbd_service') or '').strip()
    if svc:
        return svc
    sid = block.get('sensor_id')
    if sid in (None, ''):
        return '/tutorial/wrist_rgbd'
    try:
        from ...database.models.sensor_model import Sensor
        s = Sensor.find(int(sid))
        if s is not None:
            sset = s._settings or {}
            if sset.get('rgbd_service'):
                return sset['rgbd_service']
            if s.use_depth:
                return f'/ec_sensor_{sid}/wrist_rgbd'
    except Exception:
        pass
    return f'/ec_sensor_{sid}/wrist_rgbd'


def _mask_overlay_b64(mask):
    """Build a TRANSPARENT overlay PNG showing only the segmentation mask (no RGB) —
    a semi-transparent green fill where the mask is, plus a centroid marker, and full
    alpha=0 everywhere else. Composited over the live stream it shows just the mask,
    not a second copy of the image. Returns (base64_png, centroid|None)."""
    import base64
    import numpy as np
    m = np.asarray(mask).astype(bool)
    H, W = m.shape
    centroid = None
    bgra = np.zeros((H, W, 4), np.uint8)          # transparent background
    bgra[m] = (40, 220, 40, 140)                   # BGRA: green, semi-transparent
    if m.any():
        ys, xs = np.where(m)
        centroid = [int(xs.mean()), int(ys.mean())]
    try:
        import cv2
        if centroid is not None:
            cv2.circle(bgra, (centroid[0], centroid[1]), 9, (60, 60, 255, 255), -1)  # opaque red dot
        ok, buf = cv2.imencode('.png', bgra)       # cv2 PNG keeps the alpha channel
        img_b64 = base64.b64encode(buf.tobytes()).decode('ascii') if ok else None
    except Exception:
        import io
        from PIL import Image as _PILImage
        rgba = bgra[..., [2, 1, 0, 3]]              # BGRA -> RGBA for PIL
        b = io.BytesIO(); _PILImage.fromarray(rgba, 'RGBA').save(b, 'PNG')
        img_b64 = base64.b64encode(b.getvalue()).decode('ascii')
    return img_b64, centroid


def _emit_visual_reach_mask(socketio, sensor_id, mask):
    """Emit the detected target mask (transparent, mask-only) so the live monitoring
    view overlays it on the selected wrist sensor's stream (socket 'planner_wrist_mask',
    payload {sensor_id, image:data-url}). Best-effort; never blocks the reach."""
    if socketio is None or sensor_id in (None, ''):
        return
    img_b64, _ = _mask_overlay_b64(mask)
    if img_b64:
        _emit(socketio, 'planner_wrist_mask', {
            'sensor_id': int(sensor_id),
            'image': f'data:image/png;base64,{img_b64}',
        })


def _visual_reach_backproject(payload, mask, block, ee_pose=None):
    """Shared pixel→world back-projection for Wrist View Reach. Used by both the
    block handler AND the dialog's "test detect" so they can NEVER diverge — the
    same geometry that drives the robot is what the calibration readout shows.

    payload : the RGBD service payload (height/width/depth_f32_b64/intrinsics[/cam pose]).
    mask    : HxW bool target mask (already computed from the RGB).
    block   : block dict (cam_pose_mode, cam_offset, cam_pitch/yaw/roll, cam_convention).
    ee_pose : live EE pose [x,y,z,ax,ay,az] (rotvec, robot base frame) for 'manual_ee'.
              Required for manual mode; ignored for 'service' mode.

    Returns dict {target_xyz, centroid:[u,v], depth, cam_pos, ee_pos, convention,
    cam_mode}, or None if the target has < 8 pixels with valid depth.
    """
    import base64
    import numpy as np
    H, W = int(payload['height']), int(payload['width'])
    depth = np.frombuffer(base64.b64decode(payload['depth_f32_b64']), np.float32).reshape(H, W)
    m = np.asarray(mask).astype(bool)
    valid = m & np.isfinite(depth) & (depth > 1e-3)
    ys, xs = np.where(valid)
    if len(xs) < 8:
        return None
    u, v = float(xs.mean()), float(ys.mean())
    z = float(np.median(depth[valid]))
    if payload.get('fx'):
        fx = float(payload['fx']); fy = float(payload.get('fy') or fx)
        cx = float(payload.get('cx') if payload.get('cx') is not None else (W - 1) / 2.0)
        cy = float(payload.get('cy') if payload.get('cy') is not None else (H - 1) / 2.0)
    else:
        fovy = float(payload['fovy'])
        fx = fy = 0.5 * H / np.tan(np.deg2rad(fovy) * 0.5)
        cx, cy = (W - 1) / 2.0, (H - 1) / 2.0
    conv = block.get('cam_convention') or ('optical' if payload.get('fx') else 'opengl')
    if conv == 'optical':
        p_cam = np.array([(u - cx) / fx * z, (v - cy) / fy * z, z])
    else:
        p_cam = np.array([(u - cx) / fx * z, -(v - cy) / fy * z, -z])

    cam_mode = block.get('cam_pose_mode') or (
        'manual_ee' if block.get('cam_offset') is not None else 'service')
    ee_pos_out = None
    if cam_mode == 'manual_ee':
        from scipy.spatial.transform import Rotation as _Rot
        if not ee_pose:
            raise RuntimeError("visual_reach manual pose: empty EE pose (IK not configured?)")
        ee = [float(v) for v in ee_pose]
        ee_pos = np.array(ee[:3], dtype=float)
        R_ee = _Rot.from_rotvec(ee[3:6]).as_matrix() if len(ee) >= 6 else np.eye(3)
        offset = np.array(([float(o) for o in (block.get('cam_offset') or [])] + [0, 0, 0])[:3])
        pitch = float(block.get('cam_pitch') or 0.0)
        yaw = float(block.get('cam_yaw') or 0.0)
        roll = float(block.get('cam_roll') or 0.0)
        R_ee_cam = _Rot.from_euler('xyz', [pitch, yaw, roll], degrees=True).as_matrix()
        cam_pos = ee_pos + R_ee @ offset
        R = R_ee @ R_ee_cam
        target_xyz = cam_pos + R @ p_cam
        ee_pos_out = ee_pos.round(4).tolist()
    else:
        cam_pos = np.array(payload['cam_pos'], dtype=float)
        R = np.array(payload['cam_mat'], dtype=float).reshape(3, 3)
        target_xyz = cam_pos + R @ p_cam
    return {
        'target_xyz': target_xyz.round(4).tolist(),
        'centroid': [u, v],
        'depth': z,
        'cam_pos': cam_pos.round(4).tolist(),
        'ee_pos': ee_pos_out,
        'convention': conv,
        'cam_mode': cam_mode,
    }


def _run_visual_reach(block, ctx, task_control):
    """`visual_reach` block: move to an observe pose, read the wrist RGB-D, locate the
    target (SAM3 text/box if installed, else box/color), back-project its centroid via
    depth to world XYZ, and move the end-effector to hover above it.

    block keys: workspace_id, sensor_id, rgbd_service (default '/tutorial/wrist_rgbd'),
      text_prompts[], boxes[], target_color[], observe_positions{robot_id:[joints]},
      hover (m, default 0.06), duration (s), settle_sec, name.
    Sim-first (M1-proven geometry). Real cameras: a matching RGBD service/topic is needed.
    """
    import json
    import numpy as np
    from ...bridge.client import get_bridge_client
    from ...bridge.generated import robot_bridge_pb2 as pb

    workspace = ctx['workspaces_by_id'].get(block.get('workspace_id'))
    if workspace is None:
        raise RuntimeError(f"workspace_id={block.get('workspace_id')} not found")
    robots = (workspace.get('assembly') or {}).get('robots') or []

    robot = agent = None
    for rb in robots:
        ag = ctx['agents'].get(int(rb['id']))
        if ag is None:
            raise RuntimeError(f"robot '{rb.get('name')}' (id={rb['id']}) is not running — turn it on first")
        if not _is_tool_agent(ag):
            robot, agent = rb, ag
            break
    if agent is None:
        raise RuntimeError("visual_reach needs an arm (IK) agent in the workspace")

    duration = float(block.get('duration') or 4.0)

    # 1) observe pose (R1) — optional preset joint positions
    observe = block.get('observe_positions') or {}
    obs_pos = observe.get(str(robot['id'])) or observe.get(robot['id'])
    if obs_pos:
        agent.move_to([float(v) for v in obs_pos], duration=duration)
        _visual_reach_wait([agent], task_control, duration)
    time.sleep(float(block.get('settle_sec') or 0.5))
    if task_control.get('stop'):
        return

    # 2) fetch wrist RGB-D via a ROS2 Trigger (gRPC ROSProxy). Source priority:
    #    explicit rgbd_service (sim/custom) → a real use_depth sensor's bridge
    #    service /ec_sensor_<id>/wrist_rgbd → the sim default.
    service_name = _resolve_rgbd_service(block)
    client = get_bridge_client()
    resp = client.ros_proxy.CallService(pb.ROSServiceRequest(
        service_type='std_srvs/srv/Trigger', service_name=service_name, request_json=''))
    if not resp.success:
        raise RuntimeError(f"wrist RGBD service '{service_name}' transport failure: {resp.response_json}")
    outer = json.loads(resp.response_json)
    payload = json.loads(outer.get('message') or '{}')
    if not payload.get('ok'):
        raise RuntimeError(f"wrist RGBD unavailable: {payload.get('error')}")

    rgb = _visual_reach_decode_rgb(payload['rgb_jpeg_b64'])

    # 3) target mask → centroid → robust depth → world XYZ (shared back-projection,
    #    identical to the dialog's calibration readout — see _visual_reach_backproject).
    mask = _visual_reach_mask(rgb, block.get('text_prompts') or [], block.get('boxes') or [], block)
    # Show what SAM3 detected on the live wrist stream (overlay on the selected sensor).
    try:
        _emit_visual_reach_mask(ctx.get('socketio'), block.get('sensor_id'), mask)
    except Exception as _e:
        print(f"[visual_reach] mask overlay emit skipped: {_e}", flush=True)
    ee_pose = None
    cam_mode = block.get('cam_pose_mode') or (
        'manual_ee' if block.get('cam_offset') is not None else 'service')
    if cam_mode == 'manual_ee':
        ee_poses = agent.get_ee_position() or {}
        if not ee_poses:
            raise RuntimeError("visual_reach manual pose: empty EE pose (IK not configured?)")
        ee_pose = [float(v) for v in next(iter(ee_poses.values()))]
    bp = _visual_reach_backproject(payload, mask, block, ee_pose=ee_pose)
    if bp is None:
        raise RuntimeError("visual_reach: target not found in wrist view (no matching pixels with valid depth)")
    target_xyz = np.array(bp['target_xyz'], dtype=float)
    u, v = bp['centroid']
    z = bp['depth']
    print(f"[visual_reach] mode={bp['cam_mode']} conv={bp['convention']} ee_pos={bp['ee_pos']} "
          f"cam_pos={bp['cam_pos']} centroid=({u:.1f},{v:.1f}) depth={z:.3f} "
          f"target_xyz={bp['target_xyz']}", flush=True)

    # 4) move EE to hover above target (keep current/observe orientation)
    hover = float(block.get('hover') or 0.06)
    current_ee = agent.get_ee_position()
    if not current_ee:
        raise RuntimeError("visual_reach: empty EE pose — IK not configured?")
    target_ee = {}
    for ee_name, cur in current_ee.items():
        new_pose = [float(x) for x in cur]
        new_pose[0], new_pose[1] = float(target_xyz[0]), float(target_xyz[1])
        if len(new_pose) > 2:
            new_pose[2] = float(target_xyz[2] + hover)
        target_ee[ee_name] = new_pose
    _emit(ctx.get('socketio'), 'planner_visual_reach', {
        'block': block.get('name'), 'target': target_xyz.tolist(),
        'centroid': [u, v], 'depth': z, 'hover': hover,
    })
    agent.move_ee_to(target_ee, duration=duration)
    _visual_reach_wait([agent], task_control, duration)


def _visual_reach_wait(agents, task_control, duration):
    """Block until agents stop moving (or stop signal / timeout). Mirrors the
    move_relative_ee wait+cancel pattern."""
    timeout_at = time.time() + max(30.0, duration + 5.0)
    while time.time() < timeout_at:
        if task_control.get('stop'):
            for a in agents:
                try:
                    a.cancel_move_to()
                except Exception:
                    pass
            return
        if not any(a.is_moving for a in agents):
            return
        time.sleep(0.1)
    for a in agents:
        try:
            a.cancel_move_to()
        except Exception:
            pass


_HANDLERS = {
    'joint_position': _run_joint_position,
    'move_relative_ee': _run_move_relative_ee,
    'visual_reach': _run_visual_reach,
    'replay_episode': _run_replay_episode,
    'checkpoint': _run_checkpoint,
    'timesleep': _run_timesleep,
    'query_pose': _run_query_pose,
    # 'sync' is dispatched separately in _run_group because it needs group_id;
    # the lookup just has to not return None so the unknown-type guard passes.
    'sync': _run_sync,
}


def _run_group(group, ctx, task_control, repeat_count, infinite, total_iterations):
    """Execute a single group's plan in this thread. Returns ``(status, error)``."""
    socketio_instance = ctx['socketio']
    group_id = group['id']
    # 핸들러가 emit 할 때 group_id 를 알 수 있도록 thread-local 에 적재.
    _thread_local.group_id = group_id
    _thread_local.jump_to_block_id = None
    plan = list(group.get('blocks') or [])
    total = len(plan)
    # block_id → index 매핑 — fallback jump 시 빠르게 위치 찾기 위함.
    id_to_index = {b.get('id'): i for i, b in enumerate(plan) if b.get('id')}

    _emit(socketio_instance, 'planner_group_start', {
        'group_id': group_id,
        'total': total,
        'total_iterations': total_iterations,
    })

    iteration = 0
    while infinite or iteration < repeat_count:
        if task_control['stop']:
            return 'stopped', None

        iteration += 1
        # 매 iteration 시작 시 jump flag 초기화 — 직전 iteration 의 fallback 신호가
        # 새 iteration 으로 새지 않도록.
        _thread_local.jump_to_block_id = None
        _emit(socketio_instance, 'planner_iteration_start', {
            'group_id': group_id,
            'iteration': iteration,
            'total_iterations': total_iterations,
        })

        # for 가 아닌 while 로 — fallback jump 처리 시 index 를 자유롭게 옮기기
        # 위해. 한 iteration 안에서 같은 블록을 두 번 실행하는 무한 루프 방지를
        # 위해 visited 도 두지는 않음 (사용자가 repeat / 무한 모드를 의도적으로
        # 쓸 수 있고, 같은 블록 재실행 자체는 정상 시나리오).
        index = 0
        while index < total:
            if task_control['stop']:
                return 'stopped', None

            block = plan[index]
            handler = _HANDLERS.get(block.get('type'))
            block_summary = {
                'group_id': group_id,
                'index': index,
                'total': total,
                'iteration': iteration,
                'total_iterations': total_iterations,
                'block_id': block.get('id'),
                'type': block.get('type'),
                'name': block.get('name'),
            }
            _emit(socketio_instance, 'planner_block_start', block_summary)
            print(f"[NOTICE] [{group_id}] [{index + 1}/{total}] {block.get('type')} — {block.get('name')}")

            block_status = 'finished'
            block_error = None
            try:
                if handler is None:
                    raise RuntimeError(f"unknown block type '{block.get('type')}'")
                if block.get('type') == 'sync':
                    _run_sync(block, ctx, task_control, group_id)
                else:
                    handler(block, ctx, task_control)
                if task_control['stop']:
                    block_status = 'stopped'
            except Exception as e:
                block_status = 'error'
                block_error = str(e)
                print(f"[ERROR] [{group_id}] block '{block.get('name')}' failed: {traceback.format_exc()}")

            _emit(socketio_instance, 'planner_block_end', {
                **block_summary,
                'status': block_status,
                'error': block_error,
            })

            if block_status == 'error':
                return 'error', block_error
            if block_status == 'stopped':
                return 'stopped', None

            # 핸들러가 fallback jump 신호를 남겼는지 확인. 있으면 그 블록의
            # plan 내 index 로 점프 → 그 다음 iteration step 에서 fallback 이
            # 정상 블록처럼 실행되고 (block_start emit), 끝나면 그 뒤 블록부터
            # plan 이 자연스럽게 이어진다.
            jump_id = getattr(_thread_local, 'jump_to_block_id', None)
            if jump_id:
                _thread_local.jump_to_block_id = None
                target = id_to_index.get(jump_id)
                if target is not None:
                    index = target
                    continue
                print(
                    f"[WARNING] [{group_id}] jump target block_id={jump_id!r} not in "
                    f"this group's plan — continuing sequentially"
                )

            index += 1

    return 'finished', None


def planner_run(groups, workspaces, app, socketio_instance, task_control,
                repeat_count=1):
    """Execute one or more group plans in parallel threads.

    groups: list[dict] — each ``{ id, workspace_ids, blocks }``.
    workspaces: list[dict] — task dicts (with assembly+sensors), referenced by blocks.
    repeat_count:
      - >= 1: each group repeats its plan that many times
      - <= 0: infinite loop until stopped
    """
    groups = [g for g in (groups or []) if g.get('blocks')]
    total_groups = len(groups)

    try:
        repeat_count = int(repeat_count)
    except (TypeError, ValueError):
        repeat_count = 1
    infinite = repeat_count <= 0
    total_iterations = None if infinite else repeat_count

    print(f"[NOTICE] Planner run started ({total_groups} group(s), "
          f"{'infinite loop' if infinite else f'{repeat_count} iteration(s)'})")
    _emit(socketio_instance, 'planner_run_start', {
        'total_groups': total_groups,
        'group_ids': [g['id'] for g in groups],
        'total_iterations': total_iterations,
    })

    preloaded = _preload_checkpoints(groups, socketio_instance)
    sync_barriers = _compute_sync_barriers(groups)
    if sync_barriers:
        print(f"[PLANNER] sync barriers: "
              f"{ {sid: sorted(b.expected) for sid, b in sync_barriers.items()} }")

    # fallback_block_id 로 다른 블록을 참조해 실패 시 디스패치할 수 있도록 모든
    # 블록의 id→block 매핑을 미리 만들어 ctx 에 넣는다. (curriculum_rollout 의
    # block_by_id 와 동일 패턴.)
    block_by_id = {}
    for grp in groups:
        for blk in grp.get('blocks') or []:
            if blk.get('id'):
                block_by_id[blk['id']] = blk

    ctx = {
        'app': app,
        'agents': getattr(app, 'agents', {}),
        'socketio': socketio_instance,
        'workspaces_by_id': {ws['id']: ws for ws in workspaces},
        'preloaded': preloaded,
        'sync_barriers': sync_barriers,
        'block_by_id': block_by_id,
    }

    # 그룹별 상태 수집. 한 그룹이 error여도 다른 그룹은 자기 일을 끝내도록 둠
    # (단, stop은 모두에게 즉시 전파됨).
    group_results = {}

    def _worker(group):
        try:
            status, error = _run_group(group, ctx, task_control, repeat_count, infinite, total_iterations)
        except Exception as e:
            status, error = 'error', str(e)
            print(f"[ERROR] group {group.get('id')} crashed: {traceback.format_exc()}")
        finally:
            # Drop this group from every sync barrier so any peer still
            # waiting at a sync_id can move on.
            for barrier in (ctx.get('sync_barriers') or {}).values():
                barrier.drop(group['id'])
        group_results[group['id']] = (status, error)
        _emit(socketio_instance, 'planner_group_end', {
            'group_id': group['id'],
            'status': status,
            'error': error,
        })

    threads = []
    for group in groups:
        t = threading.Thread(target=_worker, args=(group,), name=f"planner_group_{group['id']}", daemon=True)
        t.start()
        threads.append(t)

    try:
        for t in threads:
            t.join()
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

        # 전체 상태 = 우선순위(error > stopped > finished).
        statuses = [s for s, _ in group_results.values()]
        if 'error' in statuses:
            overall = 'error'
            overall_error = next((e for s, e in group_results.values() if s == 'error' and e), None)
        elif 'stopped' in statuses:
            overall = 'stopped'
            overall_error = None
        else:
            overall = 'finished'
            overall_error = None

        print(f'[NOTICE] Planner run {overall}')
        _emit(socketio_instance, 'planner_run_end', {
            'status': overall,
            'error': overall_error,
        })
