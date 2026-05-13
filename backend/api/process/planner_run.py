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
    # move_homepose 가 켜져있을 때만 checkpoint_test 의 go_home_first / move_homepose 가 True.
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
    sub_control = {'stop': False, 'done': False, 'done_threshold': done_threshold if until_done else None}
    preloaded = (ctx.get('preloaded') or {}).get(block.get('checkpoint_id'))

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
                go_home_first=move_homepose,
            )
        except Exception:
            print(f"[ERROR] checkpoint_test failed: {traceback.format_exc()}")

    thread = threading.Thread(target=_runner, name=f"planner_ckpt_{block.get('id')}", daemon=True)
    thread.start()

    deadline = None if until_done else time.time() + duration
    try:
        while True:
            if task_control['stop']:
                break
            if sub_control.get('done'):
                print(f"[PLANNER] checkpoint block '{block.get('name')}' done signal triggered (threshold={done_threshold})")
                break
            if deadline is not None and time.time() >= deadline:
                break
            time.sleep(0.1)
    finally:
        sub_control['stop'] = True
        thread.join(timeout=15)
        if thread.is_alive():
            print('[WARNING] checkpoint_test thread did not exit within 15s')


def _run_timesleep(block, ctx, task_control):
    duration = float(block.get('duration') or 0)
    if duration <= 0:
        return
    deadline = time.time() + duration
    while time.time() < deadline:
        if task_control['stop']:
            return
        time.sleep(0.1)


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


_HANDLERS = {
    'joint_position': _run_joint_position,
    'checkpoint': _run_checkpoint,
    'timesleep': _run_timesleep,
    # 'sync' is dispatched separately in _run_group because it needs group_id;
    # the lookup just has to not return None so the unknown-type guard passes.
    'sync': _run_sync,
}


def _run_group(group, ctx, task_control, repeat_count, infinite, total_iterations):
    """Execute a single group's plan in this thread. Returns ``(status, error)``."""
    socketio_instance = ctx['socketio']
    group_id = group['id']
    plan = list(group.get('blocks') or [])
    total = len(plan)

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
        _emit(socketio_instance, 'planner_iteration_start', {
            'group_id': group_id,
            'iteration': iteration,
            'total_iterations': total_iterations,
        })

        for index, block in enumerate(plan):
            if task_control['stop']:
                return 'stopped', None

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

    ctx = {
        'app': app,
        'agents': getattr(app, 'agents', {}),
        'socketio': socketio_instance,
        'workspaces_by_id': {ws['id']: ws for ws in workspaces},
        'preloaded': preloaded,
        'sync_barriers': sync_barriers,
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
