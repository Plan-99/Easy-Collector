"""Planner runner: executes a plan (list of blocks) sequentially.

Block types:
    - joint_position: move each robot in a workspace to a given joint configuration
    - checkpoint:     run an inference checkpoint for `duration` seconds
    - timesleep:      pause for `duration` seconds

Stop semantics:
    The runner receives a shared `task_control` dict from ProcessManager. When the user clicks
    Stop, ProcessManager sets task_control['stop'] = True. Each block handler polls this flag.
"""

import os
import time
import threading
import traceback
from concurrent.futures import ThreadPoolExecutor

from .checkpoint_test import checkpoint_test


def _preload_checkpoints(plan, socketio_instance):
    """Load every unique checkpoint referenced by the plan into CPU memory.

    The actual GPU transfer (.cuda()) happens inside ``checkpoint_test`` when each
    block runs; this just front-loads the slow part — disk read + model construction
    + processor/OOD load — so subsequent blocks don't stall the run mid-plan.

    Returns a dict ``{checkpoint_id: {policy, preprocessor, postprocessor, ood}}``.
    Failures are logged and skipped so the block falls back to the original
    in-block load path.
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
    ckpt_blocks = [b for b in (plan or []) if b.get('type') == 'checkpoint']
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
            # LeRobot's from_pretrained reads ``device`` from config and defaults
            # to cuda when available. Force CPU here so the prefetch cache doesn't
            # pin VRAM for every preloaded model — checkpoint_test will move it
            # back to cuda when its block actually runs.
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


def _emit(socketio_instance, event, payload):
    if socketio_instance is None:
        return
    try:
        socketio_instance.emit(event, payload)
    except Exception as e:
        print(f"[ERROR] socketio emit failed ({event}): {e}")


def _resolve_position(positions, robot_id):
    """Block stores positions keyed by robot id (the JSON layer makes int keys into strings).
    Look up under both forms so we don't fail on type mismatch."""
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

    pool = ThreadPoolExecutor(max_workers=len(targets))
    try:
        futures = [pool.submit(agent.move_to, pos) for agent, pos in targets]
        timeout_at = time.time() + max(30.0, float(block.get('timeout', 30)))
        while not all(f.done() for f in futures):
            if task_control['stop']:
                print('[NOTICE] joint_position interrupted by stop signal')
                break
            if time.time() > timeout_at:
                print('[WARNING] joint_position timed out — moving on')
                break
            time.sleep(0.1)
        for f in futures:
            if f.done() and f.exception() is not None:
                print(f"[ERROR] move_to failed: {f.exception()}")
    finally:
        pool.shutdown(wait=False)


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
    sub_control = {'stop': False}
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
                move_homepose=False,
                hz=block.get('hz', 10),
                re_inference_steps=block.get('re_inference_steps', 1),
                temporal_ensemble_coeff=block.get('temporal_ensemble_coeff', 0.01),
                action_type=block.get('action_type'),
                preloaded=preloaded,
                go_home_first=False,
            )
        except Exception:
            print(f"[ERROR] checkpoint_test failed: {traceback.format_exc()}")

    thread = threading.Thread(target=_runner, name=f"planner_ckpt_{block.get('id')}", daemon=True)
    thread.start()

    deadline = time.time() + duration
    try:
        while time.time() < deadline:
            if task_control['stop']:
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


_HANDLERS = {
    'joint_position': _run_joint_position,
    'checkpoint': _run_checkpoint,
    'timesleep': _run_timesleep,
}


def planner_run(plan, workspaces, app, socketio_instance, task_control,
                repeat_count=1):
    """Execute a plan as a sequence of blocks, optionally repeating the whole plan.

    plan: list[dict] — block dicts with at least 'type', 'id', plus type-specific keys.
    workspaces: list[dict] — task dicts (with assembly+sensors), used to resolve workspace_id.
    repeat_count:
      - >= 1: run the plan that many times in a row
      - <= 0: run infinitely until stopped by user (treated as "loop forever")
    """
    plan = list(plan or [])
    total = len(plan)
    # Normalize: <=0 means infinite. We still emit a stable 'total_iterations'
    # for the UI — None for infinite, the integer otherwise.
    try:
        repeat_count = int(repeat_count)
    except (TypeError, ValueError):
        repeat_count = 1
    infinite = repeat_count <= 0
    total_iterations = None if infinite else repeat_count

    print(f"[NOTICE] Planner run started ({total} blocks, "
          f"{'infinite loop' if infinite else f'{repeat_count} iteration(s)'})")
    _emit(socketio_instance, 'planner_run_start', {
        'total': total,
        'total_iterations': total_iterations,
    })

    # Preload all checkpoint models to CPU before iterating blocks. This trades
    # an upfront stall (only at plan start) for zero mid-plan ckpt-load latency.
    preloaded = _preload_checkpoints(plan, socketio_instance)

    ctx = {
        'app': app,
        'agents': getattr(app, 'agents', {}),
        'socketio': socketio_instance,
        'workspaces_by_id': {ws['id']: ws for ws in workspaces},
        'preloaded': preloaded,
    }

    status = 'finished'
    error_message = None
    try:
        iteration = 0
        while infinite or iteration < repeat_count:
            if task_control['stop']:
                status = 'stopped'
                break

            iteration += 1
            _emit(socketio_instance, 'planner_iteration_start', {
                'iteration': iteration,
                'total_iterations': total_iterations,
            })
            if infinite:
                print(f"[NOTICE] === Iteration {iteration} (infinite loop) ===")
            elif repeat_count > 1:
                print(f"[NOTICE] === Iteration {iteration}/{repeat_count} ===")

            iter_status = 'finished'
            iter_error = None
            for index, block in enumerate(plan):
                if task_control['stop']:
                    iter_status = 'stopped'
                    break

                handler = _HANDLERS.get(block.get('type'))
                block_summary = {
                    'index': index,
                    'total': total,
                    'iteration': iteration,
                    'total_iterations': total_iterations,
                    'block_id': block.get('id'),
                    'type': block.get('type'),
                    'name': block.get('name'),
                }
                _emit(socketio_instance, 'planner_block_start', block_summary)
                print(f"[NOTICE] [{index + 1}/{total}] {block.get('type')} — {block.get('name')}")

                block_status = 'finished'
                block_error = None
                try:
                    if handler is None:
                        raise RuntimeError(f"unknown block type '{block.get('type')}'")
                    handler(block, ctx, task_control)
                    if task_control['stop']:
                        block_status = 'stopped'
                except Exception as e:
                    block_status = 'error'
                    block_error = str(e)
                    print(f"[ERROR] block '{block.get('name')}' failed: {traceback.format_exc()}")

                _emit(socketio_instance, 'planner_block_end', {
                    **block_summary,
                    'status': block_status,
                    'error': block_error,
                })

                if block_status == 'error':
                    iter_status = 'error'
                    iter_error = block_error
                    break
                if block_status == 'stopped':
                    iter_status = 'stopped'
                    break

            if iter_status == 'error':
                status = 'error'
                error_message = iter_error
                break
            if iter_status == 'stopped':
                status = 'stopped'
                break
            # 정상 종료된 iteration은 계속 진행 (또는 반복 횟수 끝)
        else:
            # while loop가 정상 종료(=infinite=False고 모두 마침)된 경우.
            # while-else는 break가 없을 때만 실행됨.
            status = 'finished'
    finally:
        # Drop the prefetch cache so model parameters can be GC'd promptly.
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

        print(f'[NOTICE] Planner run {status}')
        _emit(socketio_instance, 'planner_run_end', {
            'status': status,
            'error': error_message,
        })
