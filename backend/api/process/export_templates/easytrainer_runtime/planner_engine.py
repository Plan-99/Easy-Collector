"""Standalone planner execution engine for an exported EasyTrainer planner.

This is the runtime core shared by ``run_planner.py`` (autonomous single-shot
runner) and ``ros_planner_service.py`` (service-triggered node). It mirrors the
in-container ``src/backend/api/process/planner_run.py`` but with no Flask, no
database, no gRPC bridge — robots are driven directly over ROS 2 topics via
``SimpleAgent`` and checkpoints are run through the bundled ``CheckpointInference``.

A planner is a set of *groups*; each group is a sequential list of *blocks* and
groups run in parallel threads (they never share a robot, by construction). Block
types:

    joint_position : interpolate the workspace's robots to a joint config
    checkpoint     : run a bundled inference checkpoint (closed loop)
    timesleep      : pause
    sync           : cross-group barrier
    query_pose     : call a ROS 2 std_srvs/Trigger service, drive robots to the
                     returned pose (joint mode only — EE mode needs an IK solver
                     which is not bundled)

Stop semantics: a shared ``task_control`` dict carries ``{'stop': bool}``; every
group worker polls it and aborts when set.
"""
from __future__ import annotations

import json
import threading
import time
import traceback
from pathlib import Path
from typing import Dict, List, Optional

import numpy as np

from .simple_agent import SimpleAgent
from .simple_env import SimpleEnv
from .image_parser import fetch_image_with_config


ASSEMBLY_SLOTS = ("left_arm", "right_arm", "left_tool", "right_tool", "mobile_base")


# ──────────────────────────────────────────────────────────────────────────────
# Sync barrier (verbatim behaviour of planner_run.SyncBarrier)
# ──────────────────────────────────────────────────────────────────────────────
class SyncBarrier:
    """Generation-counted barrier across groups for the planner ``sync`` block."""

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
            while self.generation == my_gen and not task_control.get("stop"):
                self.cond.wait(timeout=0.2)

    def drop(self, group_id):
        with self.cond:
            if group_id not in self.expected:
                return
            self.expected.discard(group_id)
            self.arrived.discard(group_id)
            if self.expected and self.arrived >= self.expected:
                self.arrived = set()
                self.generation += 1
            self.cond.notify_all()


def _compute_sync_barriers(groups) -> Dict[str, SyncBarrier]:
    sync_groups: Dict[str, set] = {}
    for grp in groups:
        for blk in grp.get("blocks") or []:
            if blk.get("type") != "sync":
                continue
            sid = (blk.get("sync_id") or "").strip()
            if not sid:
                continue
            sync_groups.setdefault(sid, set()).add(grp["id"])
    return {sid: SyncBarrier(gids) for sid, gids in sync_groups.items()}


# ──────────────────────────────────────────────────────────────────────────────
# Context — built once, shared by every group worker thread
# ──────────────────────────────────────────────────────────────────────────────
class PlannerContext:
    """Holds everything a planner run needs: ROS node, agents, envs, checkpoints.

    Built once at startup (``build_context``) so neither group workers nor
    repeated service calls re-create ROS subscriptions or reload models.
    """

    def __init__(self, node, bundle_dir: Path, meta: dict, device: str = "cuda",
                 dry_run: bool = False):
        self.node = node
        self.bundle_dir = Path(bundle_dir)
        self.meta = meta
        self.device = device
        self.dry_run = dry_run

        # workspace_id (int) -> workspace meta dict
        self.workspaces: Dict[int, dict] = {
            int(k): v for k, v in (meta.get("workspaces") or {}).items()
        }
        # checkpoint_id (str) -> per-checkpoint export_meta dict
        self.checkpoints_meta: Dict[str, dict] = {
            str(k): v for k, v in (meta.get("checkpoints") or {}).items()
        }

        self.agents: Dict[int, SimpleAgent] = {}
        self.envs: Dict[int, SimpleEnv] = {}
        self.inferences: Dict[str, object] = {}
        self.sync_barriers: Dict[str, SyncBarrier] = {}
        # Interpolation nodes — one per robot with interpolation=True. The
        # runner attaches each to its executor so the 200Hz smoothing timer
        # fires alongside the planner thread.
        self.interpolation_nodes: List = []

    # ── lookups ──────────────────────────────────────────────────────────
    def workspace(self, ws_id) -> dict:
        ws = self.workspaces.get(int(ws_id))
        if ws is None:
            raise RuntimeError(f"workspace_id={ws_id} not found in planner_meta.json")
        return ws

    def workspace_robots(self, ws_id) -> List[dict]:
        ws = self.workspace(ws_id)
        return (ws.get("assembly") or {}).get("robots") or []

    def workspace_agents(self, ws_id) -> List[SimpleAgent]:
        """Agents for a workspace, sorted by robot id (matches the state-concat
        order the checkpoint was trained with)."""
        robots = sorted(self.workspace_robots(ws_id), key=lambda r: int(r["id"]))
        out = []
        for r in robots:
            agent = self.agents.get(int(r["id"]))
            if agent is None:
                raise RuntimeError(
                    f"robot '{r.get('name')}' (id={r['id']}) has no agent — "
                    f"unsupported write_type or it was skipped at startup"
                )
            out.append(agent)
        return out


def build_context(node, bundle_dir, meta: dict, device: str = "cuda",
                  dry_run: bool = False, preload_checkpoints: bool = True,
                  executor=None) -> PlannerContext:
    """Create the SimpleAgents, SimpleEnvs and CheckpointInference objects the
    planner needs. ROS subscriptions/publishers are created here, so the node's
    executor must be spinning (or about to spin) for them to fire.

    If ``executor`` is provided and any referenced robot has ``interpolation``
    enabled in its meta, a per-robot ``JointInterpolationNode`` is instantiated
    and added to that executor so 10Hz inference goals are smoothed to 200Hz
    before they reach the robot driver."""
    from inference import CheckpointInference  # bundled single-step API
    from .interpolation_node import JointInterpolationNode

    ctx = PlannerContext(node, bundle_dir, meta, device=device, dry_run=dry_run)

    # ── 1. one SimpleAgent per unique robot across every workspace ───────
    seen_robot_ids = set()
    interp_robots: List[dict] = []
    for ws_id, ws in ctx.workspaces.items():
        for robot in (ws.get("assembly") or {}).get("robots") or []:
            rid = int(robot["id"])
            if rid in seen_robot_ids:
                continue
            seen_robot_ids.add(rid)
            try:
                ctx.agents[rid] = SimpleAgent(node, robot)
                if bool(robot.get("interpolation", False)):
                    interp_robots.append(robot)
            except NotImplementedError as e:
                # write_type != topic — keep going so groups that don't touch
                # this robot still run; groups that need it fail loudly later.
                print(f"[planner_engine][WARN] robot '{robot.get('name')}' "
                      f"(id={rid}) skipped: {e}")

    # ── 1b. per-robot interpolation nodes (200Hz smoothing) ──────────────
    if interp_robots:
        if executor is None:
            print(f"[planner_engine][WARN] {len(interp_robots)} robot(s) have "
                  f"interpolation=true but no executor was passed to "
                  f"build_context — smoothing is disabled. The agent will "
                  f"still publish goals to ec_joint_cmd, but no smoothing "
                  f"node is listening. Pass executor=<your executor> to enable.")
        else:
            for robot in interp_robots:
                rid = int(robot["id"])
                hz = robot.get("interpolation_hz") or 200.0
                inode = JointInterpolationNode(
                    rid,
                    write_topic=robot["write_topic"],
                    write_topic_msg=robot["write_topic_msg"],
                    joint_names=robot.get("joint_names") or [],
                    publish_rate=float(hz),
                )
                executor.add_node(inode)
                ctx.interpolation_nodes.append(inode)
                print(f"[planner_engine] interpolation node started for "
                      f"robot id={rid} at {hz}Hz")

    # ── 2. one SimpleEnv per workspace (for checkpoint blocks' images) ───
    for ws_id, ws in ctx.workspaces.items():
        sensors = ws.get("sensors") or []
        robots = sorted(
            (ws.get("assembly") or {}).get("robots") or [],
            key=lambda r: int(r["id"]),
        )
        agents = [ctx.agents[int(r["id"])] for r in robots if int(r["id"]) in ctx.agents]
        ctx.envs[ws_id] = SimpleEnv(node, agents, sensors)

    # ── 3. preload every unique checkpoint referenced by a block ─────────
    if preload_checkpoints:
        for cid, ckpt_meta in ctx.checkpoints_meta.items():
            ckpt_dir = ctx.bundle_dir / "checkpoints" / str(cid)
            meta_path = ckpt_dir / "export_meta.json"
            if not meta_path.exists():
                print(f"[planner_engine][WARN] checkpoint {cid} has no "
                      f"export_meta.json at {meta_path}, skipping preload")
                continue
            print(f"[planner_engine] loading checkpoint {cid} from {ckpt_dir} ...")
            t0 = time.time()
            inf = CheckpointInference(str(ckpt_dir), str(meta_path), device=device)
            inf.reset()
            ctx.inferences[str(cid)] = inf
            print(f"[planner_engine] checkpoint {cid} ready ({time.time() - t0:.1f}s)")

    return ctx


# ──────────────────────────────────────────────────────────────────────────────
# Shared helpers
# ──────────────────────────────────────────────────────────────────────────────
def _wait_for_agents(agents: List[SimpleAgent], task_control: dict,
                     max_wait: float = 35.0) -> None:
    """Block until every agent's move_to finishes, the deadline passes, or stop
    is signalled — cancelling outstanding moves on stop/timeout."""
    deadline = time.time() + max_wait
    while time.time() < deadline:
        if task_control.get("stop"):
            print("[planner_engine] motion interrupted by stop signal")
            for a in agents:
                try:
                    a.cancel_move_to()
                except Exception:
                    pass
            return
        if not any(a.is_moving for a in agents):
            return
        time.sleep(0.1)
    print("[planner_engine][WARN] motion timed out — cancelling and moving on")
    for a in agents:
        try:
            a.cancel_move_to()
        except Exception:
            pass


def _resolve_sensor_config(workspace: dict, sensor_id) -> dict:
    """Per-sensor crop / rotate / resize / sam3 from the workspace meta. The
    planner block references a workspace, so its sensor config (not the
    checkpoint's training-task config) is what we apply."""
    sid = str(sensor_id)
    return {
        "sensor_id": sid,
        "sam3": (workspace.get("sensor_sam3") or {}).get(sid),
        "cropped_area": (workspace.get("sensor_cropped_area") or {}).get(sid),
        "rotate": (workspace.get("sensor_rotate") or {}).get(sid, 0),
        "resize": (workspace.get("sensor_img_size") or {}).get(sid),
    }


def _assembly_slot_map(workspace: dict) -> Dict[str, dict]:
    """Filled assembly slot name → robot dict."""
    assembly = workspace.get("assembly") or {}
    return {slot: assembly[slot] for slot in ASSEMBLY_SLOTS if assembly.get(slot)}


# ──────────────────────────────────────────────────────────────────────────────
# Block handlers
# ──────────────────────────────────────────────────────────────────────────────
def _run_joint_position(block: dict, ctx: PlannerContext, task_control: dict):
    ws_id = block.get("workspace_id")
    ctx.workspace(ws_id)  # existence check
    positions = block.get("positions") or {}
    duration = float(block.get("duration", 5.0))

    targets = []
    for robot in ctx.workspace_robots(ws_id):
        rid = int(robot["id"])
        pos = positions.get(str(rid), positions.get(rid))
        if pos is None:
            continue
        agent = ctx.agents.get(rid)
        if agent is None:
            raise RuntimeError(
                f"robot '{robot.get('name')}' (id={rid}) is not controllable "
                f"(unsupported write_type?)"
            )
        targets.append((agent, [float(p) for p in pos]))

    if not targets:
        print(f"[planner_engine][WARN] joint_position '{block.get('name')}' "
              f"has no joint targets, skipping")
        return

    for agent, pos in targets:
        agent.move_to(pos, duration=duration)
    _wait_for_agents([a for a, _ in targets], task_control,
                     max_wait=max(duration + 5.0, 30.0))


def _run_timesleep(block: dict, ctx: PlannerContext, task_control: dict):
    duration = float(block.get("duration") or 0)
    if duration <= 0:
        return
    deadline = time.time() + duration
    while time.time() < deadline:
        if task_control.get("stop"):
            return
        time.sleep(0.1)


def _run_sync(block: dict, ctx: PlannerContext, task_control: dict, group_id: str):
    sync_id = (block.get("sync_id") or "").strip()
    if not sync_id:
        return
    barrier = ctx.sync_barriers.get(sync_id)
    if barrier is None:
        return
    print(f"[planner_engine] [{group_id}] sync '{sync_id}' waiting "
          f"(expected={sorted(barrier.expected)})")
    barrier.wait(group_id, task_control)
    print(f"[planner_engine] [{group_id}] sync '{sync_id}' passed")


def _run_query_pose(block: dict, ctx: PlannerContext, task_control: dict):
    """Call a std_srvs/Trigger service and drive the workspace's assembly to the
    returned pose. Joint mode is fully supported; EE mode needs an IK solver,
    which is not bundled — raise a clear error so the user runs it in-container."""
    from std_srvs.srv import Trigger

    ws_id = block.get("workspace_id")
    workspace = ctx.workspace(ws_id)
    service_name = (block.get("service_name") or "").strip()
    if not service_name:
        raise RuntimeError("query_pose block has an empty service_name")
    pose_type = block.get("pose_type")
    if pose_type not in ("joint_position", "end_effector_position"):
        raise RuntimeError(f"invalid pose_type: {pose_type}")
    if pose_type == "end_effector_position":
        raise RuntimeError(
            "end_effector_position query_pose is not supported in an exported "
            "planner — it needs the Pinocchio IK solver, which only runs inside "
            "EasyTrainer. Re-author this block in joint_position mode, or run "
            "the planner in-container."
        )

    duration = float(block.get("duration") or 5.0)
    slot_map = _assembly_slot_map(workspace)
    if not slot_map:
        print(f"[planner_engine][WARN] query_pose '{block.get('name')}' has no "
              f"robots in workspace, skipping")
        return

    def _resolve(slot_key):
        robot = slot_map.get(slot_key)
        if robot is None:
            raise RuntimeError(
                f"service '{service_name}' returned slot '{slot_key}', but the "
                f"workspace assembly has no such slot "
                f"(available: {sorted(slot_map)})"
            )
        agent = ctx.agents.get(int(robot["id"]))
        if agent is None:
            raise RuntimeError(
                f"robot '{robot.get('name')}' (slot '{slot_key}') is not "
                f"controllable (unsupported write_type?)"
            )
        return robot, agent

    # ── call the service ────────────────────────────────────────────────
    client = ctx.node.create_client(Trigger, service_name)
    try:
        if not client.wait_for_service(timeout_sec=5.0):
            raise RuntimeError(
                f"service '{service_name}' is not available (waited 5s) — is the "
                f"pose-provider node running?"
            )
        print(f"[planner_engine] calling query_pose service '{service_name}'")
        future = client.call_async(Trigger.Request())
        deadline = time.time() + 30.0
        while not future.done():
            if task_control.get("stop"):
                raise RuntimeError("query_pose interrupted by stop signal")
            if time.time() > deadline:
                raise RuntimeError(f"service '{service_name}' call timed out (30s)")
            time.sleep(0.02)
        resp = future.result()
    finally:
        ctx.node.destroy_client(client)

    if not resp.success:
        raise RuntimeError(
            f"service '{service_name}' reported failure: "
            f"{resp.message or 'no message'}"
        )
    try:
        payload = json.loads(resp.message)
    except (TypeError, ValueError) as e:
        raise RuntimeError(
            f"service '{service_name}' message is not valid JSON: {e} "
            f"(got: {resp.message!r})"
        )
    if not isinstance(payload, dict):
        raise RuntimeError(f"service '{service_name}' payload is not an object: {payload!r}")

    positions = payload.get("positions")
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
        current = agent.get_joint_states()
        if current is not None and len(current) != len(joints):
            raise RuntimeError(
                f"service '{service_name}' slot '{slot_key}' returned "
                f"{len(joints)} joint values, but robot '{robot.get('name')}' has "
                f"{len(current)} joints — include every joint (gripper included)"
            )
        targets.append((agent, [float(j) for j in joints]))

    for agent, pos in targets:
        agent.move_to(pos, duration=duration)
    _wait_for_agents([a for a, _ in targets], task_control,
                     max_wait=max(duration + 5.0, 30.0))
    return payload


def _checkpoint_loop(inf, env: SimpleEnv, agents: List[SimpleAgent], sensors: List[dict],
                     workspace: dict, has_succeed: bool, hz: float,
                     until_done: bool, done_threshold: float,
                     sub_control: dict, dry_run: bool):
    """Closed-loop inference — mirrors ros_inference.py's main loop. Runs in a
    worker thread; the outer handler owns the duration / done bookkeeping."""
    period = 1.0 / max(hz, 0.1)
    step = 0
    while not sub_control.get("stop"):
        loop_start = time.time()

        obs = env.get_observation()
        qpos_concat = np.concatenate(
            [np.asarray(obs["robot_states"][a.id]["qpos"], dtype=np.float32)
             for a in agents]
        )

        images = {}
        for sensor in sensors:
            sid = sensor["id"]
            raw = obs["images"][f"sensor_{sid}"]
            cfg = _resolve_sensor_config(workspace, sid)
            processed = fetch_image_with_config(raw, cfg)
            # CheckpointInference expects RGB; ros_image_to_numpy returns BGR.
            if processed.ndim == 3 and processed.shape[2] == 3:
                processed = processed[:, :, ::-1]
            images[f"sensor_{sid}"] = np.ascontiguousarray(processed)

        action = inf.infer(qpos_concat, images)

        if has_succeed and len(action) > 0:
            succeed_val = float(action[-1])
            action = action[:-1]
            if until_done and succeed_val > done_threshold:
                print(f"[planner_engine] checkpoint done signal "
                      f"(succeed={succeed_val:.3f} > {done_threshold})")
                sub_control["done"] = True
                break

        start = 0
        for a in agents:
            target = action[start: start + a.joint_len]
            start += a.joint_len
            if dry_run:
                print(f"[planner_engine][dry-run] step {step} {a.name}: "
                      f"{np.round(target, 4).tolist()}")
            else:
                a.move_joint_step(target)

        step += 1
        elapsed = time.time() - loop_start
        if elapsed < period:
            time.sleep(period - elapsed)
    print(f"[planner_engine] checkpoint loop exited after {step} steps")


def _run_checkpoint(block: dict, ctx: PlannerContext, task_control: dict):
    ws_id = block.get("workspace_id")
    workspace = ctx.workspace(ws_id)
    cid = block.get("checkpoint_id")
    inf = ctx.inferences.get(str(cid))
    if inf is None:
        raise RuntimeError(
            f"checkpoint_id={cid} was not bundled / failed to load — cannot run "
            f"this block"
        )
    ckpt_meta = ctx.checkpoints_meta.get(str(cid)) or {}

    agents = ctx.workspace_agents(ws_id)
    sensors = workspace.get("sensors") or []
    if not sensors:
        raise RuntimeError(
            f"workspace '{workspace.get('name')}' has no sensors — a checkpoint "
            f"block needs camera input"
        )
    env = ctx.envs.get(int(ws_id))
    if env is None:
        raise RuntimeError(f"no SimpleEnv built for workspace {ws_id}")

    duration = float(block.get("duration") or 30)
    until_done = bool(block.get("until_done"))
    has_succeed = bool(ckpt_meta.get("has_succeed"))
    if until_done and not has_succeed:
        raise RuntimeError(
            f"checkpoint block '{block.get('name')}' has until_done=True but the "
            f"checkpoint was not trained with has_succeed — cannot detect 'done'"
        )
    try:
        done_threshold = float(
            block.get("done_threshold") if block.get("done_threshold") is not None else 0.5
        )
    except (TypeError, ValueError):
        done_threshold = 0.5
    hz = float(block.get("hz", 10) or 10)
    move_homepose = bool(block.get("move_homepose", True))
    try:
        move_homepose_duration = float(block.get("move_homepose_duration") or 5.0)
    except (TypeError, ValueError):
        move_homepose_duration = 5.0

    # ── optional: go to the workspace home pose before inference ─────────
    if move_homepose:
        home_pose = workspace.get("home_pose")
        if home_pose:
            moved = []
            for agent in agents:
                target = home_pose.get(str(agent.id), home_pose.get(agent.id))
                if target is not None:
                    agent.move_to([float(j) for j in target],
                                  duration=move_homepose_duration)
                    moved.append(agent)
            if moved:
                print(f"[planner_engine] moving {len(moved)} robot(s) to home pose")
                _wait_for_agents(moved, task_control,
                                 max_wait=max(move_homepose_duration + 5.0, 30.0))
            if task_control.get("stop"):
                return
        else:
            print("[planner_engine][WARN] move_homepose set but workspace has no "
                  "home_pose — skipping")

    inf.reset()
    sub_control = {"stop": False, "done": False}

    def _runner():
        try:
            _checkpoint_loop(
                inf, env, agents, sensors, workspace, has_succeed, hz,
                until_done, done_threshold, sub_control, ctx.dry_run,
            )
        except Exception:
            print(f"[planner_engine][ERROR] checkpoint loop failed:\n"
                  f"{traceback.format_exc()}")
            sub_control["error"] = True

    thread = threading.Thread(target=_runner, name=f"ckpt_{block.get('id')}", daemon=True)
    thread.start()

    deadline = None if until_done else time.time() + duration
    try:
        while True:
            if task_control.get("stop"):
                break
            if sub_control.get("done") or sub_control.get("error"):
                break
            if not thread.is_alive():
                break
            if deadline is not None and time.time() >= deadline:
                break
            time.sleep(0.1)
    finally:
        sub_control["stop"] = True
        thread.join(timeout=15)
        if thread.is_alive():
            print("[planner_engine][WARN] checkpoint loop did not exit within 15s")
    if sub_control.get("error"):
        raise RuntimeError("checkpoint inference loop raised — see traceback above")


_HANDLERS = {
    "joint_position": _run_joint_position,
    "checkpoint": _run_checkpoint,
    "timesleep": _run_timesleep,
    "query_pose": _run_query_pose,
    "sync": _run_sync,  # dispatched specially (needs group_id)
}


# ──────────────────────────────────────────────────────────────────────────────
# Group / planner orchestration
# ──────────────────────────────────────────────────────────────────────────────
def _run_group(group: dict, ctx: PlannerContext, task_control: dict,
               repeat_count: int, infinite: bool):
    group_id = group["id"]
    plan = list(group.get("blocks") or [])
    total = len(plan)

    iteration = 0
    while infinite or iteration < repeat_count:
        if task_control.get("stop"):
            return "stopped", None
        iteration += 1
        print(f"[planner_engine] [{group_id}] iteration {iteration}"
              f"{'' if infinite else f'/{repeat_count}'}")

        for index, block in enumerate(plan):
            if task_control.get("stop"):
                return "stopped", None

            btype = block.get("type")
            handler = _HANDLERS.get(btype)
            print(f"[planner_engine] [{group_id}] [{index + 1}/{total}] "
                  f"{btype} — {block.get('name')}")

            try:
                if handler is None:
                    raise RuntimeError(f"unknown block type '{btype}'")
                if btype == "sync":
                    _run_sync(block, ctx, task_control, group_id)
                else:
                    handler(block, ctx, task_control)
            except Exception as e:
                print(f"[planner_engine][ERROR] [{group_id}] block "
                      f"'{block.get('name')}' failed:\n{traceback.format_exc()}")
                return "error", str(e)

            if task_control.get("stop"):
                return "stopped", None

    return "finished", None


def run_planner(ctx: PlannerContext, task_control: dict, repeat_count: int = 1):
    """Execute every group in ``ctx.meta['groups']`` in parallel threads.

    repeat_count >= 1 repeats each group's plan N times; <= 0 loops forever
    until ``task_control['stop']`` is set. Returns ``(overall_status, error)``.
    """
    groups = [g for g in (ctx.meta.get("groups") or []) if g.get("blocks")]
    if not groups:
        print("[planner_engine] planner has no groups with blocks — nothing to do")
        return "finished", None

    try:
        repeat_count = int(repeat_count)
    except (TypeError, ValueError):
        repeat_count = 1
    infinite = repeat_count <= 0

    ctx.sync_barriers = _compute_sync_barriers(groups)
    if ctx.sync_barriers:
        print(f"[planner_engine] sync barriers: "
              f"{ {sid: sorted(b.expected) for sid, b in ctx.sync_barriers.items()} }")

    print(f"[planner_engine] starting planner — {len(groups)} group(s), "
          f"{'infinite loop' if infinite else f'{repeat_count} iteration(s)'}")

    group_results: Dict[str, tuple] = {}

    def _worker(group):
        try:
            status, error = _run_group(group, ctx, task_control, repeat_count, infinite)
        except Exception as e:
            status, error = "error", str(e)
            print(f"[planner_engine][ERROR] group {group.get('id')} crashed:\n"
                  f"{traceback.format_exc()}")
        finally:
            for barrier in ctx.sync_barriers.values():
                barrier.drop(group["id"])
        group_results[group["id"]] = (status, error)
        print(f"[planner_engine] group {group['id']} → {status}"
              f"{f' ({error})' if error else ''}")

    threads = []
    for group in groups:
        t = threading.Thread(target=_worker, args=(group,),
                             name=f"planner_group_{group['id']}", daemon=True)
        t.start()
        threads.append(t)
    for t in threads:
        t.join()

    statuses = [s for s, _ in group_results.values()]
    if "error" in statuses:
        overall = "error"
        overall_error = next((e for s, e in group_results.values()
                              if s == "error" and e), None)
    elif "stopped" in statuses:
        overall, overall_error = "stopped", None
    else:
        overall, overall_error = "finished", None

    print(f"[planner_engine] planner run {overall}")
    return overall, overall_error
