"""
MuJoCo simulation runner.

Wraps a MuJoCo model so the ROS2 node only deals with high-level semantics:
- read joint states (positions + velocities)
- write joint position targets
- render the scene through named cameras (returns numpy frames from buffer)

GL/스레드 전략 (중요):
  같은 프로세스에서 viewer(GLFW/X11)와 offscreen Renderer(EGL)가 서로 다른
  스레드에서 동시에 사용되면 NVIDIA 드라이버에서 EGL 컨텍스트 생성이 실패한다
  (`Renderer.__init__`이 `_mjr_context` 할당 전에 죽음 → `__del__` 트레이스 spam).
  그래서 모든 GL 작업(viewer 초기화/sync + offscreen render) 은 SimRunner 스레드
  단독으로 수행하고, 외부(ROS timer) 는 `render_rgb()` 호출로 캐시된 numpy
  프레임만 가져간다. 캐시 갱신 주기는 image_publish_hz 인자로 제어.

Kept independent of rclpy so it can be unit-tested or reused from a different
transport (e.g. a future gRPC/shm streaming path) without changes.
"""
from __future__ import annotations

import math
import os
import threading
import time
from dataclasses import dataclass
from typing import Optional, Sequence

# MuJoCo의 GL 백엔드를 mujoco import 전에 결정해야 한다. 컨테이너에는
# X11 디스플레이가 없을 수도 있으므로 기본을 egl(NVIDIA 헤드리스)로 강제 —
# 환경변수가 이미 설정돼 있으면(예: 컨테이너 entrypoint에서 export) 그 값을 존중.
os.environ.setdefault("MUJOCO_GL", "egl")
os.environ.setdefault("PYOPENGL_PLATFORM", os.environ["MUJOCO_GL"])

import mujoco
import numpy as np


@dataclass(frozen=True)
class JointSpec:
    """A controllable joint exposed to the outside world."""
    name: str           # public name (matches ROS topic / EasyTrainer joint_names)
    qpos_addr: int      # index into mjData.qpos
    qvel_addr: int      # index into mjData.qvel
    actuator_id: int    # index into mjData.ctrl
    lower: float
    upper: float


class SimRunner:
    """Threaded MuJoCo simulator.

    Physics loop, viewer, offscreen rendering 모두 한 스레드에서 수행된다.
    호출자(ROS timer)는 `set_targets()`/`get_joint_state()`/`render_rgb()` 만 사용.
    """

    def __init__(self, model_path: str, joint_names: Sequence[str],
                 camera_names: Sequence[str],
                 image_width: int = 640, image_height: int = 480,
                 realtime_factor: float = 1.0, show_viewer: bool = False,
                 image_publish_hz: float = 20.0,
                 wrist_specs: Optional[Sequence] = None):
        self.model = mujoco.MjModel.from_xml_path(model_path)
        self.data = mujoco.MjData(self.model)
        self._show_viewer = bool(show_viewer)
        self._viewer = None

        self._home_keyframe_id = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_KEY, "home")
        self._reset_to_home()

        self._joints = self._build_joint_specs(joint_names)
        self._target = np.array([self.data.ctrl[j.actuator_id] for j in self._joints], dtype=np.float64)

        self._camera_names: list[str] = list(camera_names)
        self._camera_ids: list[int] = []
        for cam in self._camera_names:
            cid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_CAMERA, cam)
            if cid < 0:
                raise ValueError(f"Camera '{cam}' not found in MJCF.")
            self._camera_ids.append(cid)

        self._image_width = int(image_width)
        self._image_height = int(image_height)
        # Renderer는 SimRunner 스레드 안에서만 만들고 사용한다 — 다른 스레드 접근 금지.
        self._renderer: Optional[mujoco.Renderer] = None
        # 프레임 캐시(cam_idx → ndarray). 외부 reader는 lock 잡고 복사본 가져감.
        self._frame_buffers: dict[int, np.ndarray] = {}
        self._buffer_lock = threading.Lock()

        # Wrist depth: the `visual_reach` planner block needs RGB-D from a wrist
        # camera. We render a depth map for each wrist camera every tick (SimRunner
        # thread only) and snapshot the camera pose/intrinsics + its arm's ee_site
        # pose alongside it, so the backend can back-project a pixel to world XYZ.
        #
        # `wrist_specs`: list of (camera_name, ee_site_name) — one entry per arm so
        # a dual-arm world can serve `/<prefix>/wrist_rgbd` per side. When None we
        # fall back to the single-arm tutorial convention: ("wrist_cam", "ee_site").
        self._depth_renderer: Optional[mujoco.Renderer] = None
        if wrist_specs is None:
            wrist_specs = ([("wrist_cam", "ee_site")]
                           if "wrist_cam" in self._camera_names else [])
        self._wrist_specs: list[dict] = []
        for cam_name, site_name in wrist_specs:
            if cam_name not in self._camera_names:
                continue
            site_id = (mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SITE, site_name)
                       if site_name else -1)
            self._wrist_specs.append({
                "name": cam_name,
                "cam_idx": self._camera_names.index(cam_name),
                "site_id": int(site_id),
            })
        # Latest {cam_name: {"depth": ndarray, "meta": dict}} under _buffer_lock.
        self._wrist_data: dict[str, dict] = {}
        # Kept for the single-arm peg auto-grasp path (find by name; -1 if absent).
        self._ee_site_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SITE, "ee_site")

        self._target_lock = threading.Lock()
        self._stop_event = threading.Event()
        self._reset_objects_event = threading.Event()
        self._reset_request = threading.Event()
        self._reset_done = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self._realtime_factor = float(realtime_factor)
        self._render_period = 1.0 / max(1.0, float(image_publish_hz))

        # Free joints (movable objects)와 그 초기 qpos를 미리 캐시 — reset 시 robot
        # 관절은 건드리지 않고 객체만 keyframe 초기 상태로 되돌리기 위함.
        # (keyframe "home"이 정의돼 있다는 가정. 없으면 빈 list.)
        self._object_reset_specs: list[tuple[int, int, np.ndarray, np.ndarray]] = []
        try:
            home_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_KEY, "home")
        except Exception:
            home_id = -1
        if home_id >= 0:
            key_qpos = np.asarray(self.model.key_qpos[home_id])
            for jid in range(self.model.njnt):
                if int(self.model.jnt_type[jid]) == int(mujoco.mjtJoint.mjJNT_FREE):
                    qpos_addr = int(self.model.jnt_qposadr[jid])
                    dof_addr = int(self.model.jnt_dofadr[jid])
                    self._object_reset_specs.append((
                        qpos_addr,
                        dof_addr,
                        key_qpos[qpos_addr:qpos_addr + 7].copy(),
                        np.zeros(6, dtype=np.float64),
                    ))

        # Name-keyed view of the same free joints (peg / hole_base) so the
        # /tutorial/randomize service can move a *specific* object. Maps body
        # name → (qpos_addr, dof_addr, home_qpos7). Built from the home keyframe.
        self._object_specs_by_name: dict[str, tuple[int, int, np.ndarray]] = {}
        if home_id >= 0:
            key_qpos = np.asarray(self.model.key_qpos[home_id])
            for jid in range(self.model.njnt):
                if int(self.model.jnt_type[jid]) != int(mujoco.mjtJoint.mjJNT_FREE):
                    continue
                bid = int(self.model.jnt_bodyid[jid])
                bname = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_BODY, bid)
                if not bname:
                    continue
                qpos_addr = int(self.model.jnt_qposadr[jid])
                dof_addr = int(self.model.jnt_dofadr[jid])
                self._object_specs_by_name[bname] = (
                    qpos_addr, dof_addr, key_qpos[qpos_addr:qpos_addr + 7].copy())

        # Randomize request plumbing — mirrors the reset mechanism so all mjData
        # mutation happens on the physics thread (no race with mj_step).
        self._randomize_request = threading.Event()
        self._randomize_done = threading.Event()
        self._randomize_ranges: dict = {}
        self._randomize_sampled: dict = {}
        # Physics steps to let randomized objects come to rest before the sampled
        # poses are reported (so the planner grasps where they actually settle).
        self._randomize_settle_steps = 300
        self._rng = np.random.default_rng()

    def _build_joint_specs(self, joint_names: Sequence[str]) -> list[JointSpec]:
        specs: list[JointSpec] = []
        for name in joint_names:
            jid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, name)
            if jid < 0:
                raise ValueError(f"Joint '{name}' not found in MJCF.")
            qpos_addr = int(self.model.jnt_qposadr[jid])
            qvel_addr = int(self.model.jnt_dofadr[jid])

            actuator_id = -1
            for a in range(self.model.nu):
                if self.model.actuator_trnid[a, 0] == jid:
                    actuator_id = a
                    break
            if actuator_id < 0:
                raise ValueError(f"No actuator drives joint '{name}'.")

            lower, upper = float(self.model.jnt_range[jid, 0]), float(self.model.jnt_range[jid, 1])
            specs.append(JointSpec(name, qpos_addr, qvel_addr, actuator_id, lower, upper))
        return specs

    @property
    def joint_specs(self) -> list[JointSpec]:
        return self._joints

    @property
    def joint_names(self) -> list[str]:
        return [j.name for j in self._joints]

    @property
    def camera_names(self) -> list[str]:
        return list(self._camera_names)

    def _reset_to_home(self) -> None:
        """Re-load the 'home' keyframe (or zero-init if none defined)."""
        try:
            if self._home_keyframe_id >= 0:
                mujoco.mj_resetDataKeyframe(self.model, self.data, self._home_keyframe_id)
            else:
                mujoco.mj_resetData(self.model, self.data)
        except Exception:
            mujoco.mj_resetData(self.model, self.data)

    def reset(self, timeout: float = 2.0) -> bool:
        """Snap world back to the home keyframe (arm + gripper + cube + velocities).

        Performed on the physics thread to avoid racing with mj_step on
        mjData. We just flip a request flag and wait for the loop to ack;
        if the loop isn't running we fall back to an in-place reset.
        """
        if self._thread is None or not self._thread.is_alive():
            with self._target_lock:
                self._reset_to_home()
                for i, j in enumerate(self._joints):
                    self._target[i] = float(self.data.ctrl[j.actuator_id])
            return True

        self._reset_done.clear()
        self._reset_request.set()
        return self._reset_done.wait(timeout=timeout)

    def get_joint_state(self) -> tuple[np.ndarray, np.ndarray]:
        """Return (positions, velocities) for the configured joints, in order."""
        positions = np.array([self.data.qpos[j.qpos_addr] for j in self._joints], dtype=np.float64)
        velocities = np.array([self.data.qvel[j.qvel_addr] for j in self._joints], dtype=np.float64)
        return positions, velocities

    def set_targets(self, positions: dict[str, float]) -> None:
        """Update target positions for a subset of joints, clipped to range."""
        with self._target_lock:
            for i, j in enumerate(self._joints):
                if j.name in positions:
                    self._target[i] = float(np.clip(positions[j.name], j.lower, j.upper))

    def _apply_targets(self) -> None:
        with self._target_lock:
            target = self._target.copy()
        for i, j in enumerate(self._joints):
            self.data.ctrl[j.actuator_id] = target[i]

    def _update_grasp(self) -> None:
        """Kinematic auto-grasp. While the gripper is closed *around* the peg,
        force the peg to follow the gripper rigidly — a closed gripper reliably
        holds the peg (real behaviour) instead of the contact solver squeezing
        the rigid peg out (watermelon-seed ejection) the moment motion stops.
        Captured the instant the fingers close past a threshold (before contact,
        so the capture pose is clean); released when the gripper opens. Called
        right after mj_step so it overrides the physics result for the peg only."""
        m, d = self.model, self.data
        if not hasattr(self, "_grasp_ids"):
            self._grasping = False
            try:
                spec = self._object_specs_by_name.get("peg")
                grip_jid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, "gripper")
                self._grasp_ids = {
                    "peg_bid": mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "peg"),
                    "peg_qadr": int(spec[0]) if spec else -1,
                    "peg_vadr": int(spec[1]) if spec else -1,
                    "grip_qadr": int(m.jnt_qposadr[grip_jid]) if grip_jid >= 0 else -1,
                    "site": int(self._ee_site_id) if self._ee_site_id is not None else -1,
                }
            except Exception:
                self._grasp_ids = None
        ids = self._grasp_ids
        if not ids or ids["peg_qadr"] < 0 or ids["grip_qadr"] < 0 or ids["site"] < 0 \
                or ids["peg_bid"] < 0:
            return
        grip = float(d.qpos[ids["grip_qadr"]])
        closed = grip < 0.012                  # fingers closed (gripping)
        p_site = np.array(d.site_xpos[ids["site"]], dtype=float)
        R_site = np.array(d.site_xmat[ids["site"]], dtype=float).reshape(3, 3)
        peg_pos = np.array(d.xpos[ids["peg_bid"]], dtype=float)
        qa, va = ids["peg_qadr"], ids["peg_vadr"]

        if not self._grasping:
            # Capture only while the fingers close *near* the peg (threshold 0.012
            # fires ~5 mm before contact → clean capture pose). Once captured the
            # grasp LATCHES — see below.
            if closed and float(np.linalg.norm(peg_pos - p_site)) < 0.06:
                peg_R = np.array(d.xmat[ids["peg_bid"]], dtype=float).reshape(3, 3)
                self._grasp_rel_pos = R_site.T @ (peg_pos - p_site)
                self._grasp_rel_R = R_site.T @ peg_R
                self._grasping = True
            return

        # LATCHED: hold the peg as long as the gripper stays closed, regardless of
        # where the peg currently is — this overrides an external /tutorial/reset_
        # world (env.reset() at the start of an insert correction) that teleports
        # the peg back to its home keyframe. Released only when the gripper opens.
        if not closed:
            self._grasping = False
            return
        new_pos = p_site + R_site @ self._grasp_rel_pos
        quat = np.zeros(4)
        mujoco.mju_mat2Quat(quat, (R_site @ self._grasp_rel_R).flatten())
        d.qpos[qa:qa + 3] = new_pos
        d.qpos[qa + 3:qa + 7] = quat
        d.qvel[va:va + 6] = 0.0

    def render_rgb(self, cam_idx: int = 0) -> np.ndarray:
        """Return the most recent cached frame for `cam_idx` (HxWx3, uint8).

        SimRunner 스레드가 비동기로 갱신한 캐시를 그대로 반환한다 — 호출 스레드는
        GL을 건드리지 않는다. 아직 한 프레임도 만들어지지 않았으면 ValueError.
        """
        if not 0 <= cam_idx < len(self._camera_ids):
            raise IndexError(
                f"cam_idx={cam_idx} out of range; cameras={self._camera_names}")
        with self._buffer_lock:
            buf = self._frame_buffers.get(cam_idx)
            if buf is None:
                raise ValueError(f"frame buffer for cam_idx={cam_idx} not yet ready")
            return buf  # ndarray는 SimRunner가 매번 새로 할당해 넣기 때문에 공유 안전.

    def reset_objects(self) -> None:
        """Trigger an environment reset of movable objects (cube, etc.) on the
        next sim tick. Robot joints are NOT touched. Safe to call from any
        thread — actual mutation happens on the SimRunner thread.
        """
        self._reset_objects_event.set()

    def _do_reset_objects(self) -> None:
        # SimRunner 스레드에서만 호출. 자유 관절(freejoint)들의 qpos/qvel을
        # keyframe "home"의 초기값으로 되돌린다.
        for qpos_addr, dof_addr, init_qpos, init_qvel in self._object_reset_specs:
            self.data.qpos[qpos_addr:qpos_addr + 7] = init_qpos
            self.data.qvel[dof_addr:dof_addr + 6] = init_qvel
        # 누적된 가속·접촉 상태 정리를 위해 한 번 forward 계산
        try:
            mujoco.mj_forward(self.model, self.data)
        except Exception:
            pass

    # ------------------------------------------------------------------
    # Object pose readout + scene randomization (model-tester data pipeline)
    # ------------------------------------------------------------------
    def get_object_poses(self, names: Optional[Sequence[str]] = None) -> dict:
        """World poses ``{body: [x,y,z, qw,qx,qy,qz]}`` for movable objects.

        Read-only snapshot of the freejoint bodies (peg / hole_base). Quaternion
        is MuJoCo convention [w,x,y,z], matching ``task_success.check_success``.
        Safe to call from any thread (plain reads of mjData body pose arrays).
        """
        if names is None:
            names = list(self._object_specs_by_name.keys())
        out: dict[str, list] = {}
        for name in names:
            bid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, name)
            if bid < 0:
                continue
            pos = self.data.xpos[bid]
            quat = self.data.xquat[bid]
            out[name] = [float(pos[0]), float(pos[1]), float(pos[2]),
                         float(quat[0]), float(quat[1]), float(quat[2]), float(quat[3])]
        return out

    def randomize_objects(self, ranges: dict, timeout: float = 6.0) -> dict:
        """Re-sample movable-object freejoint poses within ``ranges`` and snap the
        arm back to the home keyframe. Sampling runs on the physics thread to
        avoid racing mj_step; falls back to in-place sampling if the loop is down.

        ``ranges``: ``{body_name: {"x":[lo,hi], "y":[lo,hi], "yaw":[lo,hi]?}}``.
        Returns the sampled world poses ``{body_name: [x,y,z, qw,qx,qy,qz]}``.
        """
        self._randomize_ranges = ranges or {}
        if self._thread is None or not self._thread.is_alive():
            with self._target_lock:
                self._randomize_apply(self._randomize_ranges)
            return dict(self._randomize_sampled)
        self._randomize_done.clear()
        self._randomize_request.set()
        self._randomize_done.wait(timeout=timeout)
        return dict(self._randomize_sampled)

    def randomize_peg(self, delta: float = 0.05) -> dict:
        """Re-place ONLY the peg: x,y uniformly within ±``delta`` of its home
        position, upright; the hole (and every other movable) snaps back to its
        fixed home pose. Used by the standalone peg-watchdog to recover after the
        peg gets knocked over. Restores ``_randomize_ranges`` so it doesn't change
        the default /tutorial/randomize behavior."""
        spec = self._object_specs_by_name.get("peg")
        if spec is None:
            return {}
        home_qpos = spec[2]
        hx, hy = float(home_qpos[0]), float(home_qpos[1])
        ranges = {"peg": {"x": [hx - delta, hx + delta], "y": [hy - delta, hy + delta]}}
        saved = self._randomize_ranges
        try:
            return self.randomize_objects(ranges)
        finally:
            self._randomize_ranges = saved

    def _randomize_apply(self, ranges: dict) -> None:
        """Physics-thread only (caller holds _target_lock). Snap arm home, then
        sample peg/hole_base freejoint qpos in ``ranges`` (home z + upright
        orientation preserved; yaw optional)."""
        self._reset_to_home()
        for i, j in enumerate(self._joints):
            self._target[i] = float(self.data.ctrl[j.actuator_id])
        for name, (qpos_addr, dof_addr, home_qpos) in self._object_specs_by_name.items():
            qpos = home_qpos.copy()
            r = ranges.get(name) if ranges else None
            if r:
                if "x" in r:
                    qpos[0] = float(self._rng.uniform(r["x"][0], r["x"][1]))
                if "y" in r:
                    qpos[1] = float(self._rng.uniform(r["y"][0], r["y"][1]))
                if "yaw" in r:
                    yaw = float(self._rng.uniform(r["yaw"][0], r["yaw"][1]))
                    qpos[3] = math.cos(yaw / 2.0)
                    qpos[4] = 0.0
                    qpos[5] = 0.0
                    qpos[6] = math.sin(yaw / 2.0)
            self.data.qpos[qpos_addr:qpos_addr + 7] = qpos
            self.data.qvel[dof_addr:dof_addr + 6] = 0.0
        try:
            mujoco.mj_forward(self.model, self.data)
        except Exception:
            pass
        # Settle so freejoint dynamics (drop onto the table, contact with walls)
        # fully resolve, then report the *rested* poses. The planner grasps where
        # the object actually ends up — reporting the pre-settle command pose made
        # it aim tens of mm off and miss the grasp. Arm holds home (ctrl persists).
        for _ in range(self._randomize_settle_steps):
            try:
                mujoco.mj_step(self.model, self.data)
            except Exception:
                break
        sampled: dict[str, list] = {}
        for name, (qpos_addr, dof_addr, _home) in self._object_specs_by_name.items():
            bid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, name)
            if bid < 0:
                continue
            pos = self.data.xpos[bid]
            quat = self.data.xquat[bid]
            sampled[name] = [float(pos[0]), float(pos[1]), float(pos[2]),
                             float(quat[0]), float(quat[1]), float(quat[2]), float(quat[3])]
        self._randomize_sampled = sampled

    def start(self) -> None:
        if self._thread is not None and self._thread.is_alive():
            return
        self._stop_event.clear()
        self._thread = threading.Thread(target=self._run, name="MuJoCoSimLoop", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop_event.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
            self._thread = None
        # Renderer 정리는 SimRunner 스레드의 finally에서 이미 처리됨 —
        # 외부 스레드에서 close() 부르면 같은 GL conflict로 죽을 수 있어 호출 안 함.

    # ------------------------------------------------------------------
    # SimRunner 스레드 — 모든 GL 작업은 여기에서.
    # ------------------------------------------------------------------
    def _setup_renderer(self) -> None:
        try:
            self._renderer = mujoco.Renderer(
                self.model, height=self._image_height, width=self._image_width)
        except Exception as e:
            print(f"[SimRunner] Renderer init failed: {e} — cameras will be unavailable")
            self._renderer = None
        # One depth renderer, reused sequentially for every wrist camera
        # (visual_reach block). Rendering is serial on this thread so a single
        # renderer can cover all arms.
        if self._wrist_specs and self._renderer is not None:
            try:
                self._depth_renderer = mujoco.Renderer(
                    self.model, height=self._image_height, width=self._image_width)
                self._depth_renderer.enable_depth_rendering()
            except Exception as e:
                print(f"[SimRunner] depth Renderer init failed: {e} — wrist depth unavailable")
                self._depth_renderer = None

    def _setup_viewer(self) -> None:
        if not self._show_viewer:
            return
        try:
            import mujoco.viewer as _mj_viewer
            self._viewer = _mj_viewer.launch_passive(self.model, self.data)
        except Exception as e:
            # Viewer는 환경 의존이라 실패해도 sim 자체는 살려둔다
            print(f"[SimRunner] viewer launch failed, continuing headless: {e}")
            self._viewer = None

    def _render_all_cameras(self) -> None:
        """Refresh frame buffers for all cameras (SimRunner thread only)."""
        if self._renderer is None:
            return
        new_frames: dict[int, np.ndarray] = {}
        for idx, cid in enumerate(self._camera_ids):
            try:
                self._renderer.update_scene(self.data, camera=cid)
                # render() returns a freshly-allocated ndarray; keep it as-is.
                new_frames[idx] = self._renderer.render()
            except Exception as e:
                # 한 카메라 실패해도 다른 카메라는 갱신 시도. 다음 tick에 재시도.
                print(f"[SimRunner] render failed for cam_idx={idx}: {e}")
        if new_frames:
            with self._buffer_lock:
                self._frame_buffers.update(new_frames)
        # Per-arm wrist depth + camera pose/intrinsics + ee_site snapshot
        # (visual_reach). One entry per wrist camera, rendered serially.
        if self._depth_renderer is not None and self._wrist_specs:
            new_wrist: dict[str, dict] = {}
            for spec in self._wrist_specs:
                try:
                    cid = self._camera_ids[spec["cam_idx"]]
                    self._depth_renderer.update_scene(self.data, camera=cid)
                    depth = self._depth_renderer.render()  # HxW float32, metric (m)
                    site_id = spec["site_id"]
                    ee_pos = (self.data.site_xpos[site_id].copy().tolist()
                              if site_id >= 0 else [0.0, 0.0, 0.0])
                    new_wrist[spec["name"]] = {
                        "depth": depth,
                        "meta": {
                            "cam_pos": self.data.cam_xpos[cid].copy().tolist(),
                            "cam_mat": self.data.cam_xmat[cid].copy().tolist(),  # 9, row-major
                            "fovy": float(self.model.cam_fovy[cid]),
                            "ee_pos": ee_pos,
                            "width": self._image_width,
                            "height": self._image_height,
                        },
                    }
                except Exception as e:
                    print(f"[SimRunner] wrist depth render failed ({spec['name']}): {e}")
            if new_wrist:
                with self._buffer_lock:
                    self._wrist_data.update(new_wrist)

    @property
    def wrist_cam_names(self) -> list[str]:
        return [s["name"] for s in self._wrist_specs]

    def get_wrist_rgbd(self, cam_name: Optional[str] = None) -> Optional[dict]:
        """Latest wrist RGB + depth + camera pose/intrinsics + ee_site pose for the
        given wrist camera (defaults to the first registered one).

        Returns None if there is no such wrist cam or nothing has been rendered
        yet. Read-only snapshot for the `visual_reach` block; no GL touched here.
        """
        if not self._wrist_specs:
            return None
        if cam_name is None:
            cam_name = self._wrist_specs[0]["name"]
        spec = next((s for s in self._wrist_specs if s["name"] == cam_name), None)
        if spec is None:
            return None
        with self._buffer_lock:
            rgb = self._frame_buffers.get(spec["cam_idx"])
            wd = self._wrist_data.get(cam_name)
        if rgb is None or wd is None:
            return None
        out = {"rgb": rgb, "depth": wd["depth"]}
        out.update(wd["meta"])
        return out

    def _run(self) -> None:
        # GL contexts must be created on this thread.
        self._setup_renderer()
        self._setup_viewer()

        wall_start = time.monotonic()
        sim_start = self.data.time
        next_render = time.monotonic()
        try:
            while not self._stop_event.is_set():
                if self._reset_objects_event.is_set():
                    self._do_reset_objects()
                    self._reset_objects_event.clear()
                # Scene randomization (model-tester collector). Done on this
                # thread, before stepping, so peg/hole_base land at the sampled
                # pose without racing mj_step.
                if self._randomize_request.is_set():
                    with self._target_lock:
                        self._randomize_apply(self._randomize_ranges)
                    wall_start = time.monotonic()
                    sim_start = self.data.time
                    self._randomize_request.clear()
                    self._randomize_done.set()
                # Honor reset requests on the physics thread so we don't race
                # mj_step on mjData. Resets arm + gripper + cube + velocities
                # via the 'home' keyframe.
                if self._reset_request.is_set():
                    with self._target_lock:
                        self._reset_to_home()
                        for i, j in enumerate(self._joints):
                            self._target[i] = float(self.data.ctrl[j.actuator_id])
                    wall_start = time.monotonic()
                    sim_start = self.data.time
                    self._reset_request.clear()
                    self._reset_done.set()
                self._apply_targets()
                mujoco.mj_step(self.model, self.data)
                self._update_grasp()   # kinematic auto-grasp (hold peg when gripped)

                if self._viewer is not None:
                    try:
                        if not self._viewer.is_running():
                            break
                        self._viewer.sync()
                    except Exception:
                        # viewer가 죽어도 sim은 계속
                        self._viewer = None

                now = time.monotonic()
                if now >= next_render:
                    self._render_all_cameras()
                    # 절대시간 기반 스케줄 — drift 누적 안 되도록.
                    next_render = max(next_render + self._render_period, now)

                if self._realtime_factor > 0:
                    expected_wall = (self.data.time - sim_start) / self._realtime_factor
                    drift = expected_wall - (time.monotonic() - wall_start)
                    if drift > 0:
                        time.sleep(min(drift, 0.005))
        finally:
            if self._viewer is not None:
                try:
                    self._viewer.close()
                except Exception:
                    pass
                self._viewer = None
            if self._renderer is not None:
                try:
                    self._renderer.close()
                except Exception:
                    pass
                self._renderer = None
