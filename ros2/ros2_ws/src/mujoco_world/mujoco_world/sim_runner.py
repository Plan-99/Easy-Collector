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
                 image_publish_hz: float = 20.0):
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
