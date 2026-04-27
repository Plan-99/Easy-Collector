"""
MuJoCo simulation runner.

Wraps a MuJoCo model so the ROS2 node only deals with high-level semantics:
- read joint states (positions + velocities)
- write joint position targets
- render the scene through a named camera

Kept independent of rclpy so it can be unit-tested or reused from a different
transport (e.g. a future gRPC/shm streaming path) without changes.
"""
from __future__ import annotations

import threading
from dataclasses import dataclass
from typing import Sequence

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

    The physics loop runs on its own thread at the model's timestep. ROS callbacks
    only mutate the latest command target through `set_targets`, which is cheap
    and lock-free enough at our control rates.
    """

    def __init__(self, model_path: str, joint_names: Sequence[str], camera_name: str,
                 image_width: int = 640, image_height: int = 480,
                 realtime_factor: float = 1.0, show_viewer: bool = False):
        self.model = mujoco.MjModel.from_xml_path(model_path)
        self.data = mujoco.MjData(self.model)
        self._show_viewer = bool(show_viewer)
        self._viewer = None

        # Reset to keyframe "home" if present
        try:
            home_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_KEY, "home")
            if home_id >= 0:
                mujoco.mj_resetDataKeyframe(self.model, self.data, home_id)
        except Exception:
            mujoco.mj_resetData(self.model, self.data)

        self._joints = self._build_joint_specs(joint_names)
        self._target = np.array([self.data.ctrl[j.actuator_id] for j in self._joints], dtype=np.float64)

        self._camera_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_CAMERA, camera_name)
        if self._camera_id < 0:
            raise ValueError(f"Camera '{camera_name}' not found in MJCF.")

        self._image_width = int(image_width)
        self._image_height = int(image_height)
        self._renderer: mujoco.Renderer | None = None
        self._render_lock = threading.Lock()

        self._target_lock = threading.Lock()
        self._stop_event = threading.Event()
        self._thread: threading.Thread | None = None
        self._realtime_factor = float(realtime_factor)

    def _build_joint_specs(self, joint_names: Sequence[str]) -> list[JointSpec]:
        specs: list[JointSpec] = []
        for name in joint_names:
            jid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, name)
            if jid < 0:
                raise ValueError(f"Joint '{name}' not found in MJCF.")
            qpos_addr = int(self.model.jnt_qposadr[jid])
            qvel_addr = int(self.model.jnt_dofadr[jid])

            # Find an actuator that drives this joint (position actuator -> trnid points to joint)
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

    def render_rgb(self) -> np.ndarray:
        """Render an RGB frame from the configured camera (HxWx3, uint8)."""
        with self._render_lock:
            if self._renderer is None:
                self._renderer = mujoco.Renderer(self.model, height=self._image_height,
                                                 width=self._image_width)
            self._renderer.update_scene(self.data, camera=self._camera_id)
            return self._renderer.render()

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
        with self._render_lock:
            if self._renderer is not None:
                self._renderer.close()
                self._renderer = None

    def _run(self) -> None:
        import time
        if self._show_viewer:
            try:
                import mujoco.viewer as _mj_viewer
                self._viewer = _mj_viewer.launch_passive(self.model, self.data)
            except Exception as e:
                # Viewer는 환경 의존이라 실패해도 sim 자체는 살려둔다
                print(f"[SimRunner] viewer launch failed, continuing headless: {e}")
                self._viewer = None
        wall_start = time.monotonic()
        sim_start = self.data.time
        try:
            while not self._stop_event.is_set():
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
