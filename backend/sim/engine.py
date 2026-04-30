"""
PyBullet simulation engine for EasyTrainer.
Runs in a separate subprocess to prevent segfaults from crashing the main server.
Communicates via multiprocessing queues.
"""
import multiprocessing
import numpy as np
import time
import os


def _sim_worker(cmd_queue, frame_queue, state_queue):
    """Subprocess that runs PyBullet. Completely isolated from the main process."""
    import pybullet as p
    import pybullet_data

    client = None
    robot_id = None
    running = False
    controllable_joints = []

    # Camera defaults
    cam = {
        'width': 640, 'height': 480, 'fov': 60,
        'near': 0.01, 'far': 10.0,
        'target': [0, 0, 0.3], 'distance': 1.0,
        'yaw': 45, 'pitch': -30,
    }

    while True:
        # Process commands (non-blocking when running, blocking when idle)
        try:
            timeout = 0.001 if running else 0.1
            cmd = cmd_queue.get(timeout=timeout)
        except Exception:
            cmd = None

        if cmd is not None:
            action = cmd.get('action')

            if action == 'start':
                # Stop previous if any
                if client is not None:
                    try:
                        p.disconnect(client)
                    except Exception:
                        pass

                urdf_path = cmd['urdf_path']
                cam.update(cmd.get('cam_config') or {})

                try:
                    client = p.connect(p.DIRECT)
                    p.setAdditionalSearchPath(pybullet_data.getDataPath())
                    p.setGravity(0, 0, -9.81, physicsClientId=client)
                    p.loadURDF("plane.urdf", physicsClientId=client)

                    robot_id = p.loadURDF(
                        urdf_path, [0, 0, 0],
                        p.getQuaternionFromEuler([0, 0, 0]),
                        useFixedBase=True,
                        flags=p.URDF_USE_INERTIA_FROM_FILE,
                        physicsClientId=client,
                    )

                    # Enumerate joints
                    controllable_joints = []
                    for i in range(p.getNumJoints(robot_id, physicsClientId=client)):
                        info = p.getJointInfo(robot_id, i, physicsClientId=client)
                        if info[2] in (p.JOINT_REVOLUTE, p.JOINT_PRISMATIC):
                            controllable_joints.append({
                                'index': i,
                                'name': info[1].decode('utf-8'),
                                'type': int(info[2]),
                                'lower': float(info[8]),
                                'upper': float(info[9]),
                            })

                    running = True
                    state_queue.put({'status': 'running', 'joints': controllable_joints})
                except Exception as e:
                    state_queue.put({'status': 'error', 'error': str(e)})

            elif action == 'stop':
                running = False
                if client is not None:
                    try:
                        p.disconnect(client)
                    except Exception:
                        pass
                    client = None
                state_queue.put({'status': 'stopped'})

            elif action == 'set_joints':
                if running and robot_id is not None:
                    for idx, pos in cmd.get('positions', {}).items():
                        p.setJointMotorControl2(
                            robot_id, int(idx), p.POSITION_CONTROL,
                            targetPosition=float(pos), force=100,
                            physicsClientId=client,
                        )

            elif action == 'set_camera':
                cam.update(cmd.get('params', {}))

            elif action == 'quit':
                if client is not None:
                    try:
                        p.disconnect(client)
                    except Exception:
                        pass
                break

        # Step physics + render
        if running and client is not None:
            try:
                p.stepSimulation(physicsClientId=client)

                # Render frame
                view = p.computeViewMatrixFromYawPitchRoll(
                    cameraTargetPosition=cam['target'],
                    distance=cam['distance'],
                    yaw=cam['yaw'], pitch=cam['pitch'], roll=0,
                    upAxisIndex=2, physicsClientId=client,
                )
                proj = p.computeProjectionMatrixFOV(
                    fov=cam['fov'],
                    aspect=cam['width'] / cam['height'],
                    nearVal=cam['near'], farVal=cam['far'],
                    physicsClientId=client,
                )
                _, _, rgba, _, _ = p.getCameraImage(
                    width=cam['width'], height=cam['height'],
                    viewMatrix=view, projectionMatrix=proj,
                    renderer=p.ER_TINY_RENDERER,
                    physicsClientId=client,
                )

                rgba = np.array(rgba, dtype=np.uint8).reshape(cam['height'], cam['width'], 4)
                bgr = rgba[:, :, [2, 1, 0]]

                # Non-blocking put, drop old frames
                try:
                    while not frame_queue.empty():
                        frame_queue.get_nowait()
                    frame_queue.put_nowait(bgr)
                except Exception:
                    pass

            except Exception as e:
                print(f"[SimWorker] Error: {e}")

            time.sleep(1.0 / 30.0)  # ~30 FPS


class SimEngine:
    """Manages a PyBullet simulation in a subprocess."""

    def __init__(self):
        self._process = None
        self._cmd_queue = None
        self._frame_queue = None
        self._state_queue = None
        self._running = False
        self._joints = []

    @property
    def is_running(self):
        return self._running and self._process is not None and self._process.is_alive()

    def start(self, urdf_path: str, cam_config: dict | None = None):
        """Start simulation subprocess."""
        if self._running:
            self.stop()

        if not os.path.isfile(urdf_path):
            raise FileNotFoundError(f"URDF not found: {urdf_path}")

        ctx = multiprocessing.get_context('spawn')
        self._cmd_queue = ctx.Queue()
        self._frame_queue = ctx.Queue(maxsize=2)
        self._state_queue = ctx.Queue()

        self._process = ctx.Process(
            target=_sim_worker,
            args=(self._cmd_queue, self._frame_queue, self._state_queue),
            daemon=True,
        )
        self._process.start()

        self._cmd_queue.put({
            'action': 'start',
            'urdf_path': urdf_path,
            'cam_config': cam_config,
        })

        # Wait for response
        try:
            result = self._state_queue.get(timeout=15)
            if result.get('status') == 'error':
                self._process.terminate()
                raise RuntimeError(result.get('error', 'Unknown error'))
            self._joints = result.get('joints', [])
            self._running = True
        except Exception as e:
            if self._process.is_alive():
                self._process.terminate()
            raise

    def stop(self):
        """Stop simulation subprocess."""
        self._running = False
        if self._cmd_queue:
            try:
                self._cmd_queue.put({'action': 'quit'})
            except Exception:
                pass
        if self._process and self._process.is_alive():
            self._process.join(timeout=3)
            if self._process.is_alive():
                self._process.terminate()
        self._process = None
        self._joints = []

    def get_joint_info(self) -> list[dict]:
        return self._joints

    def set_joint_positions(self, positions: dict):
        if self.is_running:
            self._cmd_queue.put({'action': 'set_joints', 'positions': positions})

    def set_camera(self, **kwargs):
        if self.is_running:
            self._cmd_queue.put({'action': 'set_camera', 'params': kwargs})

    def render_frame(self) -> np.ndarray | None:
        """Get the latest rendered frame (non-blocking)."""
        if not self.is_running:
            return None
        try:
            return self._frame_queue.get_nowait()
        except Exception:
            return None
