import subprocess
import os
import signal
import threading
import time
import numpy as np
from scipy.spatial.transform import Rotation as R


class ViveController:
    """
    VIVE 컨트롤러 텔레오퍼레이션 클래스.

    ros2 vive_udp_bridge 노드의 서브프로세스 실행부터 ROS2 토픽 구독,
    delta 계산, 에이전트 제어까지 vive 관련 모든 lifecycle을 캡슐화한다.
    """

    ROS2_PACKAGE = 'vive_udp_bridge'
    ROS2_NODE = 'udp_json_to_ros2'
    TOPIC_TEMPLATE = '/vive/{role}/joint_states'  # /vive/<key>/joint_states (JointState)를 구독한다.

    # SteamVR(Y-up)에서 로봇(Z-up) 좌표계로 변환하는 기본 행렬
    # vive X → robot X,  vive Z → -robot Y,  vive Y → robot Z
    DEFAULT_FRAME_TRANSFORM = np.array([
        [1,  0,  0],   # robot X = vive X
        [0,  0, -1],   # robot Y = -vive Z  (부호 반전)
        [0,  1,  0],   # robot Z = vive Y
    ], dtype=float)

    def __init__(self, node, socketio_instance, role='right_wrist', frame_transform=None, scale_factor=1.0, step_rate=30,
                 smoothing_alpha=0.1, deadzone_pos=0.001, deadzone_rot=0.001):
        """
        frame_transform: vive 좌표계 → 로봇 좌표계 변환 3x3 행렬 (None이면 DEFAULT 사용).
        scale_factor: position/rotation delta에 곱해지는 배율.
        step_rate: start_teleop() 백그라운드 스레드의 step() 호출 주기 (Hz).
        smoothing_alpha: EMA 필터 계수 (0~1). 작을수록 더 부드럽지만 반응이 느림.
        deadzone_pos: position delta의 deadzone 임계값 (m). 이 이하의 움직임은 무시.
        deadzone_rot: rotation delta의 deadzone 임계값 (rad). 이 이하의 회전은 무시.
        """
        self.node = node
        self.socketio_instance = socketio_instance
        self.role = role

        T = self.DEFAULT_FRAME_TRANSFORM if frame_transform is None else np.array(frame_transform, dtype=float)
        self._T = T
        self._T_inv = T.T  # T는 orthogonal 행렬이므로 T^-1 = T^T
        self._scale = scale_factor
        self._step_interval = 1.0 / step_rate

        self._sub = None
        self._process = None
        self._current_pose = None   # [x, y, z, qx, qy, qz, qw]
        self._vive_origin_pose = None  # origin 시점의 vive 포즈
        self._lock = threading.Lock()
        self._ready_event = threading.Event()
        self._teleop_running = False
        self._teleop_thread = None
        self._on_step = None        # callback(offset: [dx, dy, dz, dax, day, daz])
        self._smoothing_alpha = smoothing_alpha
        self._deadzone_pos = deadzone_pos
        self._deadzone_rot = deadzone_rot
        self._prev_offset = [0.0] * 6  # 직전 step의 cumulative offset (robot frame)
        self._last_step_delta = [0.0] * 6  # 직전 step 대비 현재 포즈 변화량 (필터 적용 후)
        self._filtered_delta = np.zeros(6)  # EMA 필터 상태

    def _start_ros_node(self):
        """vive_udp_bridge ROS2 노드를 서브프로세스로 실행한다."""
        popen_args = {
            'stdout': subprocess.PIPE,
            'stderr': subprocess.PIPE,
            'text': True,
            'bufsize': 1,
        }
        if os.name != 'nt':
            popen_args['preexec_fn'] = os.setsid

        self._process = subprocess.Popen(
            ['ros2', 'run', self.ROS2_PACKAGE, self.ROS2_NODE],
            **popen_args,
        )

        def _reader(stream, tag):
            for line in stream:
                line = line.strip()
                if line:
                    print(f'[vive_node {tag}] {line}')

        threading.Thread(target=_reader, args=(self._process.stdout, 'stdout'), daemon=True).start()
        threading.Thread(target=_reader, args=(self._process.stderr, 'stderr'), daemon=True).start()

    def wait_for_ready(self, timeout=30.0) -> bool:
        """
        vive_node 프로세스를 실행한 뒤 ROS2 토픽 구독을 시작하고,
        첫 메시지가 수신될 때까지 대기한다.
        timeout(초) 내에 수신되지 않으면 False를 반환하고 정리한다.
        """
        from sensor_msgs.msg import JointState as ViveJointState

        self._start_ros_node()

        topic = self.TOPIC_TEMPLATE.format(role=self.role)
        self._sub = self.node.create_subscription(ViveJointState, topic, self._callback, 10)
        print(f'[VIVE] Waiting for controller on {topic} ...')

        if not self._ready_event.wait(timeout=timeout):
            print('[VIVE] Controller not detected within timeout. Stopping.')
            self.destroy()
            self.socketio_instance.emit('vive_node_error', {'message': 'VIVE controller not detected'})
            return False

        self.socketio_instance.emit('vive_node_ready', {})
        print('[VIVE] Controller ready!')
        return True

    def _callback(self, msg):
        try:
            with self._lock:
                self._current_pose = list(msg.position[:7])  # [x, y, z, qx, qy, qz, qw]
            self._ready_event.set()
        except Exception as e:
            print(f"[ERROR] vive _callback: {e}")

    def start_teleop(self, on_step=None):
        """
        step()을 별도 스레드에서 step_rate Hz로 실행한다.
        on_step(offset): offset = [dx, dy, dz, dax, day, daz] — vive origin 대비 현재 cumulative offset.
        on_step이 None이면 last_step_delta만 갱신한다.
        """
        self._on_step = on_step
        self._teleop_running = True
        self._teleop_thread = threading.Thread(target=self._teleop_loop, daemon=True)
        self._teleop_thread.start()

    def _teleop_loop(self):
        while self._teleop_running:
            t0 = time.monotonic()
            try:
                self.step()
            except Exception as e:
                print(f"[ERROR] vive _teleop_loop step: {e}")
            elapsed = time.monotonic() - t0
            sleep_time = self._step_interval - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

    def stop_teleop(self):
        """텔레오퍼레이션 스레드를 중지한다."""
        self._teleop_running = False
        if self._teleop_thread is not None:
            self._teleop_thread.join(timeout=2.0)
            self._teleop_thread = None

    def set_origin(self):
        """현재 vive 포즈를 origin으로 기록하고 step delta를 초기화한다."""
        with self._lock:
            self._vive_origin_pose = list(self._current_pose) if self._current_pose else None
        self._prev_offset = [0.0] * 6
        self._last_step_delta = [0.0] * 6
        self._filtered_delta = np.zeros(6)
        print(f'[VIVE] Origin set. vive_pos={self._vive_origin_pose[:3] if self._vive_origin_pose else None}')

    @property
    def last_step_delta(self):
        """직전 step 대비 현재 vive 포즈 변화량 (robot frame, [dx, dy, dz, dax, day, daz])."""
        return self._last_step_delta

    def get_offset(self):
        """origin 기준 현재 누적 offset (robot frame, [dx, dy, dz, dax, day, daz]).
        record_episode에서 연속 두 호출의 차이로 ee_delta_action을 계산할 때 사용한다."""
        return list(self._prev_offset)

    def step(self):
        """
        현재 vive 포즈와 origin 간의 cumulative offset과 step-wise delta를 계산한다.
        """
        with self._lock:
            curr_pose = list(self._current_pose) if self._current_pose else None

        if curr_pose is None or self._vive_origin_pose is None:
            return

        # position offset: vive 좌표 → 로봇 좌표로 변환
        delta_xyz_vive = np.array([curr_pose[j] - self._vive_origin_pose[j] for j in range(3)])
        offset_xyz = (self._scale * self._T @ delta_xyz_vive).tolist()

        # rotation offset: T @ dR_vive @ T^T (similarity transform)
        r_origin = R.from_quat(self._vive_origin_pose[3:7])
        r_curr = R.from_quat(curr_pose[3:7])
        dR_vive_mat = (r_curr * r_origin.inv()).as_matrix()
        dR_robot_mat = self._T @ dR_vive_mat @ self._T_inv
        offset_axayaz = (self._scale * R.from_matrix(dR_robot_mat).as_rotvec()).tolist()

        offset = offset_xyz + offset_axayaz

        # step-wise delta: 이전 cumulative offset 대비 현재 변화량
        delta_xyz = [offset[i] - self._prev_offset[i] for i in range(3)]
        prev_R = R.from_rotvec(self._prev_offset[3:6])
        curr_R = R.from_rotvec(offset_axayaz)
        delta_axayaz = (curr_R * prev_R.inv()).as_rotvec().tolist()
        raw_delta = np.array(delta_xyz + delta_axayaz)

        # deadzone: 임계값 이하의 미세한 움직임 무시
        pos_norm = np.linalg.norm(raw_delta[:3])
        rot_norm = np.linalg.norm(raw_delta[3:])
        if pos_norm < self._deadzone_pos:
            raw_delta[:3] = 0.0
        if rot_norm < self._deadzone_rot:
            raw_delta[3:] = 0.0

        # EMA low-pass filter: filtered = alpha * raw + (1 - alpha) * prev_filtered
        alpha = self._smoothing_alpha
        self._filtered_delta = alpha * raw_delta + (1.0 - alpha) * self._filtered_delta
        self._last_step_delta = self._filtered_delta.tolist()

        self._prev_offset = offset

        if self._on_step is not None:
            self._on_step(offset)

    def destroy(self):
        """텔레오퍼레이션 스레드 중지, ROS2 구독 해제, vive_node 서브프로세스를 종료한다."""
        self.stop_teleop()
        if self._sub is not None:
            self.node.destroy_subscription(self._sub)
            self._sub = None

        if self._process is not None:
            try:
                if os.name != 'nt':
                    os.killpg(os.getpgid(self._process.pid), signal.SIGTERM)
                else:
                    self._process.terminate()
                self._process.wait(timeout=5)
            except Exception:
                try:
                    self._process.kill()
                except Exception:
                    pass
            self._process = None
            print('[VIVE] vive_node terminated.')
