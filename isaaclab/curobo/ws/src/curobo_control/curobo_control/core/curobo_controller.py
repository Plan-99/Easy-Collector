"""cuRobo 기반 로봇 컨트롤러.

MotionController 인터페이스를 구현하여
cuRobo MotionGen으로 trajectory를 생성하고
IsaacSim에 직접 joint command를 publish한다.

Bridge 불필요 — cuRobo가 trajectory를 직접 생성하므로
MoveIt FollowJointTrajectory 액션 서버를 거치지 않는다.
"""

import time

import rclpy
from sensor_msgs.msg import JointState

# cuRobo imports
from curobo.types.math import Pose as CuPose
from curobo.types.robot import JointState as CuJointState
from curobo.wrap.reacher.motion_gen import (
    MotionGen, MotionGenConfig, MotionGenPlanConfig,
)

import torch

from isaac_control_core.core.motion_controller import MotionController
from isaac_control_core.core.robot import RobotConfig


class CuroboController(MotionController):
    """cuRobo 기반 모션 컨트롤러.

    MotionController를 상속하여 cuRobo MotionGen으로 trajectory를 생성하고
    IsaacSim에 직접 joint command를 publish한다.

    Args:
        robot_config: RobotConfig 인스턴스.
        curobo_config_path: cuRobo 로봇 설정 YAML 경로.
        object_names: 추적할 오브젝트 이름 리스트.
        node_name: ROS2 노드 이름.
    """

    def __init__(
        self,
        robot_config: RobotConfig,
        curobo_config_path: str,
        object_names: list[str] | None = None,
        node_name: str = "curobo_controller",
    ):
        super().__init__(robot_config, object_names, node_name)

        # 마지막으로 명령한 그리퍼 너비 추적
        self._last_gripper_width = robot_config.gripper.open_width

        # 로봇별 그리퍼 스케일 설정
        self.GRIPPER_REAL_MAX = robot_config.gripper.open_width
        self.GRIPPER_SIM_MAX = self._GRIPPER_SIM_LIMITS.get(
            robot_config.name, 0.035
        )

        # IsaacSim 직접 통신 (sim 스케일 0~0.035)
        self._cmd_pub = self.create_publisher(
            JointState, "/simulation/joint_command", 10
        )
        # 외부용 토픽 (real 스케일 0~0.085) — 녹화/재생 호환
        self._joint_states_real_pub = self.create_publisher(
            JointState, "/joint_states", 10
        )
        self._joint_command_real_pub = self.create_publisher(
            JointState, "/joint_command", 10
        )

        # cuRobo MotionGen 초기화
        import os
        import yaml
        self._curobo_config_path = curobo_config_path
        self.get_logger().info(f"cuRobo MotionGen 초기화 중... ({curobo_config_path})")

        # 절대경로면 YAML dict로 로드, 아니면 cuRobo 내장 경로에서 찾기
        if os.path.isabs(curobo_config_path) or os.path.exists(curobo_config_path):
            with open(curobo_config_path, encoding="utf-8") as f:
                robot_cfg = yaml.safe_load(f)
            motion_gen_config = MotionGenConfig.load_from_robot_config(
                robot_cfg["robot_cfg"],
                interpolation_dt=0.01,
                num_trajopt_seeds=4,
                num_graph_seeds=4,
                rotation_threshold=0.2,   # ~11도 유격 허용
                position_threshold=0.005,  # 5mm
            )
        else:
            motion_gen_config = MotionGenConfig.load_from_robot_config(
                curobo_config_path,
                interpolation_dt=0.01,
                num_trajopt_seeds=4,
                num_graph_seeds=4,
                rotation_threshold=0.2,   # ~11도 유격 허용
                position_threshold=0.005,  # 5mm
            )
        self._motion_gen = MotionGen(motion_gen_config)
        self._motion_gen.warmup()

        # collision constraint 없을 때 check_start_state 에러 방지
        self._motion_gen.check_start_state = lambda *args, **kwargs: (True, None)

        self.get_logger().info("cuRobo MotionGen 준비 완료!")

        self._plan_config = MotionGenPlanConfig(
            enable_graph=True,
            enable_opt=True,
            max_attempts=500,
            check_start_validity=False,  # collision constraint 없을 때 에러 방지
        )

    # ── 플래너 대기 ────────────────────────────────────────────

    def _wait_for_planner(self):
        """cuRobo는 __init__에서 이미 warmup 완료."""
        self.get_logger().info("cuRobo MotionGen 이미 초기화됨.")

    # ── joint_state 콜백 오버라이드 (gripper sim→real 변환) ────

    def _joint_state_cb(self, msg):
        """IsaacSim의 sim 값(0~0.035)을 real 값(0~0.085)으로 변환하여 저장."""
        gripper_names = set(self._robot_config.gripper.joint_names)
        gripper_names.add("joint8")  # mimic joint

        converted = JointState()
        converted.header = msg.header
        converted.name = list(msg.name)
        converted.position = list(msg.position)
        converted.velocity = list(msg.velocity)
        converted.effort = list(msg.effort)

        for i, name in enumerate(converted.name):
            if name in gripper_names:
                converted.position[i] = abs(converted.position[i]) * (
                    self.GRIPPER_REAL_MAX / self.GRIPPER_SIM_MAX
                )

        self._current_joint_state = converted
        # real 스케일로 변환된 joint_states를 외부 토픽에 publish
        self._joint_states_real_pub.publish(converted)

    # ── 헬퍼 ──────────────────────────────────────────────────

    @property
    def gripper_length_override(self) -> float | None:
        # ee_link=tcp: PiperConfig 기본값(-0.05) 사용
        return -0.022

    @property
    def needs_orientation_correction(self) -> bool:
        return False

    def _publish_cmd(self, cmd: JointState):
        """joint command를 IsaacSim(sim)과 외부(real) 양쪽에 publish."""
        # IsaacSim에는 그대로 (이미 sim 스케일)
        self._cmd_pub.publish(cmd)

        # 외부 토픽에는 gripper를 real 스케일로 변환
        gripper_names = set(self._robot_config.gripper.joint_names)
        gripper_names.add("joint8")

        real_cmd = JointState()
        real_cmd.header = cmd.header
        real_cmd.name = list(cmd.name)
        real_cmd.position = list(cmd.position)

        for i, name in enumerate(real_cmd.name):
            if name in gripper_names:
                real_cmd.position[i] = abs(real_cmd.position[i]) * (
                    self.GRIPPER_REAL_MAX / self.GRIPPER_SIM_MAX
                )
        self._joint_command_real_pub.publish(real_cmd)

    def _get_current_cu_joint_state(self) -> CuJointState:
        """현재 관절 상태를 cuRobo JointState로 변환."""
        js = self._current_joint_state
        arm_names = self._robot_config.arm_joint_names
        pos_map = {js.name[i]: js.position[i] for i in range(len(js.name))}
        positions = [pos_map.get(n, 0.0) for n in arm_names]
        return CuJointState.from_position(
            position=torch.tensor([positions], dtype=torch.float32).cuda(),
            joint_names=arm_names,
        )

    def _execute_cu_trajectory(self, trajectory, speed_scale: float = 1.0) -> bool:
        """cuRobo trajectory를 IsaacSim에 publish.

        현재 위치 → trajectory 첫 점 blending + 감가속 프로필 적용.
        마지막 위치를 hold하여 물리 안정화.

        Args:
            speed_scale: 전체 속도 배율. 0.5=절반 속도, 1.0=원래 속도.
        """
        positions = trajectory.position.cpu().numpy()  # (T, num_joints)

        arm_names = self._robot_config.arm_joint_names
        gripper_names = self._robot_config.gripper.joint_names
        sim_gripper = self._real_to_sim_gripper(self._last_gripper_width)

        # 현재 실제 관절 위치 (trajectory 시작점과 다를 수 있음)
        js = self._current_joint_state
        if js is not None:
            js_map = {n: p for n, p in zip(js.name, js.position)}
            actual_start = [js_map.get(n, 0.0) for n in arm_names]
        else:
            actual_start = positions[0].tolist()

        traj_start = positions[0].tolist()
        start_diff = max(abs(a - t) for a, t in zip(actual_start, traj_start))

        base_dt = (1.0 / 60.0) / speed_scale

        # 1) Blending: 현재 위치 → trajectory 첫 점 (차이가 크면 보간)
        if start_diff > 0.05:  # 3도 이상 차이나면 blending
            self.get_logger().info(f"[DEBUG] blending: start_diff={start_diff:.3f} rad")
            n_blend = max(int(start_diff / 0.01), 10)  # 최소 10스텝
            for i in range(1, n_blend + 1):
                alpha = i / n_blend
                # smooth step (ease-in-out)
                alpha_smooth = alpha * alpha * (3.0 - 2.0 * alpha)
                blended = [a + alpha_smooth * (t - a)
                           for a, t in zip(actual_start, traj_start)]
                cmd = JointState()
                cmd.name = arm_names + gripper_names + ["joint8"]
                cmd.position = blended + [sim_gripper] + [-sim_gripper]
                self._publish_cmd(cmd)
                time.sleep(base_dt)

        # 2) 메인 trajectory 실행 (감가속 프로필)
        n_total = positions.shape[0]
        for t_idx in range(n_total):
            arm_pos = positions[t_idx].tolist()

            cmd = JointState()
            cmd.name = arm_names + gripper_names + ["joint8"]
            cmd.position = arm_pos + [sim_gripper] + [-sim_gripper]
            self._publish_cmd(cmd)

            progress = t_idx / max(n_total - 1, 1)
            # 처음 15% 가속, 마지막 25% 감속 (도착 시 충분히 느려짐)
            if progress < 0.15:
                speed_factor = 0.3 + 0.7 * (progress / 0.15)
            elif progress > 0.75:
                decel = (1.0 - progress) / 0.25  # 1.0 → 0.0
                speed_factor = 0.3 + 0.7 * decel
            else:
                speed_factor = 1.0
            time.sleep(base_dt / max(speed_factor, 0.1))

        # 3) 마지막 위치 hold (물리 안정화)
        final_pos = positions[-1].tolist()
        final_cmd = JointState()
        final_cmd.name = arm_names + gripper_names + ["joint8"]
        final_cmd.position = final_pos + [sim_gripper] + [-sim_gripper]
        for _ in range(40):
            self._publish_cmd(final_cmd)
            time.sleep(1.0 / 60.0)
        # 상태 동기화
        for _ in range(10):
            self._publish_cmd(final_cmd)
            rclpy.spin_once(self, timeout_sec=0.05)

        self.get_logger().info("Trajectory 실행 완료!")
        return True

    # ── 모션 인터페이스 ──────────────────────────────────────

    def move_to_joint(self, joint_positions: dict) -> bool:
        """관절 목표 위치로 이동."""
        self.get_logger().info("관절 이동 (cuRobo)")

        # 현재 관절 상태 최신화
        for _ in range(10):
            rclpy.spin_once(self, timeout_sec=0.05)

        arm_names = self._robot_config.arm_joint_names
        target_pos = [joint_positions.get(n, 0.0) for n in arm_names]
        goal = CuJointState.from_position(
            position=torch.tensor([target_pos], dtype=torch.float32).cuda(),
            joint_names=arm_names,
        )
        current = self._get_current_cu_joint_state()

        result = self._motion_gen.plan_single_js(current, goal, self._plan_config)

        if not result.success.item():
            self.get_logger().error("cuRobo 관절 플래닝 실패!")
            return False

        return self._execute_cu_trajectory(result.get_interpolated_plan())

    def move_to_pose(
        self, x: float, y: float, z: float,
        ox: float | None = None, oy: float | None = None,
        oz: float | None = None, ow: float | None = None,
        **kwargs,
    ) -> bool:
        """포즈 목표로 이동. rotation_threshold로 orientation 유격 허용."""
        if ox is None:
            current_ori = self.get_current_ee_orientation()
            if current_ori:
                ox, oy, oz, ow = current_ori
            else:
                grasp = self._robot_config.grasp_orientation
                if grasp:
                    ox, oy, oz, ow = grasp
                else:
                    ox, oy, oz, ow = 1.0, 0.0, 0.0, 0.0
        else:
            oy = oy or 0.0
            oz = oz or 0.0
            ow = ow or 0.0

        self.get_logger().info(
            f"포즈 이동 (cuRobo): ({x:.3f}, {y:.3f}, {z:.3f}), "
            f"ori=({ox:.3f}, {oy:.3f}, {oz:.3f}, {ow:.3f})"
        )

        # 현재 관절 상태 최신화
        for _ in range(10):
            rclpy.spin_once(self, timeout_sec=0.05)

        current = self._get_current_cu_joint_state()

        goal_pose = CuPose(
            position=torch.tensor([[x, y, z]], dtype=torch.float32).cuda(),
            quaternion=torch.tensor([[ow, ox, oy, oz]], dtype=torch.float32).cuda(),
        )

        result = self._motion_gen.plan_single(current, goal_pose, self._plan_config)

        if not result.success.item():
            self.get_logger().error(
                f"cuRobo 포즈 플래닝 실패! "
                f"target=({x:.3f}, {y:.3f}, {z:.3f}), "
                f"ori=({ox:.3f}, {oy:.3f}, {oz:.3f}, {ow:.3f}), "
                f"status={result.status if hasattr(result, 'status') else 'unknown'}"
            )
            return False

        traj = result.get_interpolated_plan()
        positions = traj.position.cpu().numpy()
        arm_names = self._robot_config.arm_joint_names
        start_joints = positions[0].tolist()
        end_joints = positions[-1].tolist()
        diffs = [abs(s - e) for s, e in zip(start_joints, end_joints)]
        max_diff = max(diffs)
        max_idx = diffs.index(max_diff)
        self.get_logger().info(
            f"[DEBUG] plan: start={[f'{v:.3f}' for v in start_joints]}"
        )
        self.get_logger().info(
            f"[DEBUG] plan: end  ={[f'{v:.3f}' for v in end_joints]}"
        )
        self.get_logger().info(
            f"[DEBUG] plan: {len(positions)} steps, max_diff={max_diff:.3f} at {arm_names[max_idx]}"
        )

        return self._execute_cu_trajectory(traj)

    def move_linear(
        self, x: float, y: float, z: float,
        ox: float | None = None, oy: float | None = None,
        oz: float | None = None, ow: float | None = None,
        velocity_scaling: float = 0.3,
        acceleration_scaling: float = 0.3,
        **kwargs,
    ) -> bool:
        """직선 경로로 이동 (IK + joint space 보간).

        현재 orientation을 유지하면서 목표까지 직선 이동.
        MotionGen 대신 IK로 목표 관절만 구하고 joint space에서 보간.
        """
        if ox is None:
            # TF 최신화 후 현재 orientation 읽기
            for _ in range(20):
                rclpy.spin_once(self, timeout_sec=0.05)
            current_ori = self.get_current_ee_orientation()
            if current_ori:
                ox, oy, oz, ow = current_ori
            else:
                grasp = self._robot_config.grasp_orientation
                if grasp:
                    ox, oy, oz, ow = grasp
                    self.get_logger().warn("[move_linear] TF 실패, grasp_orientation 폴백")
                else:
                    ox, oy, oz, ow = 0.0, 1.0, 0.0, 0.0
        else:
            oy = oy or 0.0
            oz = oz or 0.0
            ow = ow or 0.0

        self.get_logger().info(
            f"직선 이동 (IK): ({x:.3f}, {y:.3f}, {z:.3f}), "
            f"ori=({ox:.3f}, {oy:.3f}, {oz:.3f}, {ow:.3f})"
        )

        # 현재 관절 상태
        arm_names = self._robot_config.arm_joint_names
        gripper_names = self._robot_config.gripper.joint_names
        js = self._current_joint_state
        js_map = {n: p for n, p in zip(js.name, js.position)}
        current_arm = [js_map.get(n, 0.0) for n in arm_names]

        goal_pose = CuPose(
            position=torch.tensor([[x, y, z]], dtype=torch.float32).cuda(),
            quaternion=torch.tensor([[ow, ox, oy, oz]], dtype=torch.float32).cuda(),
        )
        current_cu = self._get_current_cu_joint_state()

        # retract_config: 해가 여러 개일 때 이 값에 가까운 해 선호
        # seed_config: IK 탐색 시작점 (현재 관절에서 시작)
        seed = current_cu.position.unsqueeze(1)  # (1, 1, n_dof)
        ik_result = self._motion_gen.ik_solver.solve_single(
            goal_pose,
            retract_config=current_cu.position,
            seed_config=seed,
            use_nn_seed=False,
        )

        if not ik_result.success.item():
            self.get_logger().warn(
                f"IK 실패: ({x:.3f}, {y:.3f}, {z:.3f}), MotionGen 폴백"
            )
            # 폴백 시 전달된 orientation 그대로 사용
            return self.move_to_pose(x, y, z, ox, oy, oz, ow)

        target_arm = ik_result.solution[0, 0].cpu().tolist()
        max_diff = max(abs(c - t) for c, t in zip(current_arm, target_arm))
        self.get_logger().info(f"[DEBUG] IK max_diff={max_diff:.3f} rad")

        if max_diff > 2.0:
            self.get_logger().warn(
                f"IK 해가 너무 다름 ({max_diff:.3f} rad), MotionGen 폴백"
            )
            return self.move_to_pose(x, y, z, ox, oy, oz, ow)

        # joint space 보간 (가감속 적용)
        sim_gripper = self._real_to_sim_gripper(self._last_gripper_width)
        n_steps = max(int(max_diff / 0.01), 30)  # 최소 30스텝
        base_dt = 1.0 / 60.0

        for i in range(1, n_steps + 1):
            alpha = i / n_steps
            interp = [c + alpha * (t_val - c) for c, t_val in zip(current_arm, target_arm)]

            cmd = JointState()
            cmd.name = arm_names + gripper_names + ["joint8"]
            cmd.position = interp + [sim_gripper] + [-sim_gripper]
            self._publish_cmd(cmd)

            # 가감속: 처음 15%, 마지막 15% 감속
            progress = i / n_steps
            if progress < 0.15:
                speed_factor = 0.3 + 0.7 * (progress / 0.15)
            elif progress > 0.85:
                speed_factor = 0.3 + 0.7 * ((1.0 - progress) / 0.15)
            else:
                speed_factor = 1.0
            time.sleep(base_dt / speed_factor)

        # 마지막 위치 hold (안정화) — 위로 튀는 현상 방지
        final_cmd = JointState()
        final_cmd.name = arm_names + gripper_names + ["joint8"]
        final_cmd.position = target_arm + [sim_gripper] + [-sim_gripper]
        for _ in range(50):
            self._publish_cmd(final_cmd)
            time.sleep(1.0 / 50.0)

        # 상태 동기화
        for _ in range(30):
            self._publish_cmd(final_cmd)
            rclpy.spin_once(self, timeout_sec=0.05)

        self.get_logger().info("직선 이동 완료!")
        return True

    def hold_position(self, duration: float = 0.3):
        """현재 관절 위치를 유지하면서 대기 (모션 간 튀는 현상 방지).

        time.sleep과 달리 매 프레임 joint command를 publish하여
        물리 시뮬레이션이 로봇을 움직이지 않게 한다.
        """
        js = self._current_joint_state
        if js is None:
            time.sleep(duration)
            return

        cmd = JointState()
        cmd.name = list(js.name)
        cmd.position = list(js.position)

        # gripper는 sim 값으로 변환
        arm_names = self._robot_config.arm_joint_names
        gripper_names = self._robot_config.gripper.joint_names
        sim_gripper = self._real_to_sim_gripper(self._last_gripper_width)

        js_map = {n: p for n, p in zip(js.name, js.position)}
        arm_pos = [js_map.get(n, 0.0) for n in arm_names]

        cmd = JointState()
        cmd.name = arm_names + gripper_names + ["joint8"]
        cmd.position = arm_pos + [sim_gripper] + [-sim_gripper]

        n_frames = int(duration * 60)  # 60Hz
        for _ in range(max(n_frames, 1)):
            self._publish_cmd(cmd)
            time.sleep(1.0 / 60.0)

    # 그리퍼 값 매핑 (기본값, __init__에서 로봇별로 업데이트)
    GRIPPER_REAL_MAX = 0.085
    GRIPPER_SIM_MAX = 0.035

    # 로봇별 sim max 매핑
    _GRIPPER_SIM_LIMITS = {
        "piper": 0.035,       # standard piper
        "piper_v100": 0.05,   # wide gripper
    }

    def _real_to_sim_gripper(self, width: float) -> float:
        """실제 로봇 그리퍼 값 → 시뮬레이션 값."""
        return width * (self.GRIPPER_SIM_MAX / self.GRIPPER_REAL_MAX)

    def set_gripper(self, width: float) -> bool:
        """그리퍼를 지정 너비로 이동 (직접 joint command publish)."""
        sim_width = self._real_to_sim_gripper(width)
        self.get_logger().info(f"그리퍼: {'열기' if width > 0.02 else '닫기'} (real={width:.4f}, sim={sim_width:.4f})")

        gripper_names = self._robot_config.gripper.joint_names
        if not gripper_names or not self._current_joint_state:
            self.get_logger().warn("그리퍼 관절 정보 없음")
            return False

        # 현재 전체 관절 상태 복사
        js_map = {n: p for n, p in zip(
            self._current_joint_state.name,
            self._current_joint_state.position
        )}

        # 그리퍼 관절 변경 (sim 스케일) + mimic joint 동기화
        for name in gripper_names:
            js_map[name] = sim_width
        # joint8 = -joint7 (mimic)
        if "joint8" in js_map:
            js_map["joint8"] = -sim_width

        # 1초간 10Hz로 publish (부드러운 이동)
        for _ in range(10):
            cmd = JointState()
            cmd.name = list(js_map.keys())
            cmd.position = list(js_map.values())
            self._publish_cmd(cmd)
            time.sleep(0.1)

        # 그리퍼 상태 저장 + joint state 동기화
        self._last_gripper_width = width
        for _ in range(10):
            rclpy.spin_once(self, timeout_sec=0.05)

        self.get_logger().info("그리퍼 완료!")
        return True