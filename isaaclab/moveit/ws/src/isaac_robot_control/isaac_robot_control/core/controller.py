"""MoveIt 기반 로봇 컨트롤러.

MotionController를 상속하여 MoveGroup, Cartesian, 그리퍼 제어를 수행합니다.
어떤 로봇이든 RobotConfig만 구현하면 동일한 컨트롤러를 재사용할 수 있습니다.
"""

import copy
import time

import rclpy
from rclpy.action import ActionClient

from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    MotionPlanRequest,
    PlanningOptions,
    Constraints,
    JointConstraint,
    RobotState,
)
from moveit_msgs.srv import GetCartesianPath
from geometry_msgs.msg import Pose
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration

from isaac_control_core.core.robot import RobotConfig
from isaac_control_core.core.motion_controller import MotionController


class MoveItController(MotionController):
    """MoveIt 기반 모션 컨트롤러.

    Args:
        robot_config: 로봇 설정 (관절 이름, 토픽, 프레임 등).
        object_names: 추적할 오브젝트 이름 리스트 (Marker 토픽 구독).
        node_name: ROS2 노드 이름.
    """

    def __init__(
        self,
        robot_config: RobotConfig,
        object_names: list[str] | None = None,
        node_name: str = "robot_controller",
    ):
        super().__init__(robot_config, object_names, node_name)

        # MoveGroup 액션 클라이언트
        self._move_group_client = ActionClient(self, MoveGroup, "/move_action")

        # Arm trajectory 액션 클라이언트
        self._arm_trajectory_client = ActionClient(
            self, FollowJointTrajectory,
            robot_config.arm_controller_topic,
        )

        # 그리퍼 액션 클라이언트
        self._gripper_client = ActionClient(
            self, FollowJointTrajectory,
            robot_config.gripper.controller_topic,
        )

        # Cartesian path 서비스 클라이언트
        self._cartesian_client = self.create_client(
            GetCartesianPath, "/compute_cartesian_path",
        )

    # ── 플래너 대기 ───────────────────────────────────────────

    def _wait_for_planner(self):
        """MoveIt 서버/서비스 대기."""
        self.get_logger().info("MoveGroup 액션 서버 대기 중...")
        self._move_group_client.wait_for_server()
        self.get_logger().info("Arm trajectory 서버 대기 중...")
        self._arm_trajectory_client.wait_for_server()
        self.get_logger().info("그리퍼 액션 서버 대기 중...")
        self._gripper_client.wait_for_server()
        self.get_logger().info("Cartesian path 서비스 대기 중...")
        self._cartesian_client.wait_for_service()

    # ── MoveGroup (자유 공간 이동) ─────────────────────────────

    def move_to_joint(self, joint_positions: dict) -> bool:
        """관절 목표 위치로 이동 (OMPL 플래닝)."""
        self.get_logger().info("관절 이동 (MoveGroup)")

        goal = MoveGroup.Goal()
        request = MotionPlanRequest()
        request.group_name = self._robot_config.move_group_name
        request.num_planning_attempts = 10
        request.allowed_planning_time = 5.0

        constraints = Constraints()
        for joint_name, position in joint_positions.items():
            jc = JointConstraint()
            jc.joint_name = joint_name
            jc.position = position
            jc.tolerance_above = 0.01
            jc.tolerance_below = 0.01
            jc.weight = 1.0
            constraints.joint_constraints.append(jc)

        request.goal_constraints.append(constraints)
        goal.request = request
        goal.planning_options = PlanningOptions()
        goal.planning_options.plan_only = False

        return self._send_move_goal(goal)

    def move_to_pose(
        self, x: float, y: float, z: float,
        ox: float | None = None, oy: float | None = None,
        oz: float | None = None, ow: float | None = None,
    ) -> bool:
        """포즈 목표로 이동 (Cartesian 우선, 실패 시 MoveGroup 폴백)."""
        if ox is None:
            current_ori = self.get_current_ee_orientation()
            if current_ori:
                ox, oy, oz, ow = current_ori
            else:
                ox, oy, oz, ow = 1.0, 0.0, 0.0, 0.0
        else:
            oy = oy or 0.0
            oz = oz or 0.0
            ow = ow or 0.0

        self.get_logger().info(f"포즈 이동: ({x:.3f}, {y:.3f}, {z:.3f})")

        target = Pose()
        target.position.x = x
        target.position.y = y
        target.position.z = z
        target.orientation.x = ox
        target.orientation.y = oy
        target.orientation.z = oz
        target.orientation.w = ow

        success = self._move_cartesian([target])
        if success:
            return True

        self.get_logger().warn("Cartesian 실패, MoveGroup으로 폴백")
        return self._move_with_movegroup(x, y, z, ox, oy, oz, ow)

    def move_linear(
        self, x: float, y: float, z: float,
        ox: float | None = None, oy: float | None = None,
        oz: float | None = None, ow: float | None = None,
        velocity_scaling: float = 0.3,
        acceleration_scaling: float = 0.3,
    ) -> bool:
        """Cartesian 직선 경로로 이동."""
        if ox is None:
            current_ori = self.get_current_ee_orientation()
            if current_ori:
                ox, oy, oz, ow = current_ori
                self.get_logger().info(
                    f"직선 이동 (Cartesian): ({x:.3f}, {y:.3f}, {z:.3f}), "
                    f"현재 자세 유지 quat=({ox:.3f}, {oy:.3f}, {oz:.3f}, {ow:.3f})"
                )
            else:
                ox, oy, oz, ow = 1.0, 0.0, 0.0, 0.0
                self.get_logger().info(f"직선 이동 (Cartesian): ({x:.3f}, {y:.3f}, {z:.3f}), 기본 자세")
        else:
            oy = oy or 0.0
            oz = oz or 0.0
            ow = ow or 0.0
            self.get_logger().info(f"직선 이동 (Cartesian): ({x:.3f}, {y:.3f}, {z:.3f})")

        target = Pose()
        target.position.x = x
        target.position.y = y
        target.position.z = z
        target.orientation.x = ox
        target.orientation.y = oy
        target.orientation.z = oz
        target.orientation.w = ow

        return self._move_cartesian(
            [target],
            velocity_scaling=velocity_scaling,
            acceleration_scaling=acceleration_scaling,
        )

    def set_gripper(self, width: float) -> bool:
        """그리퍼를 지정 너비로 이동."""
        self.get_logger().info(f"그리퍼: {'열기' if width > 0.02 else '닫기'} ({width:.4f})")

        gripper_cfg = self._robot_config.gripper

        goal = FollowJointTrajectory.Goal()
        trajectory = JointTrajectory()
        trajectory.joint_names = gripper_cfg.joint_names

        point = JointTrajectoryPoint()
        point.positions = [width] * len(gripper_cfg.joint_names)
        point.time_from_start = Duration(sec=1, nanosec=0)
        trajectory.points.append(point)

        goal.trajectory = trajectory

        future = self._gripper_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().error("그리퍼 목표 거부됨!")
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result()

        if result.result.error_code == FollowJointTrajectory.Result.SUCCESSFUL:
            self.get_logger().info("그리퍼 완료!")
            return True
        else:
            self.get_logger().warn(f"그리퍼 결과: {result.result.error_code}")
            return True  # 물체에 막혀도 성공 처리

    # ── MoveIt 내부 메서드 ────────────────────────────────────

    def _move_with_movegroup(
        self, x, y, z, ox, oy, oz, ow,
        orientation_tolerance: float = 0.35,
    ) -> bool:
        """MoveGroup 액션으로 포즈 이동 (폴백용)."""
        from moveit_msgs.msg import PositionConstraint, OrientationConstraint, BoundingVolume
        from shape_msgs.msg import SolidPrimitive

        cfg = self._robot_config

        goal = MoveGroup.Goal()
        request = MotionPlanRequest()
        request.group_name = cfg.move_group_name
        request.num_planning_attempts = 20
        request.allowed_planning_time = 10.0

        position_constraint = PositionConstraint()
        position_constraint.header.frame_id = cfg.base_frame
        position_constraint.link_name = cfg.ee_frame
        position_constraint.weight = 1.0

        bounding_volume = BoundingVolume()
        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.SPHERE
        primitive.dimensions = [0.02]
        bounding_volume.primitives.append(primitive)

        target_pose = Pose()
        target_pose.position.x = x
        target_pose.position.y = y
        target_pose.position.z = z
        bounding_volume.primitive_poses.append(target_pose)
        position_constraint.constraint_region = bounding_volume

        orientation_constraint = OrientationConstraint()
        orientation_constraint.header.frame_id = cfg.base_frame
        orientation_constraint.link_name = cfg.ee_frame
        orientation_constraint.orientation.x = ox
        orientation_constraint.orientation.y = oy
        orientation_constraint.orientation.z = oz
        orientation_constraint.orientation.w = ow
        orientation_constraint.absolute_x_axis_tolerance = orientation_tolerance
        orientation_constraint.absolute_y_axis_tolerance = orientation_tolerance
        orientation_constraint.absolute_z_axis_tolerance = 0.5
        orientation_constraint.weight = 1.0

        constraints = Constraints()
        constraints.position_constraints.append(position_constraint)
        constraints.orientation_constraints.append(orientation_constraint)

        request.goal_constraints.append(constraints)
        goal.request = request
        goal.planning_options = PlanningOptions()
        goal.planning_options.plan_only = False

        return self._send_move_goal(goal)

    def _send_move_goal(self, goal: MoveGroup.Goal) -> bool:
        """MoveGroup 액션 전송 및 결과 대기."""
        req = goal.request
        self.get_logger().info(
            f"[DEBUG] group={req.group_name}, "
            f"joint_constraints={[jc.joint_name for jc in (req.goal_constraints[0].joint_constraints if req.goal_constraints else [])]},"
            f" planning_attempts={req.num_planning_attempts}"
        )
        future = self._move_group_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        goal_handle = future.result()

        if goal_handle is None:
            self.get_logger().error("MoveGroup 서버 응답 없음 (goal_handle=None)!")
            return False

        if not goal_handle.accepted:
            self.get_logger().error("목표가 거부됨!")
            return False

        self.get_logger().info("플래닝 + 실행 중...")
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result()

        err = result.result.error_code.val
        if err == 1:  # SUCCESS
            self.get_logger().info("이동 완료!")
            return True
        else:
            error_names = {
                -1: "FAILURE", -2: "PLANNING_FAILED", -3: "INVALID_MOTION_PLAN",
                -4: "MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE",
                -5: "CONTROL_FAILED", -6: "UNABLE_TO_AQUIRE_SENSOR_DATA",
                -7: "TIMED_OUT", -10: "PREEMPTED", -12: "INVALID_GROUP_NAME",
                -31: "INVALID_ROBOT_STATE", -33: "INVALID_GOAL_CONSTRAINTS",
                99999: "UNDEFINED/NOT_INITIALIZED",
            }
            self.get_logger().error(
                f"이동 실패: error_code={err} ({error_names.get(err, 'UNKNOWN')})"
            )
            if hasattr(result.result, 'planning_time'):
                self.get_logger().error(f"  planning_time={result.result.planning_time:.3f}s")
            return False

    def _move_cartesian(
        self, waypoints: list, max_step: float = 0.01,
        min_fraction: float = 0.7,
        velocity_scaling: float = 0.3,
        acceleration_scaling: float = 0.3,
    ) -> bool:
        """Cartesian 직선 경로 계획 + 실행."""
        cfg = self._robot_config

        start_state = RobotState()
        start_state.joint_state = copy.deepcopy(self._current_joint_state)

        request = GetCartesianPath.Request()
        request.header.frame_id = cfg.base_frame
        request.group_name = cfg.move_group_name
        request.link_name = cfg.ee_frame
        request.start_state = start_state
        request.waypoints = waypoints
        request.max_step = max_step
        request.avoid_collisions = True
        request.max_velocity_scaling_factor = velocity_scaling
        request.max_acceleration_scaling_factor = acceleration_scaling

        future = self._cartesian_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        response = future.result()

        fraction = response.fraction
        self.get_logger().info(f"Cartesian 경로: {fraction * 100:.1f}% 달성")

        if fraction < min_fraction:
            self.get_logger().warn(f"Cartesian 경로 부족 ({fraction * 100:.1f}%)")
            return False

        return self._execute_trajectory(response.solution)

    def _execute_trajectory(self, trajectory) -> bool:
        """RobotTrajectory를 FollowJointTrajectory 액션으로 실행."""
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = trajectory.joint_trajectory

        future = self._arm_trajectory_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().error("Trajectory 실행 거부됨!")
            return False

        self.get_logger().info("Trajectory 실행 중...")
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result()

        if result.result.error_code == FollowJointTrajectory.Result.SUCCESSFUL:
            self.get_logger().info("실행 완료!")
            return True
        else:
            self.get_logger().error(f"실행 실패: {result.result.error_code}")
            return False


# 하위 호환 alias
RobotController = MoveItController
