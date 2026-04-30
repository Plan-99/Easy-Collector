"""
MoveIt2 Python API를 사용한 Franka 제어 데모.

IsaacSim + trajectory_bridge + move_group이 실행 중인 상태에서 사용.

사용법 (MoveIt2 Docker 컨테이너 내부):
    ros2 run isaac_robot_control demo_control
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, PoseStamped
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    MotionPlanRequest,
    PlanningOptions,
    Constraints,
    JointConstraint,
    RobotState,
)
from sensor_msgs.msg import JointState
from rclpy.action import ActionClient
import time


class DemoControl(Node):
    def __init__(self):
        super().__init__("demo_control")

        self._move_group_client = ActionClient(self, MoveGroup, "/move_action")

        self._current_joint_state = None
        self.create_subscription(
            JointState, "/simulation/joint_states", self._joint_state_cb, 10
        )

        self.get_logger().info("MoveGroup 액션 서버 대기 중...")
        self._move_group_client.wait_for_server()
        self.get_logger().info("연결 완료!")

    def _joint_state_cb(self, msg: JointState):
        self._current_joint_state = msg

    def move_to_joint_positions(self, joint_positions: dict):
        """관절 목표 위치로 이동."""
        self.get_logger().info(f"관절 목표: {joint_positions}")

        goal = MoveGroup.Goal()
        request = MotionPlanRequest()
        request.group_name = "panda_arm"
        request.num_planning_attempts = 10
        request.allowed_planning_time = 5.0

        # 관절 제약 조건 설정
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

        future = self._move_group_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().error("목표가 거부됨!")
            return False

        self.get_logger().info("플래닝 + 실행 중...")
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result()

        if result.result.error_code.val == 1:  # SUCCESS
            self.get_logger().info("이동 완료!")
            return True
        else:
            self.get_logger().error(f"실패: error_code={result.result.error_code.val}")
            return False


# 미리 정의된 포즈들
POSES = {
    "home": {
        "panda_joint1": 0.0,
        "panda_joint2": -0.785,
        "panda_joint3": 0.0,
        "panda_joint4": -2.356,
        "panda_joint5": 0.0,
        "panda_joint6": 1.571,
        "panda_joint7": 0.785,
    },
    "ready": {
        "panda_joint1": 0.0,
        "panda_joint2": -0.5,
        "panda_joint3": 0.0,
        "panda_joint4": -1.5,
        "panda_joint5": 0.0,
        "panda_joint6": 1.8,
        "panda_joint7": 0.785,
    },
    "left": {
        "panda_joint1": -0.5,
        "panda_joint2": -0.5,
        "panda_joint3": -0.3,
        "panda_joint4": -2.0,
        "panda_joint5": -0.2,
        "panda_joint6": 1.8,
        "panda_joint7": 0.3,
    },
    "right": {
        "panda_joint1": 0.5,
        "panda_joint2": -0.5,
        "panda_joint3": 0.3,
        "panda_joint4": -2.0,
        "panda_joint5": 0.2,
        "panda_joint6": 1.8,
        "panda_joint7": 1.2,
    },
}


def main(args=None):
    rclpy.init(args=args)
    node = DemoControl()

    # joint_states가 들어올 때까지 대기
    node.get_logger().info("IsaacSim joint_states 대기 중...")
    while node._current_joint_state is None:
        rclpy.spin_once(node, timeout_sec=0.5)
    node.get_logger().info("joint_states 수신 확인!")

    print("\n" + "=" * 50)
    print("  Franka MoveIt2 데모 컨트롤러")
    print("=" * 50)
    print("\n사용 가능한 포즈: home, ready, left, right")
    print("종료: quit\n")

    try:
        while True:
            user_input = input("목표 포즈 입력> ").strip().lower()
            if user_input == "quit":
                break
            if user_input in POSES:
                node.move_to_joint_positions(POSES[user_input])
            else:
                print(f"알 수 없는 포즈: {user_input}")
                print(f"사용 가능: {list(POSES.keys())}")
    except (KeyboardInterrupt, EOFError):
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
