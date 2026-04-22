#!/usr/bin/env python3
"""
Right -> Left JointTrajectory bridge.

/fairino16_controller_right/joint_trajectory 를 구독해서,
각 joint position 에 sign/offset 변환을 적용한 뒤
/fairino16_controller_left/joint_trajectory 로 그대로 퍼블리시한다.

position 이외(joint_names, velocities, accelerations, effort, time_from_start,
header 등)는 모두 원본 그대로 전달한다.

실행:
    python3 src/backend/scripts/right_to_left_trajectory_bridge.py
    # 또는 파라미터 오버라이드:
    python3 src/backend/scripts/right_to_left_trajectory_bridge.py \
        --ros-args -p signs:="[1,-1,-1,1,-1,1]" -p offsets:="[0.0,0.0,0.0,0.0,0.0,0.0]"
"""

import copy

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory


IN_TOPICS = {
    # "/fairino16_controller_right/joint_trajectory": "/fairino16_controller_left/joint_trajectory",
    "/admittance/planned_trajectory_right": "/admittance/planned_trajectory_left",
}

DEFAULT_SIGNS = [-1.0, -1.0, -1.0, -1.0, -1.0, -1.0]
DEFAULT_OFFSETS = [0.0, -3.14159, 0.0, -3.14159, 0.0, 0.0]


class RightToLeftBridge(Node):
    def __init__(self) -> None:
        super().__init__("right_to_left_trajectory_bridge")

        self.declare_parameter("signs", DEFAULT_SIGNS)
        self.declare_parameter("offsets", DEFAULT_OFFSETS)
        self.signs = list(self.get_parameter("signs").value)
        self.offsets = list(self.get_parameter("offsets").value)

        if len(self.signs) != len(self.offsets):
            raise ValueError(
                f"signs({len(self.signs)}) and offsets({len(self.offsets)}) length mismatch"
            )

        self.pubs = {}
        for in_topic, out_topic in IN_TOPICS.items():
            pub = self.create_publisher(JointTrajectory, out_topic, 10)
            self.pubs[in_topic] = pub
            self.create_subscription(
                JointTrajectory, in_topic,
                lambda msg, p=pub: self._on_msg(msg, p), 10
            )
            self.get_logger().info(f"Bridging {in_topic} -> {out_topic}")

        self.get_logger().info(
            f"  signs   = {self.signs}\n"
            f"  offsets = {self.offsets}"
        )

    def _on_msg(self, msg: JointTrajectory, pub) -> None:
        out = copy.deepcopy(msg)

        out.joint_names = [
            name.replace("_right", "_left") for name in out.joint_names
        ]

        n = len(self.signs)
        for point in out.points:
            if not point.positions:
                continue
            if len(point.positions) != n:
                self.get_logger().warn(
                    f"position length {len(point.positions)} != signs length {n}; "
                    f"passing through without transform"
                )
                continue
            transformed = [
                self.signs[i] * float(point.positions[i]) + self.offsets[i]
                for i in range(n)
            ]
            point.positions = transformed

        pub.publish(out)


def main() -> None:
    rclpy.init()
    node = RightToLeftBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
