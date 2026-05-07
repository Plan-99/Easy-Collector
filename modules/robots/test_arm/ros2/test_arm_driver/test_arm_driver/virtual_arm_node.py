"""Virtual 6-DOF arm node.

EasyTrainer 모듈 위자드 ROS 모드 검증용. 실제 하드웨어 없이 동작:
- /joint_cmd (sensor_msgs/JointState) 를 구독해 내부 joint state 를 갱신
- /joint_states (sensor_msgs/JointState) 로 내부 state 를 publish_rate(Hz) 주기로 publish
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


JOINT_NAMES = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']


class VirtualArmNode(Node):
    def __init__(self):
        super().__init__('virtual_arm_node')

        self.declare_parameter('publish_rate', 200.0)
        rate = float(self.get_parameter('publish_rate').value)

        self._positions = [0.0] * len(JOINT_NAMES)

        # /joint_states publish
        self._state_pub = self.create_publisher(JointState, 'joint_states', 10)
        # /joint_cmd subscribe
        self._cmd_sub = self.create_subscription(
            JointState, 'joint_cmd', self._on_cmd, 10
        )

        self._timer = self.create_timer(1.0 / rate, self._publish_state)
        self.get_logger().info(
            f'[{self.get_namespace()}] virtual_arm_node started '
            f'(publish_rate={rate} Hz, /joint_cmd → /joint_states)'
        )

    def _on_cmd(self, msg: JointState) -> None:
        if len(msg.position) == len(self._positions):
            self._positions = list(msg.position)
        else:
            self.get_logger().warn(
                f'/joint_cmd 길이 {len(msg.position)} != 기대값 {len(self._positions)}, 무시'
            )

    def _publish_state(self) -> None:
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = JOINT_NAMES
        msg.position = self._positions
        msg.velocity = [0.0] * len(JOINT_NAMES)
        msg.effort = [0.0] * len(JOINT_NAMES)
        self._state_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = VirtualArmNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
