import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import time


class TestArmNode(Node):
    def __init__(self):
        super().__init__('test_arm_node')

        self.joint_names = [
            'joint1', 'joint2', 'joint3',
            'joint4', 'joint5', 'joint6', 'gripper'
        ]
        self.joint_positions = [0.0] * 7

        self.declare_parameter('publish_rate', 50.0)
        publish_rate = self.get_parameter('publish_rate').value

        # joint_states_single 퍼블리셔 (현재 관절 상태)
        self.state_pub = self.create_publisher(JointState, 'joint_states_single', 10)

        # joint_states 구독 (명령 수신)
        self.cmd_sub = self.create_subscription(
            JointState, 'joint_states', self.cmd_callback, 10
        )

        # 주기적으로 현재 상태 퍼블리시
        timer_period = 1.0 / publish_rate
        self.timer = self.create_timer(timer_period, self.publish_state)

        self.get_logger().info(
            f'[{self.get_namespace()}] Test arm node started. '
            f'Publishing on joint_states_single, subscribing on joint_states'
        )

    def cmd_callback(self, msg: JointState):
        if len(msg.position) == len(self.joint_positions):
            self.joint_positions = list(msg.position)
        else:
            self.get_logger().warn(
                f'Expected {len(self.joint_positions)} joints, got {len(msg.position)}'
            )

    def publish_state(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        msg.position = self.joint_positions
        msg.velocity = [0.0] * 7
        msg.effort = [0.0] * 7
        self.state_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = TestArmNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
