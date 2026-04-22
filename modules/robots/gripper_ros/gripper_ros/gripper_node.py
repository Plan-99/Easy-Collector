import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

class GripperNode(Node):
    def __init__(self):
        super().__init__('gripper_node')
        self.data_pub = self.create_publisher(JointState, 'gripper/states', 10)
        self.timer = self.create_timer(1.0, self.publish_gripper_data)

        self.create_subscription(JointState, 'gripper/cmd', self.gripper_cmd_cb, 10)

    def publish_gripper_data(self):
        msg = JointState()
        msg.name = ['gripper_joint']
        msg.position = [0.0]
        self.data_pub.publish(msg)

    def gripper_cmd_cb(self, msg):
        self.get_logger().info(f'Received gripper command: {msg.position}')

def main():
    rclpy.init()
    node = GripperNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
