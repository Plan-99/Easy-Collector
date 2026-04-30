"""
Piper TrajectoryBridge 엔트리포인트.

FollowJointTrajectory action server ↔ IsaacSim /joint_command 브릿지.
"""

import rclpy

from isaac_robot_control.robots import PiperTrajectoryBridge


def main(args=None):
    rclpy.init(args=args)
    node = PiperTrajectoryBridge()
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
