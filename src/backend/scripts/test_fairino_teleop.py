#!/usr/bin/env python3
"""
Fairino16 Left teleop streaming test.

- /fairino16_controller_left/state 에서 현재 관절 상태를 1회 읽는다.
- 현재 상태를 시작점으로, j2_left만 0.003 rad/step씩 증가시키며
  총 0.1 rad 이동을 30Hz (duration 0.1s) 스트리밍으로 퍼블리시한다.
- JTC의 topic 인터페이스 동작을 검증하기 위한 스탠드얼론 스크립트.

실행:
    python3 -m src.backend.scripts.test_fairino_teleop
또는:
    python3 src/backend/scripts/test_fairino_teleop.py
"""

import time

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from control_msgs.msg import JointTrajectoryControllerState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


CONTROLLER_NS = "/fairino16_controller_left"
STATE_TOPIC = f"{CONTROLLER_NS}/state"
CMD_TOPIC = f"{CONTROLLER_NS}/joint_trajectory"

TARGET_JOINT = "j2_left"
STEP_RAD = -0.003
TOTAL_RAD = -0.1
PUBLISH_HZ = 50.0
POINT_DURATION_SEC = 0
STATE_WAIT_TIMEOUT_SEC = 5.0


class FairinoTeleopTest(Node):
    def __init__(self) -> None:
        super().__init__("fairino_teleop_test")

        self._joint_names: list[str] | None = None
        self._current_positions: list[float] | None = None

        self._state_sub = self.create_subscription(
            JointTrajectoryControllerState,
            STATE_TOPIC,
            self._state_cb,
            10,
        )
        self._cmd_pub = self.create_publisher(JointTrajectory, CMD_TOPIC, 10)

    def _state_cb(self, msg: JointTrajectoryControllerState) -> None:
        if self._current_positions is not None:
            return
        self._joint_names = list(msg.joint_names)
        # Humble control_msgs/JointTrajectoryControllerState:
        #   feedback: JointTrajectoryPoint (current state)
        # Older versions:
        #   actual: JointTrajectoryPoint
        if hasattr(msg, 'feedback') and msg.feedback.positions:
            self._current_positions = list(msg.feedback.positions)
        elif hasattr(msg, 'actual') and msg.actual.positions:
            self._current_positions = list(msg.actual.positions)
        else:
            self.get_logger().warn("State message has no usable positions field")

    def wait_for_state(self, timeout_sec: float) -> bool:
        start = time.time()
        while rclpy.ok() and self._current_positions is None:
            rclpy.spin_once(self, timeout_sec=0.1)
            if time.time() - start > timeout_sec:
                return False
        return True

    def run(self) -> None:
        self.get_logger().info(f"Waiting for state on {STATE_TOPIC} ...")
        if not self.wait_for_state(STATE_WAIT_TIMEOUT_SEC):
            self.get_logger().error(f"Timed out waiting for {STATE_TOPIC}")
            return

        assert self._joint_names is not None
        assert self._current_positions is not None

        if TARGET_JOINT not in self._joint_names:
            self.get_logger().error(
                f"{TARGET_JOINT} not in controller joint_names: {self._joint_names}"
            )
            return

        target_index = self._joint_names.index(TARGET_JOINT)
        start_positions = list(self._current_positions)
        start_target = start_positions[target_index]

        num_steps = int(round(TOTAL_RAD / STEP_RAD))
        period = 1.0 / PUBLISH_HZ

        self.get_logger().info(
            f"Start {TARGET_JOINT}={start_target:.4f}, "
            f"step={STEP_RAD}, total={TOTAL_RAD}, "
            f"num_steps={num_steps}, rate={PUBLISH_HZ}Hz, duration={POINT_DURATION_SEC}s"
        )

        positions = list(start_positions)

        next_tick = time.time()
        for i in range(1, num_steps + 1):
            positions[target_index] = start_target + STEP_RAD * i

            traj = JointTrajectory()
            traj.joint_names = list(self._joint_names)

            point = JointTrajectoryPoint()
            point.positions = [float(p) for p in positions]
            point.velocities = []
            point.accelerations = []
            point.time_from_start = Duration(seconds=POINT_DURATION_SEC).to_msg()

            traj.points = [point]
            self._cmd_pub.publish(traj)

            self.get_logger().info(
                f"[{i:02d}/{num_steps}] {TARGET_JOINT} -> {positions[target_index]:.4f}"
            )

            next_tick += period
            sleep_time = next_tick - time.time()
            if sleep_time > 0:
                time.sleep(sleep_time)
            else:
                next_tick = time.time()

        self.get_logger().info("Done. Holding final target for 0.5s.")
        time.sleep(0.5)


def main() -> None:
    rclpy.init()
    node = FairinoTeleopTest()
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()