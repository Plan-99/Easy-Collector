#!/usr/bin/env python3
"""에피소드 전체를 FollowJointTrajectory 액션으로 한 번에 전송하는 스크립트.

replay_episode.py가 토픽을 100Hz로 실시간 퍼블리시하는 반면,
이 스크립트는 전체 trajectory를 한 번의 action goal로 넘긴다.

Usage
-----
    python3 replay_episode_by_traj.py /tmp/replay_test/datasets/9/episode_000000

    python3 replay_episode_by_traj.py /tmp/replay_test/datasets/9/episode_000000 \
        --action-name /fairino16_controller_right/follow_joint_trajectory \
        --hz 100 \
        --robot-key robot_0
"""
from __future__ import annotations

import argparse
import os

import numpy as np
import pyarrow.parquet as pq

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration


def _read_json(path):
    import json
    with open(path) as f:
        return json.load(f)


def load_actions(episode_path: str, robot_key: str) -> tuple[np.ndarray, list[str]]:
    """에피소드 경로에서 action 배열과 joint 이름을 로드한다."""
    parts = episode_path.rsplit("/", 1)
    dataset_dir = parts[0]
    ep_name = parts[1] if len(parts) > 1 else "episode_000000"
    ep_index = int(ep_name.replace("episode_", ""))

    info = _read_json(os.path.join(dataset_dir, "meta", "info.json"))
    chunk = ep_index // info["chunks_size"]

    parquet_path = os.path.join(
        dataset_dir, "data", f"chunk-{chunk:03d}", f"episode_{ep_index:06d}.parquet"
    )
    if not os.path.exists(parquet_path):
        raise FileNotFoundError(f"Parquet not found: {parquet_path}")

    table = pq.read_table(parquet_path)
    df = table.to_pandas()
    action_data = np.array(df["action"].tolist(), dtype=np.float32)

    action_names = info["features"]["action"].get("names", [])
    if action_names and isinstance(action_names[0], list):
        action_names = action_names[0]

    if action_names:
        indices = [i for i, n in enumerate(action_names) if n.startswith(robot_key + "_")]
        if indices:
            action_data = action_data[:, indices]

    joint_names = [f"j{i+1}_right" for i in range(action_data.shape[1])]

    return action_data, joint_names


class ReplayTrajectoryNode(Node):
    def __init__(self, action_name: str, joint_names: list[str]):
        super().__init__("replay_episode_by_traj")
        self._action_client = ActionClient(
            self, FollowJointTrajectory, action_name,
        )
        self.joint_names = joint_names
        self.get_logger().info(f"Action: {action_name} ({len(joint_names)} joints)")

    def send_trajectory(self, actions: np.ndarray, hz: float):
        self.get_logger().info("Waiting for action server...")
        self._action_client.wait_for_server()

        period = 1.0 / hz
        trajectory = JointTrajectory()
        trajectory.joint_names = self.joint_names

        # 첫 포인트: 시작 위치로 5초간 이동
        start_pt = JointTrajectoryPoint()
        start_pt.positions = actions[0].tolist()
        start_pt.velocities = [0.0] * len(self.joint_names)
        start_pt.time_from_start = Duration(sec=5, nanosec=0)
        trajectory.points.append(start_pt)

        # 나머지 포인트: hz 간격으로 시간 배정 (5초 오프셋 이후)
        for i, action in enumerate(actions):
            pt = JointTrajectoryPoint()
            pt.positions = action.tolist()
            t = 5.0 + (i + 1) * period
            sec = int(t)
            nanosec = int((t - sec) * 1e9)
            pt.time_from_start = Duration(sec=sec, nanosec=nanosec)
            trajectory.points.append(pt)

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = trajectory

        total_time = 5.0 + len(actions) * period
        self.get_logger().info(
            f"Sending {len(trajectory.points)} points (total {total_time:.1f}s)"
        )

        future = self._action_client.send_goal_async(
            goal, feedback_callback=self._feedback_cb,
        )
        rclpy.spin_until_future_complete(self, future)

        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Goal rejected!")
            return

        self.get_logger().info("Goal accepted, executing...")
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

        result = result_future.result()
        self.get_logger().info(f"Done. error_code={result.result.error_code}")

    def _feedback_cb(self, feedback_msg):
        pass


def main():
    parser = argparse.ArgumentParser(
        description="에피소드 전체를 FollowJointTrajectory 액션으로 리플레이",
    )
    parser.add_argument("episode_path",
        help="에피소드 경로 (예: /tmp/replay_test/datasets/9/episode_000000)")
    parser.add_argument("--action-name",
        default="/fairino16_controller_right/follow_joint_trajectory")
    parser.add_argument("--hz", type=float, default=100.0)
    parser.add_argument("--robot-key", default="robot_0")
    args = parser.parse_args()

    print(f"Loading episode: {args.episode_path}")
    actions, joint_names = load_actions(args.episode_path, args.robot_key)
    print(f"Loaded {len(actions)} steps, {len(joint_names)} joints: {joint_names}")

    rclpy.init()
    node = ReplayTrajectoryNode(args.action_name, joint_names)

    try:
        node.send_trajectory(actions, args.hz)
    except KeyboardInterrupt:
        node.get_logger().info("Interrupted")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
