#!/usr/bin/env python3
"""에피소드 qaction을 JointTrajectory로 리플레이하는 ROS 2 서비스 노드.

리플레이 중에만 Right→Left trajectory bridge가 활성화된다.

Usage
-----
    python3 replay_episode.py /tmp/replay_test/datasets/9/episode_000000

    # 다른 터미널에서:
    ros2 service call /replay_episode/start std_srvs/srv/Trigger
    ros2 service call /replay_episode/stop  std_srvs/srv/Trigger
"""
from __future__ import annotations

import argparse
import copy
import threading
import time
import os

import numpy as np
import pyarrow.parquet as pq

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_srvs.srv import Trigger
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


# ── Right→Left bridge config ─────────────────────────────────────────────

BRIDGE_TOPICS = {
    "/admittance/planned_trajectory_right": "/admittance/planned_trajectory_left",
    # "/fairino16_controller_right/joint_trajectory": "/fairino16_controller_left/joint_trajectory",
}
DEFAULT_SIGNS = [-1.0, -1.0, -1.0, -1.0, -1.0, -1.0]
DEFAULT_OFFSETS = [0.0, -3.14159, 0.0, -3.14159, 0.0, 0.0]


class ReplayServiceNode(Node):
    def __init__(self, topic: str, joint_names: list[str], actions: np.ndarray, hz: float):
        super().__init__("replay_episode")
        self._cb_group = ReentrantCallbackGroup()

        self.pub = self.create_publisher(JointTrajectory, topic, 10)
        self.joint_names = joint_names
        self.actions = actions
        self.hz = hz

        self._worker: threading.Thread | None = None
        self._stop_event = threading.Event()
        self._done_event = threading.Event()
        self._lock = threading.Lock()
        self._step_count = 0
        self._replaying = False  # bridge gating flag

        # ── Right→Left bridge ──
        self.declare_parameter("signs", DEFAULT_SIGNS)
        self.declare_parameter("offsets", DEFAULT_OFFSETS)
        self._bridge_signs = list(self.get_parameter("signs").value)
        self._bridge_offsets = list(self.get_parameter("offsets").value)
        if len(self._bridge_signs) != len(self._bridge_offsets):
            raise ValueError(
                f"signs({len(self._bridge_signs)}) and "
                f"offsets({len(self._bridge_offsets)}) length mismatch"
            )

        self._bridge_pubs = {}
        for in_topic, out_topic in BRIDGE_TOPICS.items():
            pub = self.create_publisher(JointTrajectory, out_topic, 10)
            self._bridge_pubs[in_topic] = pub
            self.create_subscription(
                JointTrajectory, in_topic,
                lambda msg, p=pub: self._bridge_on_msg(msg, p), 10,
            )
            self.get_logger().info(f"Bridge registered: {in_topic} -> {out_topic}")

        self.get_logger().info(
            f"  bridge signs   = {self._bridge_signs}\n"
            f"  bridge offsets = {self._bridge_offsets}"
        )

        # ── Services ──
        self._start_srv = self.create_service(
            Trigger, "~/start", self._handle_start, callback_group=self._cb_group,
        )
        self._stop_srv = self.create_service(
            Trigger, "~/stop", self._handle_stop, callback_group=self._cb_group,
        )

        ns = self.get_name()
        self.get_logger().info(
            f"Replay node ready ({len(actions)} steps, {hz} Hz, topic={topic})\n"
            f"  ros2 service call /{ns}/start std_srvs/srv/Trigger\n"
            f"  ros2 service call /{ns}/stop  std_srvs/srv/Trigger"
        )

    # ── Bridge callback (only active during replay) ──────────────────────

    def _bridge_on_msg(self, msg: JointTrajectory, pub) -> None:
        if not self._replaying:
            return

        out = copy.deepcopy(msg)
        out.joint_names = [
            name.replace("_right", "_left") for name in out.joint_names
        ]

        n = len(self._bridge_signs)
        for point in out.points:
            if not point.positions:
                continue
            if len(point.positions) != n:
                self.get_logger().warn(
                    f"position length {len(point.positions)} != signs length {n}; "
                    f"passing through without transform"
                )
                continue
            point.positions = [
                self._bridge_signs[i] * float(point.positions[i]) + self._bridge_offsets[i]
                for i in range(n)
            ]

        pub.publish(out)

    # ── Model ─────────────────────────────────────────────────────────────

    def model(self, action: np.ndarray) -> np.ndarray:
        return action

    # ── Replay logic ─────────────────────────────────────────────────────

    @property
    def is_running(self) -> bool:
        return self._worker is not None and self._worker.is_alive()

    def _publish(self, action: np.ndarray, duration_sec: int = 0):
        msg = JointTrajectory()
        msg.joint_names = self.joint_names
        pt = JointTrajectoryPoint()
        pt.positions = action.tolist()
        pt.velocities = []
        pt.time_from_start = Duration(sec=duration_sec, nanosec=0)
        msg.points = [pt]
        self.pub.publish(msg)

    def _handle_start(self, request, response):
        with self._lock:
            if self.is_running:
                response.success = False
                response.message = "already running"
                return response

            self._stop_event.clear()
            self._done_event.clear()
            self._step_count = 0
            self._worker = threading.Thread(target=self._replay_loop, daemon=True)
            self._worker.start()

        self.get_logger().info(f"[start] replay started ({len(self.actions)} steps)")

        # 에피소드 완료까지 대기
        self._done_event.wait()

        response.success = True
        response.message = f"replay finished ({self._step_count}/{len(self.actions)} steps)"
        self.get_logger().info(f"[start] {response.message}")
        return response

    def _handle_stop(self, request, response):
        with self._lock:
            if not self.is_running:
                response.success = False
                response.message = "not running"
                return response
            self._stop_event.set()

        self._worker.join(timeout=5.0)
        response.success = True
        response.message = f"stopped after {self._step_count} steps"
        self.get_logger().info(f"[stop] {response.message}")
        return response

    def _replay_loop(self):
        period = 1.0 / self.hz
        total = len(self.actions)

        # 첫 프레임 위치로 천천히 이동
        self.get_logger().info("Moving to start position (duration=5s)...")
        self._publish(self.actions[0], duration_sec=2)
        # 8초 대기 (stop으로 중단 가능)
        if self._stop_event.wait(timeout=8.0):
            self.get_logger().info("[replay] stopped during move-to-start")
            return

        self.get_logger().info(f"Replaying {total} steps at {self.hz} Hz")
        self._replaying = True  # bridge ON
        next_tick = time.time()

        try:
            for i, action in enumerate(self.actions):
                if self._stop_event.is_set():
                    break

                self._publish(self.model(action))
                self._step_count = i + 1

                if i % 100 == 0:
                    self.get_logger().info(
                        f"[step {i}/{total}] {np.round(action, 4).tolist()}"
                    )

                next_tick += period
                remaining = next_tick - time.time()
                if remaining > 0.002:
                    time.sleep(remaining - 0.002)
                if remaining > 0:
                    while time.time() < next_tick:
                        pass
                else:
                    next_tick = time.time()
        finally:
            self._replaying = False  # bridge OFF
            self.get_logger().info(f"[replay] finished {self._step_count}/{total} steps")
            self._done_event.set()


def main():
    parser = argparse.ArgumentParser(
        description="에피소드 qaction 리플레이 서비스 노드 (Right→Left bridge 내장)",
    )
    parser.add_argument("episode_path",
        help="에피소드 경로 (예: /tmp/replay_test/datasets/9/episode_000000)")
    parser.add_argument("--topic", default="/fairino16_controller_right/joint_trajectory")
    parser.add_argument("--hz", type=float, default=100.0)
    parser.add_argument("--robot-key", default="robot_0")
    args = parser.parse_args()

    print(f"Loading episode: {args.episode_path}")
    actions, joint_names = load_actions(args.episode_path, args.robot_key)
    print(f"Loaded {len(actions)} steps, {len(joint_names)} joints: {joint_names}")

    rclpy.init()
    node = ReplayServiceNode(args.topic, joint_names, actions, args.hz)
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        print("\nShutting down.")
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
