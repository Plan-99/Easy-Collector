"""Minimal environment wrapper for closed-loop ROS inference.

Subscribes to the per-sensor image topic listed in ``export_meta.json`` and
exposes ``get_observation()`` returning the same dict shape as
EasyTrainer's full ``Env``::

    {
        "robot_states": {agent_id: {"qpos": [...]}, ...},
        "images":       {f"sensor_{id}": np.ndarray (H, W, 3, BGR), ...},
    }

Image messages are kept as raw BGR numpy arrays here — the model-side image
preprocessing (crop / rotate / resize / normalize) is applied later in
``ros_inference.py`` before calling the policy.
"""
from __future__ import annotations

import threading
import time
from typing import List

from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, CompressedImage

from .image_parser import ros_image_to_numpy


class SimpleEnv:
    def __init__(self, node: Node, agents: List, sensors: List[dict]):
        self.node = node
        self.agents = agents
        self.sensors = sensors

        self._img_lock = threading.Lock()
        self._latest_images = {}
        self._sensor_subs = []

        for sensor in sensors:
            sid = sensor['id']
            msg_type = CompressedImage
            if sensor.get('read_topic_msg') == 'sensor_msgs/Image':
                msg_type = Image

            # Most camera drivers publish with BEST_EFFORT reliability + VOLATILE
            # durability (sensor_data profile). Subscribing with the default
            # (RELIABLE) silently drops every message — the publisher and
            # subscriber are QoS-incompatible. Use sensor_data so we match.
            sub = node.create_subscription(
                msg_type,
                sensor['read_topic'],
                lambda msg, _sid=sid: self._image_cb(msg, _sid),
                qos_profile_sensor_data,
            )
            self._sensor_subs.append(sub)
            print(f"[SimpleEnv] sensor_{sid}: {sensor['read_topic']} "
                  f"({sensor.get('read_topic_msg', 'CompressedImage')})")

    def _image_cb(self, msg, sensor_id) -> None:
        try:
            arr = ros_image_to_numpy(msg)
        except Exception as e:
            print(f"[SimpleEnv] sensor_{sensor_id} decode error: {e}")
            return
        with self._img_lock:
            self._latest_images[sensor_id] = arr

    # ────────────────────────────────────────────────────────────────────
    def get_observation(self) -> dict:
        """Single-step observation snapshot. Blocks until every sensor has at
        least one frame and every agent has at least one joint state.
        """
        # Robot states
        robot_states = {}
        for agent in self.agents:
            qpos = agent.get_joint_states()
            robot_states[agent.id] = {"qpos": qpos}

        # Images
        image_dict = {}
        deadline = time.time() + 5.0
        for sensor in self.sensors:
            sid = sensor['id']
            while True:
                with self._img_lock:
                    img = self._latest_images.get(sid)
                if img is not None:
                    break
                if time.time() > deadline:
                    raise RuntimeError(
                        f"[SimpleEnv] sensor_{sid} no image received on "
                        f"{sensor['read_topic']} after 5 seconds. "
                        f"Is the camera driver publishing?"
                    )
                time.sleep(0.02)
            image_dict[f"sensor_{sid}"] = img

        return {
            "robot_states": robot_states,
            "images": image_dict,
        }
