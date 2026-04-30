"""
ROS2 entry point for the MuJoCo tutorial world.

Topics:
  Pub  /tutorial/joint_states                       sensor_msgs/JointState
  Pub  /tutorial/camera/image_raw/compressed        sensor_msgs/CompressedImage
  Sub  /tutorial/joint_command                      sensor_msgs/JointState (position)

The topic prefix and joint set are parameterised so the same node can host
additional tutorial scenes in the future without code changes.
"""
from __future__ import annotations

import os
import time

import cv2
import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import CompressedImage, JointState
from std_srvs.srv import Empty

from .sim_runner import SimRunner


DEFAULT_JOINT_NAMES = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "gripper"]
DEFAULT_CAMERA_NAMES = ["top_cam", "front_cam"]


class MuJoCoWorldNode(Node):
    def __init__(self) -> None:
        super().__init__("mujoco_world_node")

        # --- Parameters -------------------------------------------------
        self.declare_parameter("scene_xml", "")
        self.declare_parameter("topic_prefix", "/tutorial")
        self.declare_parameter("joint_names", DEFAULT_JOINT_NAMES)
        self.declare_parameter("cameras", DEFAULT_CAMERA_NAMES)
        self.declare_parameter("image_width", 640)
        self.declare_parameter("image_height", 480)
        self.declare_parameter("state_publish_hz", 50.0)
        self.declare_parameter("image_publish_hz", 20.0)
        self.declare_parameter("jpeg_quality", 75)
        self.declare_parameter("realtime_factor", 1.0)
        # 기본 True — X11 auth가 잡혀 있으면 호스트에 native MuJoCo 창이 뜬다.
        # 실패해도 SimRunner._run의 try/except가 헤드리스로 폴백하므로 안전.
        self.declare_parameter("show_viewer", True)

        scene_xml = self.get_parameter("scene_xml").value
        if not scene_xml:
            scene_xml = os.path.join(
                get_package_share_directory("mujoco_world"), "assets", "scene.xml"
            )
        if not os.path.isfile(scene_xml):
            raise FileNotFoundError(f"MuJoCo scene XML not found: {scene_xml}")

        self._prefix: str = self.get_parameter("topic_prefix").value.rstrip("/")
        joint_names: list[str] = list(self.get_parameter("joint_names").value)
        camera_names: list[str] = list(self.get_parameter("cameras").value) or DEFAULT_CAMERA_NAMES
        img_w = int(self.get_parameter("image_width").value)
        img_h = int(self.get_parameter("image_height").value)
        self._jpeg_quality = int(self.get_parameter("jpeg_quality").value)
        rt_factor = float(self.get_parameter("realtime_factor").value)
        show_viewer = bool(self.get_parameter("show_viewer").value)
        image_hz = float(self.get_parameter("image_publish_hz").value)

        # --- Sim engine -------------------------------------------------
        self.get_logger().info(
            f"Loading MuJoCo scene: {scene_xml} (show_viewer={show_viewer})"
        )
        self._sim = SimRunner(
            model_path=scene_xml,
            joint_names=joint_names,
            camera_names=camera_names,
            image_width=img_w,
            image_height=img_h,
            realtime_factor=rt_factor,
            show_viewer=show_viewer,
            image_publish_hz=image_hz,
        )
        self._joint_names = self._sim.joint_names
        self._camera_names = self._sim.camera_names
        self._sim.start()

        # --- ROS pubs/subs ---------------------------------------------
        self._bridge = CvBridge()

        reliable_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
        )
        sensor_qos = QoSProfile(
            depth=2,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
        )

        self._joint_pub = self.create_publisher(
            JointState, f"{self._prefix}/joint_states", reliable_qos)
        # 카메라당 별도 publisher: /<prefix>/<cam_name>/image_raw/compressed
        self._image_pubs: list = [
            self.create_publisher(
                CompressedImage,
                f"{self._prefix}/{cam}/image_raw/compressed",
                sensor_qos,
            )
            for cam in self._camera_names
        ]

        self.create_subscription(
            JointState, f"{self._prefix}/joint_command",
            self._on_joint_command, reliable_qos)

        # 환경 초기화 서비스 — 호출 시 큐브 등 자유 객체만 keyframe 초기 상태로
        # 되돌린다 (로봇 관절은 그대로). record_episode가 한 에피소드 끝날 때
        # 호출하여 다음 시연을 동일 출발점에서 시작하기 위함.
        self.create_service(
            Empty, f"{self._prefix}/reset_world", self._on_reset_world)

        # --- Timers -----------------------------------------------------
        state_period = 1.0 / max(1.0, float(self.get_parameter("state_publish_hz").value))
        image_period = 1.0 / max(1.0, float(self.get_parameter("image_publish_hz").value))

        self.create_timer(state_period, self._publish_joint_state)
        self.create_timer(image_period, self._publish_camera_images)

        self.get_logger().info(
            f"MuJoCo world up. prefix='{self._prefix}', joints={self._joint_names}, "
            f"cameras={self._camera_names} ({img_w}x{img_h})"
        )

    # --- Callbacks -----------------------------------------------------

    def _on_joint_command(self, msg: JointState) -> None:
        if not msg.position:
            return
        # Map by name when names are provided; otherwise positional.
        positions: dict[str, float] = {}
        if msg.name:
            for n, p in zip(msg.name, msg.position):
                positions[n] = float(p)
        else:
            for n, p in zip(self._joint_names, msg.position):
                positions[n] = float(p)
        self._sim.set_targets(positions)

    def _on_reset_world(self, request, response):
        try:
            self._sim.reset_objects()
            self.get_logger().info("Tutorial world objects reset.")
        except Exception as e:
            self.get_logger().warn(f"reset_world failed: {e}")
        return response

    def _publish_joint_state(self) -> None:
        positions, velocities = self._sim.get_joint_state()
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self._joint_names
        msg.position = positions.tolist()
        msg.velocity = velocities.tolist()
        msg.effort = []
        self._joint_pub.publish(msg)

    def _publish_camera_images(self) -> None:
        # SimRunner 스레드가 비동기로 갱신한 캐시를 그대로 가져와 publish.
        # 이 콜백은 GL을 건드리지 않으므로 viewer/EGL race가 발생하지 않는다.
        stamp = self.get_clock().now().to_msg()
        for idx, pub in enumerate(self._image_pubs):
            try:
                rgb = self._sim.render_rgb(idx)
            except ValueError:
                # 첫 프레임이 아직 캐시되지 않은 startup 단계 — 다음 tick에 다시.
                continue
            except Exception as e:
                self.get_logger().warn(
                    f"Render fetch failed (cam={self._camera_names[idx]}): {e}",
                    throttle_duration_sec=5.0,
                )
                continue
            bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
            ok, buf = cv2.imencode(".jpg", bgr, [cv2.IMWRITE_JPEG_QUALITY, self._jpeg_quality])
            if not ok:
                continue
            msg = CompressedImage()
            msg.header.stamp = stamp
            msg.format = "jpeg"
            msg.data = buf.tobytes()
            pub.publish(msg)

    # --- Lifecycle -----------------------------------------------------

    def destroy_node(self) -> bool:
        try:
            self._sim.stop()
        except Exception:
            pass
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MuJoCoWorldNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
