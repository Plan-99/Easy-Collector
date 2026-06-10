"""
ROS2 entry point for the MuJoCo tutorial / dual-arm test worlds.

Single-group (default, e.g. tutorial / dual_arm_test):
  Pub  /<prefix>/joint_states                    sensor_msgs/JointState
  Pub  /<prefix>/<cam>/image_raw/compressed      sensor_msgs/CompressedImage
  Sub  /<prefix>/joint_command                   sensor_msgs/JointState (position)

Multi-group (dual_arm_assembly_test): ONE physics sim, N topic namespaces.
  Set the `groups` parameter to a JSON list, each entry hosting a disjoint
  subset of joints (and any cameras) under its own topic prefix:
    [{"topic_prefix":"/da_asm_left", "joint_names":[...], "cameras":[...]},
     {"topic_prefix":"/da_asm_right","joint_names":[...], "cameras":[...]}]
  This lets two independent role='single_arm' robots be assembled while
  sharing one MuJoCo scene. When `groups` is empty the node behaves exactly
  as before (built from topic_prefix/joint_names/cameras params).
"""
from __future__ import annotations

import base64
import json
import os

import cv2
import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import CompressedImage, JointState
from std_msgs.msg import String
from std_srvs.srv import Empty, Trigger

from . import task_success
from .sim_runner import SimRunner


DEFAULT_JOINT_NAMES = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "gripper"]
# wrist_cam added for the `visual_reach` planner block: it is published live (so the
# block's settings dialog can stream it) and its depth is rendered for 3D targeting.
DEFAULT_CAMERA_NAMES = ["top_cam", "front_cam", "wrist_cam"]

# Default per-object sampling ranges for the peg-in-hole benchmark, used by the
# /tutorial/randomize service when no `randomize_ranges` param is supplied.
# Mirrors tutorial_pegtask.launch.py so the model-tester collector gets scene
# diversity. Keys are freejoint body names; ranges are metres (yaw radians).
DEFAULT_RANDOMIZE_RANGES = {
    "peg": {"x": [0.25, 0.40], "y": [-0.05, 0.10]},
    "hole_base": {"x": [0.25, 0.40], "y": [-0.15, -0.05], "yaw": [-0.3, 0.3]},
}


class _Group:
    """A single ROS topic namespace exposing a subset of the shared sim."""

    def __init__(self, prefix: str, joint_names: list, cameras: list,
                 wrist_cam: str = "", ee_site: str = ""):
        self.prefix = prefix.rstrip("/")
        self.joint_names = list(joint_names)
        self.cameras = list(cameras)
        # Optional per-group wrist RGB-D source for the `visual_reach` block:
        # `wrist_cam` is rendered with depth and paired with this arm's `ee_site`,
        # served at /<prefix>/wrist_rgbd. Empty -> this group exposes no wrist_rgbd.
        self.wrist_cam = (wrist_cam or "").strip()
        self.ee_site = (ee_site or "").strip()
        # Filled in once the SimRunner exists (indices into the sim's union order).
        self.joint_indices: list = []
        self.camera_indices: list = []
        self.joint_pub = None
        self.image_pubs: list = []


class MuJoCoWorldNode(Node):
    def __init__(self) -> None:
        super().__init__("mujoco_world_node")

        # --- Parameters -------------------------------------------------
        self.declare_parameter("scene_xml", "")
        self.declare_parameter("topic_prefix", "/tutorial")
        self.declare_parameter("joint_names", DEFAULT_JOINT_NAMES)
        self.declare_parameter("cameras", DEFAULT_CAMERA_NAMES)
        # Multi-group config (JSON string). Empty -> single group from the
        # topic_prefix/joint_names/cameras params above (backward compatible).
        self.declare_parameter("groups", "")
        self.declare_parameter("image_width", 640)
        self.declare_parameter("image_height", 480)
        self.declare_parameter("state_publish_hz", 50.0)
        self.declare_parameter("image_publish_hz", 20.0)
        self.declare_parameter("jpeg_quality", 75)
        self.declare_parameter("realtime_factor", 1.0)
        # 기본 True — X11 auth가 잡혀 있으면 호스트에 native MuJoCo 창이 뜬다.
        # 실패해도 SimRunner._run의 try/except가 헤드리스로 폴백하므로 안전.
        self.declare_parameter("show_viewer", True)
        # Per-object sampling ranges for /tutorial/randomize (JSON). Empty -> the
        # packaged DEFAULT_RANDOMIZE_RANGES (peg-in-hole) above.
        self.declare_parameter("randomize_ranges", "")

        scene_xml = self.get_parameter("scene_xml").value
        if not scene_xml:
            scene_xml = os.path.join(
                get_package_share_directory("mujoco_world"), "assets", "scene.xml"
            )
        if not os.path.isfile(scene_xml):
            raise FileNotFoundError(f"MuJoCo scene XML not found: {scene_xml}")

        # Ground-truth task identity (peg / cube) for the success service, plus
        # the randomize sampling ranges. Both feed the model-tester data pipeline.
        self._scene_id = task_success.scene_id_from_path(scene_xml)
        rr_raw = self.get_parameter("randomize_ranges").value
        try:
            self._randomize_ranges = json.loads(rr_raw) if rr_raw else dict(DEFAULT_RANDOMIZE_RANGES)
        except (json.JSONDecodeError, TypeError):
            self._randomize_ranges = dict(DEFAULT_RANDOMIZE_RANGES)

        # reset_prefix is where reset/reset_world services live; for a
        # single-group world it equals the group prefix (unchanged behavior).
        self._reset_prefix: str = self.get_parameter("topic_prefix").value.rstrip("/")
        img_w = int(self.get_parameter("image_width").value)
        img_h = int(self.get_parameter("image_height").value)
        self._jpeg_quality = int(self.get_parameter("jpeg_quality").value)
        rt_factor = float(self.get_parameter("realtime_factor").value)
        show_viewer = bool(self.get_parameter("show_viewer").value)

        # --- Resolve groups --------------------------------------------
        self._groups: list = self._build_groups()

        # Union of joints / cameras across all groups (dedup, preserve order)
        # feeds a single SimRunner — one physics sim shared by every group.
        union_joints: list = []
        union_cameras: list = []
        for g in self._groups:
            for j in g.joint_names:
                if j not in union_joints:
                    union_joints.append(j)
            for c in g.cameras:
                if c not in union_cameras:
                    union_cameras.append(c)

        # Per-arm wrist RGB-D sources for the `visual_reach` block: each group may
        # declare a (wrist_cam, ee_site) pair. SimRunner renders depth for each and
        # snapshots that arm's ee_site pose. Empty -> SimRunner's single-arm default
        # ("wrist_cam"/"ee_site") so tutorial mode is unchanged.
        wrist_specs: list = []
        for g in self._groups:
            if g.wrist_cam and g.wrist_cam not in [w[0] for w in wrist_specs]:
                wrist_specs.append((g.wrist_cam, g.ee_site))

        # --- Sim engine -------------------------------------------------
        self.get_logger().info(
            f"Loading MuJoCo scene: {scene_xml} (show_viewer={show_viewer}); "
            f"{len(self._groups)} group(s)"
        )
        self._sim = SimRunner(
            model_path=scene_xml,
            joint_names=union_joints,
            camera_names=union_cameras,
            image_width=img_w,
            image_height=img_h,
            realtime_factor=rt_factor,
            show_viewer=show_viewer,
            image_publish_hz=float(self.get_parameter("image_publish_hz").value),
            wrist_specs=wrist_specs or None,
        )
        self._sim_joint_names = self._sim.joint_names
        self._sim_camera_names = self._sim.camera_names
        self._sim.start()

        # Precompute per-group index maps into the sim's union ordering.
        for g in self._groups:
            g.joint_indices = [self._sim_joint_names.index(j) for j in g.joint_names]
            g.camera_indices = [self._sim_camera_names.index(c) for c in g.cameras]

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

        for g in self._groups:
            g.joint_pub = self.create_publisher(
                JointState, f"{g.prefix}/joint_states", reliable_qos)
            g.image_pubs = [
                self.create_publisher(
                    CompressedImage,
                    f"{g.prefix}/{cam}/image_raw/compressed",
                    sensor_qos,
                )
                for cam in g.cameras
            ]
            # Each group's command sub feeds the shared, name-keyed sim — two
            # groups writing disjoint joints never collide.
            self.create_subscription(
                JointState, f"{g.prefix}/joint_command",
                self._make_command_cb(g), reliable_qos)

        # 환경 초기화 서비스 — reset_world(Empty)는 자유 객체만, reset(Trigger)는
        # home keyframe 전체로 스냅. 그룹과 무관한 전역 동작이라 reset_prefix 하나에만.
        self.create_service(
            Empty, f"{self._reset_prefix}/reset_world", self._on_reset_world)
        self.create_service(
            Trigger, f"{self._reset_prefix}/reset", self._on_reset)
        # Model-tester data pipeline: randomize re-samples peg/hole_base poses +
        # snaps the arm home; check_success runs the ground-truth pose check.
        self.create_service(
            Trigger, f"{self._reset_prefix}/randomize", self._on_randomize)
        # Peg-only re-place (x,y within ±0.05 of home, hole fixed). The standalone
        # peg-watchdog calls this when it detects the peg has tipped over.
        self.create_service(
            Trigger, f"{self._reset_prefix}/randomize_peg", self._on_randomize_peg)
        self.create_service(
            Trigger, f"{self._reset_prefix}/check_success", self._on_check_success)
        # visual_reach: return latest wrist RGB-D + camera pose/intrinsics + ee_site
        # pose as JSON in the Trigger response message (backend reaches this via the
        # gRPC ROSProxy.CallService bridge). The reset-prefix service serves the
        # default (first) wrist cam — tutorial/single-arm behavior unchanged.
        self.create_service(
            Trigger, f"{self._reset_prefix}/wrist_rgbd",
            self._make_wrist_rgbd_cb(None))
        # Per-arm services so a dual-arm world exposes each side's wrist RGB-D under
        # its own topic prefix (e.g. /da_asm_left/wrist_rgbd). Only for groups that
        # declared a wrist_cam.
        for g in self._groups:
            if g.wrist_cam and g.prefix != self._reset_prefix:
                self.create_service(
                    Trigger, f"{g.prefix}/wrist_rgbd",
                    self._make_wrist_rgbd_cb(g.wrist_cam))

        # --- Timers -----------------------------------------------------
        state_period = 1.0 / max(1.0, float(self.get_parameter("state_publish_hz").value))
        image_period = 1.0 / max(1.0, float(self.get_parameter("image_publish_hz").value))

        self.create_timer(state_period, self._publish_joint_states)
        self.create_timer(image_period, self._publish_camera_images)
        # Movable-object pose stream consumed by tutorial_planner_node (it waits
        # for peg/hole_base poses before planning). JSON {body: [x,y,z,qw,qx,qy,qz]}.
        self._object_poses_pub = self.create_publisher(
            String, f"{self._reset_prefix}/object_poses_json", reliable_qos)
        self.create_timer(state_period, self._publish_object_poses)

        self.get_logger().info(
            "MuJoCo world up. "
            + "; ".join(
                f"group[{g.prefix}] joints={g.joint_names} cams={g.cameras}"
                for g in self._groups
            )
            + f" ({img_w}x{img_h})"
        )

    # --- Group resolution ----------------------------------------------

    def _build_groups(self) -> list:
        raw = (self.get_parameter("groups").value or "").strip()
        if raw:
            try:
                specs = json.loads(raw)
            except json.JSONDecodeError as e:
                raise ValueError(f"Invalid 'groups' JSON: {e}")
            groups = []
            for spec in specs:
                groups.append(_Group(
                    prefix=spec["topic_prefix"],
                    joint_names=spec.get("joint_names", []),
                    cameras=spec.get("cameras", []),
                    wrist_cam=spec.get("wrist_cam", ""),
                    ee_site=spec.get("ee_site", ""),
                ))
            if not groups:
                raise ValueError("'groups' parsed to an empty list")
            return groups
        # Single-group fallback (unchanged tutorial behavior).
        prefix = self.get_parameter("topic_prefix").value
        joint_names = list(self.get_parameter("joint_names").value)
        cameras = list(self.get_parameter("cameras").value) or DEFAULT_CAMERA_NAMES
        return [_Group(prefix=prefix, joint_names=joint_names, cameras=cameras)]

    # --- Callbacks -----------------------------------------------------

    def _make_command_cb(self, group: "_Group"):
        def _cb(msg: JointState) -> None:
            if not msg.position:
                return
            positions: dict = {}
            if msg.name:
                for n, p in zip(msg.name, msg.position):
                    positions[n] = float(p)
            else:
                # Positional: interpret against THIS group's joint order.
                for n, p in zip(group.joint_names, msg.position):
                    positions[n] = float(p)
            self._sim.set_targets(positions)
        return _cb

    def _on_reset(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        try:
            self._sim.reset()
            response.success = True
            response.message = "Sim reset to home keyframe."
        except Exception as e:
            response.success = False
            response.message = f"Reset failed: {e}"
            self.get_logger().error(response.message)
        return response

    def _on_randomize(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        """Re-sample peg/hole_base poses and snap the arm home. Returns the
        sampled world poses under the ``sampled`` key so the planner can use
        them as authoritative without waiting for an object-pose topic."""
        try:
            sampled = self._sim.randomize_objects(self._randomize_ranges)
            response.success = True
            response.message = json.dumps({
                "scene_id": self._scene_id,
                "sampled": sampled,
            })
        except Exception as e:
            response.success = False
            response.message = f"randomize failed: {e}"
            self.get_logger().error(response.message)
        return response

    def _on_randomize_peg(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        """Re-place only the peg (x,y within ±0.05 of home), hole stays fixed."""
        try:
            sampled = self._sim.randomize_peg(0.05)
            response.success = True
            response.message = json.dumps({"scene_id": self._scene_id, "sampled": sampled})
        except Exception as e:
            response.success = False
            response.message = f"randomize_peg failed: {e}"
            self.get_logger().error(response.message)
        return response

    def _on_check_success(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        """Ground-truth success check over the current object poses. The collector
        keeps an episode only when ``success`` is true; the evaluator counts these
        across trials. Result JSON is returned in the message."""
        try:
            poses = self._sim.get_object_poses()
            success, metrics = task_success.check_success(self._scene_id, poses)
            response.success = bool(success)
            response.message = json.dumps({
                "success": bool(success),
                "scene_id": self._scene_id,
                "metrics": metrics,
                "poses": poses,
            })
        except Exception as e:
            response.success = False
            response.message = json.dumps({"success": False, "error": str(e)})
            self.get_logger().error(f"check_success failed: {e}")
        return response

    def _make_wrist_rgbd_cb(self, cam_name):
        def _cb(request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
            return self._on_wrist_rgbd(request, response, cam_name)
        return _cb

    def _on_wrist_rgbd(self, request: Trigger.Request, response: Trigger.Response,
                       cam_name=None) -> Trigger.Response:
        try:
            data = self._sim.get_wrist_rgbd(cam_name)
            if data is None:
                response.success = False
                response.message = json.dumps({"ok": False, "error": "wrist rgbd not available"})
                return response
            rgb = cv2.cvtColor(data["rgb"], cv2.COLOR_RGB2BGR)
            ok, jpg = cv2.imencode(".jpg", rgb, [cv2.IMWRITE_JPEG_QUALITY, 90])
            depth = np.ascontiguousarray(data["depth"], dtype=np.float32)
            payload = {
                "ok": True,
                "rgb_jpeg_b64": base64.b64encode(jpg.tobytes()).decode("ascii") if ok else None,
                "depth_f32_b64": base64.b64encode(depth.tobytes()).decode("ascii"),
                "height": int(data["height"]), "width": int(data["width"]),
                "cam_pos": data["cam_pos"], "cam_mat": data["cam_mat"],
                "fovy": data["fovy"], "ee_pos": data["ee_pos"],
            }
            response.success = True
            response.message = json.dumps(payload)
        except Exception as e:
            response.success = False
            response.message = json.dumps({"ok": False, "error": str(e)})
            self.get_logger().error(f"wrist_rgbd failed: {e}")
        return response

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
            self.get_logger().info("World objects reset.")
        except Exception as e:
            self.get_logger().warn(f"reset_world failed: {e}")
        return response

    def _publish_joint_states(self) -> None:
        positions, velocities = self._sim.get_joint_state()
        stamp = self.get_clock().now().to_msg()
        for g in self._groups:
            msg = JointState()
            msg.header.stamp = stamp
            msg.name = g.joint_names
            msg.position = [float(positions[i]) for i in g.joint_indices]
            msg.velocity = [float(velocities[i]) for i in g.joint_indices]
            msg.effort = []
            g.joint_pub.publish(msg)

    def _publish_object_poses(self) -> None:
        try:
            poses = self._sim.get_object_poses()
        except Exception:
            return
        if not poses:
            return
        msg = String()
        msg.data = json.dumps(poses)
        self._object_poses_pub.publish(msg)

    def _publish_camera_images(self) -> None:
        # SimRunner 스레드가 비동기로 갱신한 캐시를 그대로 가져와 publish.
        # 이 콜백은 GL을 건드리지 않으므로 viewer/EGL race가 발생하지 않는다.
        stamp = self.get_clock().now().to_msg()
        for g in self._groups:
            for local_idx, pub in enumerate(g.image_pubs):
                cam_idx = g.camera_indices[local_idx]
                try:
                    rgb = self._sim.render_rgb(cam_idx)
                except ValueError:
                    # 첫 프레임이 아직 캐시되지 않은 startup 단계 — 다음 tick에 다시.
                    continue
                except Exception as e:
                    self.get_logger().warn(
                        f"Render fetch failed (cam={g.cameras[local_idx]}): {e}",
                        throttle_duration_sec=5.0,
                    )
                    continue
                bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
                ok, buf = cv2.imencode(".jpg", bgr, [cv2.IMWRITE_JPEG_QUALITY, self._jpeg_quality])
                if not ok:
                    continue
                m = CompressedImage()
                m.header.stamp = stamp
                m.format = "jpeg"
                m.data = buf.tobytes()
                pub.publish(m)

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
