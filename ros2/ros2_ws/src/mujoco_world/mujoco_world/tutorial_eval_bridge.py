"""HTTP bridge that lets the backend container drive the tutorial sim.

The ros2 container has rclpy but no torch / lerobot. The backend container
has torch / lerobot / CUDA but no rclpy. The model-test evaluator runs in
the backend, but it has to *observe* the MuJoCo sim and *publish actions*
to it — both of which require ROS.

Rather than wire up another gRPC service, this bridge exposes a tiny
JSON-over-HTTP API on port 7799 (configurable). Endpoints:

  POST /reset
      Calls /tutorial/randomize, waits for one fresh observation, returns
      the initial obs JSON.
  GET  /observe
      Returns the most recent observation:
          {seq, state, joint_order, stamp,
           images: {cam_name: jpeg_b64, ...},
           <cam_name>_jpeg_b64: ...}        # legacy alias, present for
                                              # backwards-compatibility
  POST /step    body={"action": [7 floats]}
      Publishes the action to /tutorial/joint_command, waits for the next
      tick, returns the new observation.
  GET  /check
      Calls /tutorial/check_success and returns the parsed JSON payload
      (scene_id, success, metrics, poses).
  POST /shutdown
      Cleanly tears down.

Action / state order is the canonical 7-vector: joint1..joint6 + gripper.

Usage (inside ros2 container)::

    ros2 run mujoco_world tutorial_eval_bridge --port 7799 \
        --cameras wrist_cam,wrist_cam_down
"""
from __future__ import annotations

import argparse
import base64
import json
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Optional

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage, JointState
from std_srvs.srv import Trigger


ARM_JOINTS = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
TOOL_JOINT = "gripper"
ALL_JOINTS = ARM_JOINTS + [TOOL_JOINT]


# Map MJCF camera names → legacy short keys used by older evaluator clients.
# Allows clients that still ask for top_jpeg_b64 / front_jpeg_b64 /
# wrist_jpeg_b64 to keep working when those cameras are present.
_LEGACY_SHORT_BY_CAM = {
    "top_cam": "top",
    "front_cam": "front",
    "wrist_cam": "wrist",
}


class EvalBridgeNode(Node):
    def __init__(self, prefix: str = "/tutorial",
                 cameras: Optional[list[str]] = None):
        super().__init__("tutorial_eval_bridge")
        self._prefix = prefix.rstrip("/")
        self._camera_names = list(cameras) if cameras else ["wrist_cam", "wrist_cam_down"]

        self._lock = threading.Lock()
        self._latest_state: Optional[JointState] = None
        self._latest_images: dict[str, Optional[CompressedImage]] = {
            name: None for name in self._camera_names
        }
        # Monotonically increasing counter so /step can wait for "next" frame
        # rather than re-returning the one we acted on.
        self._state_seq = 0

        cb = ReentrantCallbackGroup()
        ctrl_qos = QoSProfile(
            depth=5,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
        )
        img_qos = QoSProfile(
            depth=5,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
        )

        self.create_subscription(
            JointState, f"{self._prefix}/joint_states",
            self._on_state, ctrl_qos, callback_group=cb)
        for cam in self._camera_names:
            self.create_subscription(
                CompressedImage,
                f"{self._prefix}/{cam}/image_raw/compressed",
                self._make_on_image(cam),
                img_qos,
                callback_group=cb,
            )

        # Publisher for actions. Match the planner's publisher QoS so the
        # MuJoCo node's subscription accepts our messages.
        self._cmd_pub = self.create_publisher(
            JointState, f"{self._prefix}/joint_command", ctrl_qos)

        self._cli_random = self.create_client(
            Trigger, f"{self._prefix}/randomize", callback_group=cb)
        self._cli_check = self.create_client(
            Trigger, f"{self._prefix}/check_success", callback_group=cb)

    # ------------------------------------------------------------------
    # Subscriptions
    # ------------------------------------------------------------------
    def _on_state(self, msg: JointState) -> None:
        with self._lock:
            self._latest_state = msg
            self._state_seq += 1

    def _make_on_image(self, cam_name: str):
        def _cb(msg: CompressedImage) -> None:
            with self._lock:
                self._latest_images[cam_name] = msg
        return _cb

    # ------------------------------------------------------------------
    # Public API consumed by the HTTP handler
    # ------------------------------------------------------------------
    def wait_for_streams(self, timeout: float = 10.0) -> bool:
        deadline = time.time() + timeout
        while time.time() < deadline:
            with self._lock:
                state_ready = self._latest_state is not None
                cams_ready = all(
                    self._latest_images.get(c) is not None
                    for c in self._camera_names
                )
                if state_ready and cams_ready:
                    return True
            time.sleep(0.05)
        return False

    def snapshot(self) -> dict:
        """Return the current observation as a JSON-ready dict."""
        with self._lock:
            s = self._latest_state
            imgs = {n: self._latest_images.get(n) for n in self._camera_names}
            seq = self._state_seq
        if s is None or any(v is None for v in imgs.values()):
            raise RuntimeError("observation not ready")
        images_b64 = {
            n: base64.b64encode(bytes(m.data)).decode("ascii")
            for n, m in imgs.items()
        }
        out: dict = {
            "seq": seq,
            "state": _extract_positions(s),
            "joint_order": ALL_JOINTS,
            "images": images_b64,
            "camera_names": list(self._camera_names),
            "stamp": time.time(),
        }
        # Legacy aliases: top_jpeg_b64 / front_jpeg_b64 / wrist_jpeg_b64.
        # Older evaluator builds still look for these, so keep them populated
        # whenever the corresponding camera is in the active set.
        for cam, b64 in images_b64.items():
            short = _LEGACY_SHORT_BY_CAM.get(cam)
            if short:
                out[f"{short}_jpeg_b64"] = b64
        return out

    def publish_action(self, action: list[float]) -> None:
        if len(action) != len(ALL_JOINTS):
            raise ValueError(
                f"action length {len(action)} != {len(ALL_JOINTS)}")
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(ALL_JOINTS)
        msg.position = [float(x) for x in action]
        self._cmd_pub.publish(msg)

    def wait_for_next_state(self, prev_seq: int, timeout: float = 1.0) -> None:
        """Block until self._state_seq advances past prev_seq."""
        deadline = time.time() + timeout
        while time.time() < deadline:
            with self._lock:
                if self._state_seq > prev_seq:
                    return
            time.sleep(0.005)

    def call_trigger(self, client, timeout: float = 30.0) -> Trigger.Response:
        if not client.wait_for_service(timeout_sec=5.0):
            raise RuntimeError(f"service {client.srv_name} not available")
        future = client.call_async(Trigger.Request())
        deadline = time.time() + timeout
        while not future.done():
            if time.time() > deadline:
                raise TimeoutError(f"timeout on {client.srv_name}")
            time.sleep(0.02)
        return future.result()

    def do_reset(self) -> dict:
        # Mark seq so we can wait for a *new* state after the randomization
        # settles (otherwise we'd return the pre-randomize observation).
        with self._lock:
            seq_before = self._state_seq
        rnd = self.call_trigger(self._cli_random)
        if not rnd.success:
            raise RuntimeError(f"randomize_failed: {rnd.message}")
        # Let the freejoints settle and a fresh frame come in.
        time.sleep(0.4)
        self.wait_for_next_state(seq_before, timeout=2.0)
        return self.snapshot()

    def do_check(self) -> dict:
        chk = self.call_trigger(self._cli_check)
        try:
            payload = json.loads(chk.message)
        except Exception:
            payload = {"raw": chk.message}
        payload["service_success"] = bool(chk.success)
        return payload


def _extract_positions(msg: JointState) -> list[float]:
    name_to_pos = {n: float(p) for n, p in zip(msg.name, msg.position)}
    return [name_to_pos.get(name, 0.0) for name in ALL_JOINTS]


# ---------------------------------------------------------------------------
# HTTP server
# ---------------------------------------------------------------------------
def _make_handler(node: EvalBridgeNode, shutdown_event: threading.Event):
    class Handler(BaseHTTPRequestHandler):
        # Silence the default access log — ros2 logger already captures
        # everything we care about.
        def log_message(self, format, *args):  # noqa: A002
            return

        # ----- helpers --------------------------------------------------
        def _send_json(self, code: int, payload: dict) -> None:
            body = json.dumps(payload).encode("utf-8")
            self.send_response(code)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def _read_body(self) -> dict:
            length = int(self.headers.get("Content-Length", "0"))
            if length <= 0:
                return {}
            raw = self.rfile.read(length)
            if not raw:
                return {}
            return json.loads(raw.decode("utf-8"))

        # ----- routes ---------------------------------------------------
        def do_GET(self):  # noqa: N802
            try:
                if self.path == "/observe":
                    return self._send_json(200, node.snapshot())
                if self.path == "/check":
                    return self._send_json(200, node.do_check())
                if self.path == "/health":
                    return self._send_json(
                        200,
                        {"ok": True, "cameras": list(node._camera_names)},
                    )
                self._send_json(404, {"error": f"no route {self.path}"})
            except Exception as e:
                self._send_json(500, {"error": repr(e)})

        def do_POST(self):  # noqa: N802
            try:
                if self.path == "/reset":
                    return self._send_json(200, node.do_reset())
                if self.path == "/step":
                    body = self._read_body()
                    action = body.get("action")
                    if not isinstance(action, list):
                        return self._send_json(
                            400, {"error": "missing/invalid 'action'"})
                    with node._lock:
                        seq_before = node._state_seq
                    node.publish_action(action)
                    node.wait_for_next_state(seq_before, timeout=0.5)
                    return self._send_json(200, node.snapshot())
                if self.path == "/shutdown":
                    self._send_json(200, {"ok": True})
                    shutdown_event.set()
                    return
                self._send_json(404, {"error": f"no route {self.path}"})
            except Exception as e:
                self._send_json(500, {"error": repr(e)})

    return Handler


def main(args=None):
    parser = argparse.ArgumentParser(
        description="HTTP bridge for model_tester inference loop.")
    parser.add_argument("--port", type=int, default=7799)
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--topic-prefix", default="/tutorial")
    parser.add_argument("--cameras", default="wrist_cam,wrist_cam_down",
                        help="Comma-separated MJCF camera names to subscribe. "
                             "Must match the cameras the launch file exposes.")
    cli_args, ros_args = parser.parse_known_args(args)
    cameras = [c.strip() for c in cli_args.cameras.split(",") if c.strip()]
    if not cameras:
        raise SystemExit("--cameras parsed to empty list")

    rclpy.init(args=ros_args)
    node = EvalBridgeNode(prefix=cli_args.topic_prefix, cameras=cameras)
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    if not node.wait_for_streams(timeout=10.0):
        node.get_logger().error(
            "topics not ready — joint_states or cameras missing")
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()
        return 2

    shutdown_event = threading.Event()
    handler_cls = _make_handler(node, shutdown_event)
    server = ThreadingHTTPServer((cli_args.host, cli_args.port), handler_cls)
    server.daemon_threads = True

    node.get_logger().info(
        f"eval bridge ready on http://{cli_args.host}:{cli_args.port} "
        f"(cameras={cameras})")

    server_thread = threading.Thread(target=server.serve_forever, daemon=True)
    server_thread.start()

    try:
        while not shutdown_event.is_set():
            time.sleep(0.2)
    except KeyboardInterrupt:
        pass
    finally:
        server.shutdown()
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
