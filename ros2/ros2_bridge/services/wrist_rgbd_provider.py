"""Real-camera wrist RGB-D provider.

Mirrors the sim's ``/tutorial/wrist_rgbd`` Trigger so the planner ``visual_reach``
(Wrist View Reach) block works on a real RealSense too. For a sensor with
``use_depth`` on, the bridge advertises ``/ec_sensor_<id>/wrist_rgbd`` (std_srvs/
Trigger). On call it returns the latest color + aligned depth + intrinsics as a
JSON payload identical in shape to the sim one, EXCEPT:

  - intrinsics are real ``fx/fy/cx/cy`` (from camera_info) instead of a sim ``fovy``
    → the planner handler picks the OPTICAL (+z forward) back-projection convention.
  - NO ``cam_pos``/``cam_mat``: a real camera has no calibrated world pose, so the
    handler composes the camera pose from the live EE FK + the user's manual mount
    (cam_offset + pitch/yaw/roll) — the calibration-free path.

Subscribes lazily (before the realsense node publishes is fine) with BEST_EFFORT
sensor QoS to match realsense2_camera image publishers.
"""
import base64
import json

import numpy as np
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import CompressedImage, Image, CameraInfo
from std_srvs.srv import Trigger


def _sensor_qos(depth: int = 5) -> QoSProfile:
    return QoSProfile(depth=depth,
                      reliability=ReliabilityPolicy.BEST_EFFORT,
                      history=HistoryPolicy.KEEP_LAST)


class WristRgbdProvider:
    """Latches a RealSense sensor's color/aligned-depth/camera_info and serves a
    Trigger that returns them as the wrist-rgbd payload. Call ``destroy()`` to
    tear down subscriptions + service when the sensor stops."""

    def __init__(self, node, sensor_id: int):
        self.node = node
        self.sensor_id = int(sensor_id)
        ns = f'/ec_sensor_{self.sensor_id}/camera'
        self._color = None   # latest CompressedImage (JPEG bytes)
        self._depth = None    # latest Image (16UC1, mm)
        self._info = None     # latest CameraInfo

        qos = _sensor_qos()
        self._sub_color = node.create_subscription(
            CompressedImage, f'{ns}/color/image_raw/compressed', self._on_color, qos)
        self._sub_depth = node.create_subscription(
            Image, f'{ns}/aligned_depth_to_color/image_raw', self._on_depth, qos)
        # camera_info is RELIABLE/transient-ish; keep BEST_EFFORT depth-1 is fine.
        self._sub_info = node.create_subscription(
            CameraInfo, f'{ns}/color/camera_info', self._on_info, _sensor_qos(1))
        self._srv = node.create_service(
            Trigger, f'/ec_sensor_{self.sensor_id}/wrist_rgbd', self._on_trigger)
        node.get_logger().info(
            f'[WristRgbd] provider up for ec_sensor_{self.sensor_id} '
            f'(srv=/ec_sensor_{self.sensor_id}/wrist_rgbd)')

    def _on_color(self, msg):
        self._color = msg

    def _on_depth(self, msg):
        self._depth = msg

    def _on_info(self, msg):
        self._info = msg

    def _on_trigger(self, request, response):
        try:
            if self._color is None or self._depth is None:
                response.success = False
                response.message = json.dumps({'ok': False, 'error': 'no color/depth frame yet'})
                return response
            d = self._depth
            h, w = int(d.height), int(d.width)
            dt = '>u2' if getattr(d, 'is_bigendian', 0) else '<u2'
            depth_mm = np.frombuffer(d.data, dt).reshape((h, w))
            depth_m = (depth_mm.astype(np.float32)) / 1000.0  # mm → metres (sim parity)

            payload = {
                'ok': True,
                'rgb_jpeg_b64': base64.b64encode(bytes(self._color.data)).decode('ascii'),
                'depth_f32_b64': base64.b64encode(np.ascontiguousarray(depth_m).tobytes()).decode('ascii'),
                'height': h, 'width': w,
            }
            if self._info is not None and len(self._info.k) >= 6:
                k = self._info.k
                payload.update({'fx': float(k[0]), 'fy': float(k[4]),
                                'cx': float(k[2]), 'cy': float(k[5])})
            else:
                # No camera_info yet → fall back to a pinhole guess (caller may also
                # supply fovy). Better to report missing so the handler errors clearly.
                payload['error_intrinsics'] = 'camera_info not received'
            response.success = True
            response.message = json.dumps(payload)
        except Exception as e:
            response.success = False
            response.message = json.dumps({'ok': False, 'error': str(e)})
            self.node.get_logger().error(f'[WristRgbd] trigger failed: {e}')
        return response

    def destroy(self):
        try:
            self.node.destroy_service(self._srv)
            self.node.destroy_subscription(self._sub_color)
            self.node.destroy_subscription(self._sub_depth)
            self.node.destroy_subscription(self._sub_info)
        except Exception:
            pass
