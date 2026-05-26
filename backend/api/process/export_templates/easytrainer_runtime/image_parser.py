"""Verbatim copy of src/backend/utils/image_parser.py from EasyTrainer.

Decodes ROS Image / CompressedImage messages into BGR numpy arrays and applies
the per-sensor crop / rotate / resize that the training pipeline used.
"""
import cv2
import numpy as np
from sensor_msgs.msg import CompressedImage


def fetch_image_with_config(image, config):
    # Mirrors backend/utils/image_parser.py.
    # Pipeline order:  sam3 (raw) → crop (raw coords) → rotate → resize
    #
    # SAM3 mask runs first on the raw frame so prompts are anchored to the
    # original camera coordinates. Lazy import — if the runtime sam3_helper
    # is missing or the SAM3 module isn't installed on the deployment host,
    # this silently no-ops.
    if config.get('sam3'):
        try:
            from .sam3_helper import apply_sam3_to_image
            image = apply_sam3_to_image(image, config.get('sensor_id'), config['sam3'])
        except ImportError:
            pass

    if 'cropped_area' in config and config['cropped_area']:
        area = config['cropped_area']
        xy_start = (area[0], area[1])
        xy_end = (area[2], area[3])
        image = image[xy_start[1]:xy_end[1], xy_start[0]:xy_end[0]]

    if 'rotate' in config:
        angle = config['rotate']
        if angle == 90:
            image = cv2.rotate(image, cv2.ROTATE_90_CLOCKWISE)
        elif angle == 180:
            image = cv2.rotate(image, cv2.ROTATE_180)
        elif angle == 270:
            image = cv2.rotate(image, cv2.ROTATE_90_COUNTERCLOCKWISE)

    if 'resize' in config and config['resize']:
        size = config['resize']
        image = cv2.resize(image, size)

    return image


def ros_image_to_numpy(image_msg):
    if isinstance(image_msg, CompressedImage):
        np_arr = np.frombuffer(image_msg.data, np.uint8)
        return cv2.imdecode(np_arr, cv2.IMREAD_COLOR)  # BGR

    encoding_to_dtype = {
        'rgb8': ('uint8', 3),
        'bgr8': ('uint8', 3),
        'mono8': ('uint8', 1),
        'mono16': ('uint16', 1),
        'rgba8': ('uint8', 4),
        'bgra8': ('uint8', 4),
    }
    if image_msg.encoding not in encoding_to_dtype:
        raise ValueError(f"Unsupported encoding: {image_msg.encoding}")
    dtype, channels = encoding_to_dtype[image_msg.encoding]
    data = np.frombuffer(image_msg.data, dtype=dtype)
    arr = data.reshape((image_msg.height, image_msg.width, channels))
    if image_msg.encoding == 'rgb8':
        arr = arr[:, :, ::-1]
    elif image_msg.encoding == 'rgba8':
        arr = arr[:, :, [2, 1, 0, 3]]
    return arr
