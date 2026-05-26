import cv2
import numpy as np

def fetch_image_with_config(image, config):
    # Pipeline order:  crop (raw coords) → rotate → resize
    # Mirrors backend/utils/image_parser.py and the export_templates copy.
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
    from sensor_msgs.msg import CompressedImage
    if isinstance(image_msg, CompressedImage):
        # 압축 이미지 처리
        np_arr = np.frombuffer(image_msg.data, np.uint8)
        image_array = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)  # BGR로 디코딩
        return image_array

    # 일반 Image 메시지 처리
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
    image_array = data.reshape((image_msg.height, image_msg.width, channels))
    
    if image_msg.encoding == 'rgb8':
        image_array = image_array[:, :, ::-1]  # RGB -> BGR
    elif image_msg.encoding == 'rgba8':
        image_array = image_array[:, :, [2, 1, 0, 3]]  # RGBA -> BGRA

    return image_array
