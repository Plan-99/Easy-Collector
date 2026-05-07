"""USB 웹캠 launch — `usb_cam_node_exe` (ros-humble-usb-cam) 를 띄운다.

기존엔 자체 webcam_publisher_node.py 가 cv2.VideoCapture 로 읽었으나, OpenCV 4.10+
의 V4L2 백엔드가 일부 UVC 카메라에서 VIDIOC_G_INPUT ioctl 실패로 못 여는 회귀가
있어 (특히 컨테이너 안에서) 검은 화면만 나옴. ros-humble-usb-cam 은 libuvc 를
직접 써서 더 견고함 — 같은 deps 에 이미 있으니 그대로 활용.

사용자 입력은 정수 device_index 그대로 받고 launch 안에서 '/dev/video<N>' 로 변환.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value=''),
        DeclareLaunchArgument('device_index', default_value='0',
                              description='v4l2 device index, e.g. 0 → /dev/video0'),
        DeclareLaunchArgument('image_width', default_value='640'),
        DeclareLaunchArgument('image_height', default_value='480'),
        DeclareLaunchArgument('framerate', default_value='30.0'),
        # mjpeg2rgb: 카메라가 MJPEG 으로 송출, 노드가 RGB 로 디코드. 대부분의 USB 웹캠 호환.
        DeclareLaunchArgument('pixel_format', default_value='mjpeg2rgb'),
        Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            name='webcam_publisher_node',
            namespace=LaunchConfiguration('namespace'),
            output='screen',
            parameters=[{
                'video_device': PythonExpression(["'/dev/video' + str('", LaunchConfiguration('device_index'), "')"]),
                'image_width': LaunchConfiguration('image_width'),
                'image_height': LaunchConfiguration('image_height'),
                'framerate': LaunchConfiguration('framerate'),
                'pixel_format': LaunchConfiguration('pixel_format'),
                'camera_name': 'webcam',
                'camera_frame_id': 'webcam',
            }],
        ),
    ])
