import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
import cv2
from cv_bridge import CvBridge
import threading
import time

class WebcamPublisherNode(Node):
    def __init__(self):
        super().__init__('webcam_publisher_node')
        
        # 1. 런치파일에서 전달받을 파라미터 선언
        self.declare_parameter('device_index', 0)
        self.declare_parameter('width', 640)
        self.declare_parameter('height', 480)
        self.declare_parameter('fps', 30)

        # 파라미터 가져오기
        self.device_index = self.get_parameter('device_index').value
        width = self.get_parameter('width').value
        height = self.get_parameter('height').value
        fps = self.get_parameter('fps').value

        # 2. 카메라 초기화 (런치파일의 index 적용)
        self.cap = cv2.VideoCapture(self.device_index)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        self.cap.set(cv2.CAP_PROP_FPS, fps)

        if not self.cap.isOpened():
            self.get_logger().error(f"카메라를 열 수 없습니다. Index: {self.device_index}")
            # 시스템 종료 대신 노드만 정지시키려면 return 사용
            return 

        # 3. 데이터 공유 및 상태 변수
        self.frame = None
        self.ret = False
        self.running = True
        self.bridge = CvBridge()
        self.last_time = time.time()

        # 4. 카메라 캡처 전용 스레드 (성능 최적화 핵심)
        self.capture_thread = threading.Thread(target=self._capture_loop, daemon=True)
        self.capture_thread.start()

        # 5. Publisher 설정 (런치파일의 namespace는 Node 객체 생성 시 자동 적용됨)
        self.publisher_ = self.create_publisher(CompressedImage, 'image_raw/compressed', 10)
        
        # 타이머 주기 설정
        timer_period = 1.0 / fps
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.get_logger().info(f"[{self.get_namespace()}] 노드 가동 중... Index: {self.device_index} ({width}x{height} @ {fps}fps)")

    def _capture_loop(self):
        """별도 스레드에서 카메라 버퍼를 최신으로 유지"""
        while self.running and rclpy.ok():
            self.ret, self.frame = self.cap.read()
            if not self.ret:
                time.sleep(0.1)

    def timer_callback(self):
        """최신 프레임을 가져와 퍼블리시"""
        if self.ret and self.frame is not None:
            # OpenCV 이미지를 ROS CompressedImage 메시지로 변환
            msg = self.bridge.cv2_to_compressed_imgmsg(self.frame, "jpeg")
            self.publisher_.publish(msg)

            # 주기 모니터링 (터미널에서 확인용)
            now = time.time()
            dt = now - self.last_time
            # print(f"Publish Rate: {1/dt:.1f} Hz") # 필요 시 주석 해제
            self.last_time = now

    def stop(self):
        self.running = False
        if hasattr(self, 'capture_thread') and self.capture_thread.is_alive():
            self.capture_thread.join()
        if hasattr(self, 'cap'):
            self.cap.release()

def main(args=None):
    rclpy.init(args=args)
    node = WebcamPublisherNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()