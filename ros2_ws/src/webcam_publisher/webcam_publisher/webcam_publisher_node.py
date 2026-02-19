import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage # Changed from Image
import cv2
from cv_bridge import CvBridge

class WebcamPublisherNode(Node):
    def __init__(self):
        super().__init__('webcam_publisher_node')
        
        self.declare_parameter('device_index', 0)
        device_index = self.get_parameter('device_index').get_parameter_value().integer_value
        
        self.publisher_ = self.create_publisher(CompressedImage, 'image_raw', 10) # Changed topic name
        timer_period = 0.01  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.cap = cv2.VideoCapture(device_index)
        self.bridge = CvBridge()

        if not self.cap.isOpened():
            self.get_logger().error(f"Could not open webcam with index {device_index}.")
            exit()
        else:
            self.get_logger().info(f"Successfully opened webcam with index {device_index}.")

    def timer_callback(self):
        ret, frame = self.cap.read()
        if ret:
            # Changed from cv2_to_imgmsg to cv2_to_compressed_imgmsg
            self.publisher_.publish(self.bridge.cv2_to_compressed_imgmsg(frame, "jpeg")) 
        else:
            self.get_logger().warn("Could not read frame from webcam.")

def main(args=None):
    rclpy.init(args=args)
    webcam_publisher_node = WebcamPublisherNode()
    rclpy.spin(webcam_publisher_node)
    webcam_publisher_node.cap.release()
    webcam_publisher_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
