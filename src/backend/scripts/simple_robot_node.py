import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState
from std_msgs.msg import Header

class SimpleRobotNode(Node):
    def __init__(self):
        super().__init__('simple_robot_node')
        
        # 로봇의 관절 이름 정의
        self.joint_names = ['joint1', 'joint2']
        
        # 로봇의 현재 상태와 목표 상태를 저장할 변수 초기화
        self.current_positions = [-0.3, 0.3]
        self.target_positions = [0.0, 0.0]
        
        # --- 퍼블리셔 (Publisher) 설정 ---
        # '/joint_states' 토픽으로 JointState 메시지를 발행
        self.state_publisher = self.create_publisher(JointState, 'joint_states', 10)
        
        # --- 구독자 (Subscriber) 설정 ---
        self.command_subscriber = self.create_subscription(
            JointState,
            'joint_cmd',
            self.command_callback,
            10)
            
        # --- 타이머 (Timer) 설정 ---
        # 0.02초(50Hz)마다 상태를 발행하기 위한 타이머
        timer_period = 0.02
        self.timer = self.create_timer(timer_period, self.publish_state_callback)
        
        self.get_logger().info(f"'{self.get_name()}' 노드가 시작되었습니다.")
        self.get_logger().info(f"명령 수신 대기: '/joint_cmd' (std_msgs/msg/JointState)")
        self.get_logger().info(f"상태 발행: '/joint_states' (sensor_msgs/msg/JointState)")

        self.cmd_publisher = self.create_publisher(JointState, 'joint_cmd', 10)

                # --- 타이머 (Timer) 설정 ---
        # 0.02초(50Hz)마다 상태를 발행하기 위한 타이머
        timer_period = 0.02
        self.timer = self.create_timer(timer_period, self.publish_cmd_callback)


    def command_callback(self, msg):
        """'joint_cmd' 토픽에서 명령을 수신할 때 호출되는 콜백 함수"""
        if len(msg.position) != len(self.joint_names):
            self.get_logger().warn(f"수신된 명령의 관절 개수({len(msg.position)})가 예상 개수({len(self.joint_names)})와 다릅니다.")
            return
        
        self.target_positions = list(msg.position)
        self.get_logger().info(f"새 목표 위치 수신: {self.target_positions}")
        

    def publish_state_callback(self):
        """타이머에 의해 주기적으로 호출되어 현재 관절 상태를 발행하는 함수"""
        
        # 🎯 현재 위치에서 목표 위치로 부드럽게 이동하는 것을 시뮬레이션
        step = 0.05  # 목표에 얼마나 빨리 다가갈지 결정하는 값
        for i in range(len(self.joint_names)):
            error = self.target_positions[i] - self.current_positions[i]
            self.current_positions[i] += error * step
            
        # JointState 메시지 생성 및 데이터 채우기
        joint_state = JointState()
        joint_state.header = Header()
        joint_state.header.stamp = self.get_clock().now().to_msg()
        joint_state.name = self.joint_names
        joint_state.position = self.current_positions
        
        # velocity, effort는 사용하지 않으므로 빈 리스트로 설정
        joint_state.velocity = []
        joint_state.effort = []
        
        # 메시지 발행
        self.state_publisher.publish(joint_state)

    def publish_cmd_callback(self):
        """타이머에 의해 주기적으로 호출되어 현재 관절 상태를 발행하는 함수"""
        
        # JointState 메시지 생성 및 데이터 채우기
        joint_cmd = JointState()
        joint_cmd.header = Header()
        joint_cmd.header.stamp = self.get_clock().now().to_msg()
        joint_cmd.name = self.joint_names
        joint_cmd.position = self.current_positions
        
        # velocity, effort는 사용하지 않으므로 빈 리스트로 설정
        joint_cmd.velocity = []
        joint_cmd.effort = []
        
        # 메시지 발행
        self.cmd_publisher.publish(joint_cmd)

def main(args=None):
    rclpy.init(args=args)
    node = SimpleRobotNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()