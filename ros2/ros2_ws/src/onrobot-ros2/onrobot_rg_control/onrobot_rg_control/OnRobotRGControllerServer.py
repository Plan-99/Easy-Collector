import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import xmlrpc.client
import time

class OnRobot2FG7XmlRpcNode(Node):
    def __init__(self):
        super().__init__('onrobot_2fg7_xmlrpc_node')

        # 1. 파라미터 선언
        self.declare_parameter('onrobot/ip', '10.0.2.20')
        self.declare_parameter('onrobot/port', 41414) # XML-RPC 표준 포트
        self.declare_parameter('onrobot/t_index', 0)   # 단일 그리퍼는 0

        self.ip = self.get_parameter('onrobot/ip').get_parameter_value().string_value
        self.port = self.get_parameter('onrobot/port').get_parameter_value().integer_value
        self.t_index = self.get_parameter('onrobot/t_index').get_parameter_value().integer_value

        # 2. XML-RPC 서버 연결 (Compute Box)
        try:
            url = f"http://{self.ip}:{self.port}/"
            self.proxy = xmlrpc.client.ServerProxy(url, allow_none=True)
            self.get_logger().info(f"✅ XML-RPC 연결 시도 중: {url}")
            
            # 장치 연결 확인 (ID 0xC0는 2FG7)
            if self.proxy.cb_is_device_connected(self.t_index, 0xC0):
                self.get_logger().info("✅ OnRobot 2FG7 그리퍼가 성공적으로 감지되었습니다.")
            else:
                self.get_logger().warn("⚠️ 그리퍼가 감지되지 않았습니다. 전원 및 ID(0xC0)를 확인하세요.")
        except Exception as e:
            self.get_logger().error(f"❌ 서버 연결 실패: {e}")

        # 3. ROS 인터페이스
        self.joint_names = ["gripper_pos", "gripper_force"]
        self.state_pub = self.create_publisher(JointState, 'joint_states', 10)
        self.cmd_sub = self.create_subscription(
            JointState,
            'command',
            self.command_callback,
            10
        )

        # 4. 상태 업데이트 타이머 (10Hz)
        self.create_timer(0.1, self.publish_status)

    def command_callback(self, msg):
        """ 목표 폭(mm)과 힘(N)을 받아 그리퍼 구동 """
        target_pos = None
        target_force = 40  # 기본 힘 40N

        for i, name in enumerate(msg.name):
            if name == "gripper_pos":
                target_pos = msg.position[i] * 100 # m -> mm
            elif name == "gripper_force":
                target_force = int(msg.position[i])

        if target_pos is not None:
            try:
                # speed는 기본 20%로 설정
                self.proxy.twofg_grip_internal(self.t_index, float(target_pos), int(target_force), 20)
            except Exception as e:
                self.get_logger().error(f"❌ 제어 명령 오류: {e}")

    def publish_status(self):
        """ 실시간 상태 읽기 및 JointState 발행 """
        try:
            # API 호출로 현재 값 읽기
            curr_width = self.proxy.twofg_get_internal_width(self.t_index)  / 100  # mm -> m
            curr_force = self.proxy.twofg_get_force(self.t_index)
            is_busy = self.proxy.twofg_get_busy(self.t_index)

            msg = JointState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.name = self.joint_names
            msg.position = [float(curr_width), float(curr_force)]
            
            self.state_pub.publish(msg)

        except Exception as e:
            # 통신 에러 로그 (너무 잦으면 주석 처리 가능)
            # self.get_logger().error(f"❌ 상태 읽기 오류: {e}")
            pass

def main(args=None):
    rclpy.init(args=args)
    node = OnRobot2FG7XmlRpcNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()