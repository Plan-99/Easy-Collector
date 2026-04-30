#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import numpy as np
from std_srvs.srv import Trigger

class UnitreeNode(Node):
    """
    Unitree H1 팔과 Inspire 핸드 하드웨어를 ROS 2 토픽에 연결하는 브릿지 노드.
    [수정됨] 팔과 핸드의 토픽을 분리하여 발행 및 구독합니다.
    """
    
    def __init__(self):
        super().__init__('unitree_node')

        self.declare_parameter('type', 'h1_2')
        self.declare_parameter('pub_frequency', 50.0)
        
        self.robot_type = self.get_parameter('type').get_parameter_value().string_value
        self.publish_frequency = self.get_parameter('pub_frequency').get_parameter_value().double_value
        
        try:
            if self.robot_type == 'h1_2':
                from .robot_arm import H1_2_ArmController
                self.arm_ctrl = H1_2_ArmController()
            else:
                raise ValueError(f"지원되지 않는 로봇 타입: {self.robot_type}")
        except Exception as e:
            self.get_logger().fatal(f"로봇 팔 컨트롤러 초기화 실패: {e}")
            raise e


        # --- 관절 이름 정의 (원본과 동일) ---
        # 1. H1 팔 관절 (14개)
        self.arm_joint_names = [
            'l1', 'l2', 'l3', 'l4', 'l5', 'l6', 'l7',
            'r1', 'r2', 'r3', 'r4', 'r5', 'r6', 'r7'
        ]
        
        # (all_joint_names 리스트는 더 이상 필요하지 않습니다)
        self.num_arm_joints = len(self.arm_joint_names)       # 14
        

        # --- ROS 2 퍼블리셔 및 서브스크라이버 ( # <-- 변경된 부분 ) ---
        
        # 1. Publishers (상태 발행)
        self.arm_joint_state_pub = self.create_publisher(  # <-- 1. 팔 상태
            JointState,
            'unitree_arm/joint_states', # <-- 변경된 토픽
            10
        )
        
        # 발행 주기 (예: 50Hz)

        self.publish_timer = self.create_timer(
            1.0 / self.publish_frequency,
            self.publish_joint_states # <-- 변경된 콜백 함수 이름
        )
        
        # 2. Subscribers (명령 수신)
        self.arm_cmd_sub = self.create_subscription( # <-- 1. 팔 명령
            JointState,
            'unitree_arm/cmd',         # <-- 변경된 토픽
            self.arm_command_callback, # <-- 변경된 콜백
            10
        )

        # 3. Service (Go Home) ( <--- [추가됨] )
        self.go_home_service = self.create_service(
            Trigger,
            'unitree_arm/go_home',  # 서비스 이름
            self.go_home_callback   # 서비스 콜백 함수
        )
        self.get_logger().info("Service 'unitree_arm/go_home'가 준비되었습니다.")
        
        self.get_logger().info("ROS 2 (Arm/Hand) Publisher/Subscriber 생성 완료. 노드 실행 중.")

    def publish_joint_states(self): # <-- ( # <-- 변경 )
        """[수정됨] H1 팔과 Inspire 핸드의 현재 상태를 *별도 토픽*으로 발행합니다."""
        
        current_time_stamp = self.get_clock().now().to_msg()
        
        # 1. H1 팔 상태 읽기 및 발행
        try:
            current_arm_q = self.arm_ctrl.get_current_dual_arm_q()
            if current_arm_q is None or len(current_arm_q) != self.num_h1_joints:
                self.get_logger().warn("H1 팔 상태를 읽을 수 없거나 크기가 맞지 않습니다.")
            else:
                arm_msg = JointState()
                arm_msg.header.stamp = current_time_stamp
                arm_msg.name = self.h1_joint_names
                arm_msg.position = list(current_arm_q.astype(float))
                self.arm_joint_state_pub.publish(arm_msg)
                
        except Exception as e:
            self.get_logger().error(f"H1 팔 상태 읽기/발행 오류: {e}")
            

    def arm_command_callback(self, msg: JointState): # <-- ( # <-- 새로 추가 )
        """'unitree_arm/cmd' 토픽을 구독하여 H1 팔에 qpos 명령을 전송합니다."""
        
        # 메시지가 14개의 관절 위치를 포함하는지 확인
        if len(msg.position) != self.num_h1_joints:
            self.get_logger().error(
                f"[ArmCmd] 수신 오류: {len(msg.position)}개의 위치. {self.num_h1_joints}개가 필요합니다."
            )
            return
        
        if all(p < 0 for p in msg.position):
            self.arm_ctrl.ctrl_dual_arm_go_home()
            return

        arm_q_target = np.array(msg.position, dtype=float)
        arm_tauff_target = np.zeros(self.num_h1_joints)
        
        try:
            self.arm_ctrl.ctrl_dual_arm(arm_q_target, arm_tauff_target)
            self.get_logger().debug(f"[ArmCmd] 수신: {arm_q_target.round(2)}")
        except Exception as e:
            self.get_logger().error(f"H1 팔 제어 오류: {e}")


    # ( <--- [추가됨] 서비스 콜백 함수 )
    def go_home_callback(self, request, response):
        """'unitree_arm/go_home' 서비스 요청을 처리합니다."""
        
        self.get_logger().info("'go_home' 서비스 요청 수신... 팔을 홈 포지션으로 이동합니다.")
        
        try:
            # 하드웨어 컨트롤러의 홈 포지션 함수 호출
            self.arm_ctrl.ctrl_dual_arm_go_home()
            
            # 서비스 응답 설정 (성공)
            response.success = True
            response.message = "Arm command 'go_home' successfully sent."
            self.get_logger().info(response.message)
            
        except Exception as e:
            # 서비스 응답 설정 (실패)
            response.success = False
            response.message = f"Failed to send 'go_home' command: {e}"
            self.get_logger().error(response.message)
            
        return response
    

    def on_shutdown(self):
        """노드 종료 시 로봇을 안전한 상태(home)로 만듭니다. (원본과 동일)"""
        self.get_logger().info("노드 종료... 로봇 팔을 홈 포지션으로 이동합니다.")
        try:
            self.arm_ctrl.ctrl_dual_arm_go_home()
        except Exception as e:
            self.get_logger().error(f"종료 중 로봇 제어 오류: {e}")
        self.get_logger().info("종료 완료.")


def main(args=None):
    rclpy.init(args=args)
    
    node = None
    try:
        node = UnitreeNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("키보드 인터럽트 (Ctrl+C) 수신.")
    except Exception as e:
        if node:
            node.get_logger().fatal(f"노드 실행 중 치명적 오류 발생: {e}")
        else:
            print(f"노드 초기화 중 치명적 오류 발생: {e}")
    finally:
        if node:
            node.on_shutdown()
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()