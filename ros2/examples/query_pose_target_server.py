#!/usr/bin/env python3
"""
EasyTrainer Planner - Query Pose 블록용 외부 예시 노드.

Planner 의 "Query Pose" 블록은 실행 시점에 여기서 정의한 ROS 서비스를
std_srvs/srv/Trigger 타입으로 호출한다. 외부 노드는 목표 좌표를 계산해서
response.message 에 JSON 문자열로 담아 응답하면 된다.

워크스페이스는 "어셈블리" — 로봇들의 조합(예: 팔 + 툴) — 이므로 목표값은
단순 리스트가 아니라 **어셈블리 슬롯 이름으로 키잉한 딕셔너리**여야 한다.
슬롯: left_arm / right_arm / left_tool / right_tool / mobile_base
응답에 포함한 슬롯만 움직이고, 빠진 슬롯은 그대로 둔다.

- 관절 모드(joint_position):
    {"positions": {
        "left_arm":  [j1, j2, ..., gripper],   # 그 로봇의 전체 관절 수와 일치해야 함
        "left_tool": [g1]
    }}
- EE 모드(end_effector_position):
    {"poses": {
        "left_arm": {"position": [x, y, z],
                     "orientation": [rx, ry, rz],   # Euler 라디안
                     "gripper": <value>}            # 선택
    }}

이 예시는 두 서비스를 동시에 띄운다:
    /my_target        -> 관절 좌표 응답
    /my_ee_target     -> EE 좌표 응답

Planner 블록의 service_name 에 위 이름 중 하나를 입력하면 된다.

실행 방법 (EasyTrainer 와 같은 ROS_DOMAIN_ID 환경이면 어디서든 가능):
    source /opt/ros/humble/setup.bash
    python3 query_pose_target_server.py

호출 테스트:
    ros2 service call /my_target std_srvs/srv/Trigger
"""
import json

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger


class QueryPoseTargetServer(Node):
    def __init__(self):
        super().__init__('query_pose_target_server')
        self.create_service(Trigger, '/my_target', self.handle_joint)
        self.create_service(Trigger, '/my_ee_target', self.handle_ee)
        self.get_logger().info('Query Pose 예시 서비스 준비 완료: /my_target, /my_ee_target')

    def handle_joint(self, request, response):
        # 실제로는 여기서 비전/계획 등으로 목표 관절값을 계산한다.
        # 키는 어셈블리 슬롯 이름, 값은 그 로봇의 전체 관절 벡터(그리퍼 포함).
        # 관절 수가 안 맞으면 로봇이 움직이지 않는다.
        payload = {
            'positions': {
                'left_arm': [0.0, 0.5, -0.5, 0.0, 0.5, 0.0, 0.3],
                # 'left_tool': [0.4],   # 어셈블리에 툴 슬롯이 있으면 추가
            }
        }
        response.success = True
        response.message = json.dumps(payload)
        self.get_logger().info(f'/my_target -> {response.message}')
        return response

    def handle_ee(self, request, response):
        # 실제로는 여기서 목표 엔드이펙터 포즈를 계산한다.
        payload = {
            'poses': {
                'left_arm': {
                    'position': [0.30, 0.00, 0.25],
                    'orientation': [0.0, 1.57, 0.0],
                    'gripper': 0.5,
                },
            }
        }
        response.success = True
        response.message = json.dumps(payload)
        self.get_logger().info(f'/my_ee_target -> {response.message}')
        return response


def main():
    rclpy.init()
    node = QueryPoseTargetServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
