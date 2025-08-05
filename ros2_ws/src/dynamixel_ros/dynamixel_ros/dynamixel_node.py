#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from dynamixel_sdk import *
# 1. 생성한 커스텀 메시지 import (경로는 동일)
from dynamixel_ros.msg import DynamixelData

class DynamixelNode(Node):
    def __init__(self):
        # 노드 초기화
        super().__init__('dynamixel_node')

        # 주소 정의 (기존과 동일)
        self.ADDR_TORQUE_ENABLE = 64
        self.ADDR_PRESENT_POSITION = 132
        
        # 파라미터 선언 및 할당
        self.declare_parameter('baudrate', 57600)
        self.declare_parameter('device_port', '/dev/ttyUSB0')
        self.BAUDRATE = self.get_parameter('baudrate').get_parameter_value().integer_value
        self.PORTNAME = self.get_parameter('device_port').get_parameter_value().string_value
        
        self.PROTOCOL_VERSIONS = [2.0, 1.0]

        # 퍼블리셔 생성
        self.data_pub = self.create_publisher(DynamixelData, '/dynamixel/data', 10)

        self.portHandler = PortHandler(self.PORTNAME)
        self.packetHandler = None
        self.dxl_ids = []

        # --- 설정 및 스캔 실행 ---
        if self.setup_port():
            self.scan()

        # 스캔된 ID가 있을 경우에만 타이머 생성
        if self.dxl_ids:
            timer_period = 0.1  # 10Hz
            self.timer = self.create_timer(timer_period, self.timer_callback)
            self.get_logger().info("Start reading and publishing data for all dynamixels.")

    def setup_port(self):
        if not self.portHandler.openPort():
            self.get_logger().error(f"Failed to open port {self.PORTNAME}")
            return False
        if not self.portHandler.setBaudRate(self.BAUDRATE):
            self.get_logger().error(f"Failed to set baudrate to {self.BAUDRATE}")
            return False
        self.get_logger().info(f"Port {self.PORTNAME} opened successfully at {self.BAUDRATE}bps.")
        return True

    def scan(self):
        self.get_logger().info("Scanning for Dynamixels...")
        for protocol in self.PROTOCOL_VERSIONS:
            self.packetHandler = PacketHandler(protocol)
            # broadcastPing의 반환 값이 ROS 1 SDK 예제와 다를 수 있어 일반적인 ping으로 변경
            for dxl_id in range(254):
                model_number, dxl_comm_result, dxl_error = self.packetHandler.ping(self.portHandler, dxl_id)
                if dxl_comm_result == COMM_SUCCESS and dxl_error == 0:
                    self.dxl_ids.append(dxl_id)
            
            if self.dxl_ids:
                self.dxl_ids.sort()
                self.get_logger().info(f"Detected Protocol {protocol} with IDs: {self.dxl_ids}")
                break  # ID를 찾으면 해당 프로토콜로 확정
            else:
                self.get_logger().info(f"No devices found on Protocol {protocol}.")
        
        if not self.dxl_ids:
            self.get_logger().error("No Dynamixels found. Shutting down.")
            # 노드를 안전하게 종료하기 위해 rclpy.shutdown() 호출
            rclpy.shutdown()

    def timer_callback(self):
        # 3. 메시지 인스턴스 생성
        data_msg = DynamixelData()
        
        id_list = []
        value_list = []

        for dxl_id in self.dxl_ids:
            position, result, error = self.packetHandler.read4ByteTxRx(self.portHandler, dxl_id, self.ADDR_PRESENT_POSITION)
            if result == COMM_SUCCESS and error == 0:
                # 32비트 부호 있는 정수로 변환
                if position > 2147483647:
                    position -= 4294967296
                id_list.append(dxl_id)
                value_list.append(position)
        
        # 4. 리스트를 메시지에 채워넣고 발행
        if id_list:
            data_msg.ids = id_list
            data_msg.values = value_list
            self.data_pub.publish(data_msg)

    def cleanup(self):
        self.get_logger().info("Shutting down node...")
        if self.portHandler.is_open:
            self.portHandler.closePort()
            self.get_logger().info("Port closed.")

def main(args=None):
    rclpy.init(args=args)
    node = DynamixelNode()
    try:
        # spin()은 노드가 종료될 때까지 콜백 함수(timer_callback)를 계속 실행합니다.
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # 노드 종료 시 자원 정리
        node.cleanup()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()