#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from dynamixel_sdk import *
# 1. 생성한 커스텀 메시지 import
from dynamixel_ros.msg import DynamixelData

class DynamixelNode:
    def __init__(self):
        rospy.init_node('dynamixel_node', anonymous=True)
        rospy.on_shutdown(self.cleanup)

        self.ADDR_TORQUE_ENABLE = 64
        self.ADDR_PRESENT_POSITION = 132

        self.BAUDRATE = rospy.get_param('~baudrate', 57600)
        self.PORTNAME = rospy.get_param('~device_port', '/dev/ttyUSB0')
        self.PROTOCOL_VERSIONS = [2.0, 1.0] 

        self.data_pub = rospy.Publisher('/dynamixel/data', DynamixelData, queue_size=10)

        self.portHandler = PortHandler(self.PORTNAME)
        self.packetHandler = None # 프로토콜이 확정된 후 설정
        self.dxl_ids = [] # 스캔된 ID들을 저장할 리스트

        # --- 설정 및 스캔 실행 ---
        if self.setup_port():
            self.scan()

    def setup_port(self):
        if not self.portHandler.openPort():
            rospy.logerr(f"Failed to open port {self.PORTNAME}")
            return False
        if not self.portHandler.setBaudRate(self.BAUDRATE):
            rospy.logerr(f"Failed to set baudrate to {self.BAUDRATE}")
            return False
        rospy.loginfo(f"Port {self.PORTNAME} opened successfully at {self.BAUDRATE}bps.")
        return True

    def scan(self):
        rospy.loginfo("Scanning for Dynamixels...")
        for protocol in self.PROTOCOL_VERSIONS:
            self.packetHandler = PacketHandler(protocol)
            dxl_ids, dxl_comm_result = self.packetHandler.broadcastPing(self.portHandler)
            
            if dxl_comm_result == COMM_SUCCESS and dxl_ids:
                self.dxl_ids = list(dxl_ids.keys())
                self.dxl_ids.sort()
                rospy.loginfo(f"Detected Protocol {protocol} with IDs: {dxl_ids}")
                break # ID를 찾으면 해당 프로토콜로 확정
            else:
                rospy.loginfo(f"No devices found on Protocol {protocol}.")
        
        if not self.dxl_ids:
            rospy.logerr("No Dynamixels found. Shutting down.")
            rospy.signal_shutdown("No Dynamixels found.")
            return

    def run(self):
        if not self.dxl_ids: # 스캔된 ID가 없으면 실행하지 않음
            return
            
        rate = rospy.Rate(10) # 10Hz
        rospy.loginfo("Start reading and publishing data for all dynamixels.")

        while not rospy.is_shutdown():
            # 3. 메시지 인스턴스 생성
            data_msg = DynamixelData()
            
            # 읽어온 데이터를 임시로 저장할 리스트
            id_list = []
            value_list = []

            for dxl_id in self.dxl_ids:
                position, result, error = self.packetHandler.read4ByteTxRx(self.portHandler, dxl_id, self.ADDR_PRESENT_POSITION)
                if result == COMM_SUCCESS and error == 0:
                    if position > 2147483647:
                        position -= 4294967296  # 2**32 를 빼서 원래 음수 값을 구함
                    id_list.append(dxl_id)
                    value_list.append(position)
            
            # 4. 리스트를 메시지에 채워넣고 발행
            if id_list:
                data_msg.ids = id_list
                data_msg.values = value_list
                self.data_pub.publish(data_msg)

            rate.sleep()

    def cleanup(self):
        rospy.loginfo("Shutting down node...")
        if self.portHandler.is_open:
            self.portHandler.closePort()
            rospy.loginfo("Port closed.")

if __name__ == '__main__':
    try:
        node = DynamixelNode()
        node.run()
    except rospy.ROSInterruptException:
        pass