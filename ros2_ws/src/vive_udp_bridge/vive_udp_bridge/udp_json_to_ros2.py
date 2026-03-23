#!/usr/bin/env python3
import socket
import json
import threading
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

class UDPJsonToROS2(Node):
    """
    Windows의 send_udp_openvr.py가 쏘는 JSON UDP 패킷을 수신해
    /vive/<key>/pose (PoseStamped)를 퍼블리시한다.
    key는 role이 있으면 role, 없으면 serial.
    """

    def __init__(self):
        super().__init__('udp_json_to_ros2')

        # 파라미터 선언
        self.declare_parameter('ip', '0.0.0.0')
        self.declare_parameter('port', 9000)
        self.declare_parameter('frame_id', 'steamvr_world')
        self.declare_parameter('prefer_role', True)  # role 키가 있으면 그걸 토픽 키로 사용

        ip = self.get_parameter('ip').get_parameter_value().string_value
        port = self.get_parameter('port').get_parameter_value().integer_value

        # UDP 소켓 준비
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((ip, port))
        self.sock.setblocking(False)
        self.get_logger().info(f"Listening UDP on {ip}:{port}")

        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        self.prefer_role = self.get_parameter('prefer_role').get_parameter_value().bool_value

        self.pubs = {}  # key -> Publisher
        self.running = True

        # 리시브 스레드
        self.thread = threading.Thread(target=self._recv_loop, daemon=True)
        self.thread.start()

        # 주기적으로 살아있음 로그(선택)
        self.create_timer(10.0, lambda: self.get_logger().debug("alive"))

    def _recv_loop(self):
        while self.running:
            try:
                data, _ = self.sock.recvfrom(4096)
            except BlockingIOError:
                time.sleep(0.001)
                continue
            except Exception as e:
                self.get_logger().error(f"recv error: {e}")
                time.sleep(0.1)
                continue

            try:
                msg = json.loads(data.decode('utf-8'))
            except Exception as e:
                self.get_logger().warn(f"bad json: {e}")
                continue

            self._publish_pose(msg)

    def _publish_pose(self, msg: dict):
        # key 결정: role 우선(설정 시), 없으면 serial
        key = None
        if self.prefer_role:
            key = msg.get('role')
        if not key:
            key = msg.get('serial')
        if not key:
            self.get_logger().warn("packet without role/serial, ignored")
            return

        # publisher 생성/재사용
        pub = self.pubs.get(key)
        if pub is None:
            topic = f"/vive/{key}/joint_states"
            pub = self.create_publisher(JointState, topic, 10)
            self.pubs[key] = pub
            self.get_logger().info(f"Publishing on {topic}")

        # JointState 구성
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.header.frame_id = msg.get('frame', self.frame_id)

        pos = msg.get('pos', [0.0, 0.0, 0.0])
        quat = msg.get('quat', [0.0, 0.0, 0.0, 1.0])

        try:
            js.name = ['pos_x', 'pos_y', 'pos_z', 'quat_x', 'quat_y', 'quat_z', 'quat_w']
            js.position = [
                float(pos[0]), float(pos[1]), float(pos[2]),
                float(quat[0]), float(quat[1]), float(quat[2]), float(quat[3])
            ]
        except Exception as e:
            self.get_logger().warn(f"bad pose fields: {e}")
            return

        pub.publish(js)

    def destroy_node(self):
        self.running = False
        time.sleep(0.05)
        try:
            self.sock.close()
        except Exception:
            pass
        return super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = UDPJsonToROS2()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass

if __name__ == "__main__":
    main()
