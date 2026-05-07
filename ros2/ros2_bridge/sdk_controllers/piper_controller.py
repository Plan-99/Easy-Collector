# -*- coding: utf-8 -*-
"""
Piper SDK Controller.
piper_sdk를 사용하여 Piper 로봇을 직접 제어한다.
ROS2 드라이버 없이 CAN 통신으로 직접 관절 제어/상태 읽기.
"""
import sys
import os
import math
import time
from typing import Optional

from .base import BaseSDKController

# piper SDK를 import path에 추가.
# 컨테이너에서는 모듈 인스톨러가 /root/robot_sdk/piper_sdk/로 풀어두고,
# 호스트 dev 트리에서는 modules/robots/piper/sdk/piper_sdk 가 단일 출처(SoT)다.
# pip install -e 로 site-packages에 link되면 path 추가 없이도 `import piper_sdk`가
# 동작하지만, link가 누락된 환경(개발 머신·새 컨테이너)을 위해 fallback 경로를 둔다.
# 모두 setup.py가 있는 디렉토리를 가리켜야 그 안의 piper_sdk/ 패키지를 import할 수 있다.
_SDK_PATHS = [
    '/root/robot_sdk/piper',                                              # 컨테이너 마운트 (모듈 install target)
    os.path.join(os.path.dirname(__file__), '..', '..', '..', 'modules', 'robots', 'piper', 'sdk'),  # dev 트리 (modules SoT)
    '/opt/easytrainer/project/ros2/robot_sdk/piper',                      # 호스트 영속 경로 (모듈 설치 결과)
]
for _p in _SDK_PATHS:
    if os.path.isdir(_p) and _p not in sys.path:
        sys.path.insert(0, _p)
        break

# radian ↔ 0.001° 변환 계수
_RAD_TO_MILLIDEG = 180.0 / math.pi * 1000.0  # ≈ 57295.78
_MILLIDEG_TO_RAD = 1.0 / _RAD_TO_MILLIDEG

# 그리퍼: radian ↔ 0.001mm 변환 (Piper 그리퍼 범위: 0~87mm, radian 범위: 0~0.087)
# 0.087 rad ≈ 87mm → factor = 1000 (mm per rad) * 1000 (0.001mm unit) = 1e6...
# 실제로 Piper 그리퍼는 0~70000 (0~70mm) in 0.001mm units
# radian 0.087 = 87mm → factor = 87/0.087 * 1000 = 1e6
_GRIPPER_RAD_TO_MICROMM = 1000.0 * 1000.0  # rad * 1000 = mm, mm * 1000 = 0.001mm
_GRIPPER_MICROMM_TO_RAD = 1.0 / _GRIPPER_RAD_TO_MICROMM

JOINT_NAMES = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "gripper"]


class PiperSDKController(BaseSDKController):
    """Piper 로봇 SDK 직접 제어 컨트롤러."""

    def __init__(self, config: dict):
        port = config.get('can_port', 'can0')
        # Linux CAN 인터페이스 이름은 underscore 없는 형태(canX)여야 한다.
        if isinstance(port, str) and port.startswith('can_'):
            port = 'can' + port[4:]
        self._can_port = port
        self._has_gripper = config.get('has_gripper', True)
        self._speed_percent = config.get('speed_percent', 100)
        self._piper = None
        self._connected = False

    def connect(self) -> bool:
        try:
            from piper_sdk import C_PiperInterface
            self._piper = C_PiperInterface(self._can_port)
            self._piper.ConnectPort()
            self._connected = True
            print(f"[PiperSDK] Connected on {self._can_port}", flush=True)
            return True
        except Exception as e:
            print(f"[PiperSDK] Connect failed: {e}", flush=True)
            return False

    def enable(self) -> bool:
        if not self._connected:
            return False
        try:
            timeout = 5.0
            start = time.time()
            enabled = False

            while not enabled and (time.time() - start) < timeout:
                self._piper.EnableArm(7)
                if self._has_gripper:
                    self._piper.GripperCtrl(0, 1000, 0x01, 0)

                # 모든 모터 enable 상태 확인
                try:
                    info = self._piper.GetArmLowSpdInfoMsgs()
                    enabled = all([
                        info.motor_1.foc_status.driver_enable_status,
                        info.motor_2.foc_status.driver_enable_status,
                        info.motor_3.foc_status.driver_enable_status,
                        info.motor_4.foc_status.driver_enable_status,
                        info.motor_5.foc_status.driver_enable_status,
                        info.motor_6.foc_status.driver_enable_status,
                    ])
                except Exception:
                    pass

                if not enabled:
                    time.sleep(0.5)

            if not enabled:
                print(f"[PiperSDK] Enable timeout after {timeout}s", flush=True)
                return False

            self._piper.MotionCtrl_2(
                ctrl_mode=0x01,   # CAN 명령 제어
                move_mode=0x01,   # MOVE J (Joint)
                move_spd_rate_ctrl=self._speed_percent,
            )
            print(f"[PiperSDK] Arm enabled, speed={self._speed_percent}%", flush=True)
            return True
        except Exception as e:
            print(f"[PiperSDK] Enable failed: {e}", flush=True)
            return False

    def write_joints(self, positions: list, names: Optional[list] = None) -> None:
        if not self._connected or self._piper is None:
            print(f"[PiperSDK] write_joints: not connected", flush=True)
            return

        # 관절 6개 (라디안 → 0.001° 정수)
        joint_count = min(len(positions), 6)
        joint_millideg = [0] * 6
        for i in range(joint_count):
            joint_millideg[i] = int(positions[i] * _RAD_TO_MILLIDEG)

        if not hasattr(self, '_write_log_count'):
            self._write_log_count = 0
        self._write_log_count += 1
        if self._write_log_count <= 5 or self._write_log_count % 250 == 0:
            print(f"[PiperSDK] write_joints millideg={joint_millideg}", flush=True)

        self._piper.JointCtrl(*joint_millideg)

        # 그리퍼 (7번째 값이 있으면)
        if self._has_gripper and len(positions) > 6:
            gripper_val = int(abs(positions[6]) * _GRIPPER_RAD_TO_MICROMM)
            gripper_val = max(0, min(gripper_val, 80000))  # 0~80mm
            self._piper.GripperCtrl(gripper_val, 1000, 0x01, 0x00)

    def read_joints(self) -> tuple:
        if not self._connected or self._piper is None:
            print(f"[PiperSDK] read_joints: not connected (connected={self._connected}, piper={self._piper is not None})", flush=True)
            return JOINT_NAMES, [0.0] * 7

        try:
            msgs = self._piper.GetArmJointMsgs()
            js = msgs.joint_state
            raw = [js.joint_1, js.joint_2, js.joint_3, js.joint_4, js.joint_5, js.joint_6]
            positions = [v * _MILLIDEG_TO_RAD for v in raw]
            # 그리퍼 상태 읽기
            if self._has_gripper:
                try:
                    gripper_msgs = self._piper.GetArmGripperMsgs()
                    gripper_pos = gripper_msgs.gripper_state.grippers_angle * _GRIPPER_MICROMM_TO_RAD
                    positions.append(abs(gripper_pos))
                except Exception:
                    positions.append(0.0)
            if not hasattr(self, '_read_log_count'):
                self._read_log_count = 0
            self._read_log_count += 1
            if self._read_log_count <= 5 or self._read_log_count % 250 == 0:
                print(f"[PiperSDK] read_joints raw={raw}, rad={[f'{p:.4f}' for p in positions]}", flush=True)
            return JOINT_NAMES[:len(positions)], positions
        except Exception as e:
            print(f"[PiperSDK] read_joints error: {e}", flush=True)
            n = 7 if self._has_gripper else 6
            return JOINT_NAMES[:n], [0.0] * n

    def disconnect(self) -> None:
        if self._piper is not None:
            try:
                self._piper.EnableArm(motor_num=7, enable_flag=0x01)  # Disable
            except Exception:
                pass
            self._piper = None
        self._connected = False
        print("[PiperSDK] Disconnected", flush=True)
