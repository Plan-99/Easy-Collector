# -*- coding: utf-8 -*-
"""
Robotiq 2F SDK Controller.

pyRobotiqGripper (Modbus RTU over USB/RS485)를 사용해 Robotiq 2F-85 / 2F-140
그리퍼를 직접 제어한다. ROS action / hardware_interface를 거치지 않고
serial 포트로 바로 명령/상태를 주고받는다.

EasyTrainer dispatcher가 SDK_TYPE 상수를 보고 이 파일을 매칭한다.
"""
import os
import sys
import time
from typing import Optional

try:
    from base import BaseSDKController  # sdk_controllers/base.py가 sys.path에 추가됨
except ImportError:
    # dispatcher가 BaseSDKController를 모듈 글로벌에 주입하므로 import 없이도 동작.
    BaseSDKController = BaseSDKController  # type: ignore[name-defined]


SDK_TYPE = "robotiq"

# 단일 knuckle joint. URDF의 limit upper=0.8이지만 기존 module.json/spec과
# 호환을 위해 0.85까지 허용하고 controller에서 clamp한다.
JOINT_NAMES = ["robotiq_85_left_knuckle_joint"]
_RAD_MIN = 0.0
_RAD_MAX = 0.85

# pyRobotiqGripper의 position 명령은 0~255 bit (0=open, 255=closed)
_BIT_MIN = 0
_BIT_MAX = 255


def _rad_to_bit(rad: float) -> int:
    rad = max(_RAD_MIN, min(rad, _RAD_MAX))
    bit = int(round((rad - _RAD_MIN) / (_RAD_MAX - _RAD_MIN) * (_BIT_MAX - _BIT_MIN) + _BIT_MIN))
    return max(_BIT_MIN, min(bit, _BIT_MAX))


def _bit_to_rad(bit: int) -> float:
    bit = max(_BIT_MIN, min(int(bit), _BIT_MAX))
    return (bit - _BIT_MIN) / (_BIT_MAX - _BIT_MIN) * (_RAD_MAX - _RAD_MIN) + _RAD_MIN


class RobotiqSDKController(BaseSDKController):
    """Robotiq 2F gripper SDK 직접 제어 컨트롤러 (Modbus RTU)."""

    def __init__(self, config: dict):
        # custom_fields에서 들어오는 serial_port. 비어있으면 pyRobotiqGripper의
        # auto-detection에 맡긴다 ("auto" 전달 시 USB serial을 자동 탐지).
        port = config.get('serial_port') or config.get('com_port') or 'auto'
        self._com_port = port
        # Modbus device ID — 그리퍼 기본값 9. 직렬 연결된 그리퍼가 여러 대인 경우만 변경.
        self._device_id = int(config.get('device_id', 9))
        # 기본 속도/힘 (0~255). 너무 빠르거나 강하면 위험하니 보수적으로 둔다.
        self._speed = int(config.get('speed', 200))
        self._force = int(config.get('force', 100))
        # write_joints에서 호출되는 move의 wait — interpolation_node가 200Hz로
        # 부르는데 wait=True면 매 명령이 동작 완료를 기다려 누적 지연이 폭발한다.
        self._wait = False

        self._gripper = None
        self._connected = False

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------
    def connect(self) -> bool:
        try:
            from pyrobotiqgripper import RobotiqGripper
        except Exception as e:
            print(f"[RobotiqSDK] Import failed: {e}", flush=True)
            return False

        try:
            # com_port='auto'를 명시적으로 전달하면 라이브러리가 USB 직렬 포트를 탐색.
            self._gripper = RobotiqGripper(
                com_port=self._com_port,
                device_id=self._device_id,
            )
            self._gripper.connect()
            self._connected = True
            print(f"[RobotiqSDK] Connected on {self._com_port}", flush=True)
            return True
        except Exception as e:
            print(f"[RobotiqSDK] Connect failed: {e}", flush=True)
            return False

    def enable(self) -> bool:
        """그리퍼 activate. resetActivate()로 rACT 비트 rising edge 보장 — 이전 세션의
        stuck/fault 상태를 회복하기 위해 항상 reset 후 activate."""
        if not self._connected or self._gripper is None:
            return False
        try:
            self._gripper.resetActivate()
            print(f"[RobotiqSDK] Activated (speed={self._speed}, force={self._force})", flush=True)
            return True
        except Exception as e:
            print(f"[RobotiqSDK] Enable failed: {e}", flush=True)
            try:
                self._gripper.printStatus()
            except Exception:
                pass
            return False

    def disconnect(self) -> None:
        if self._gripper is not None:
            try:
                self._gripper.disconnect()
            except Exception:
                pass
            self._gripper = None
        self._connected = False
        print("[RobotiqSDK] Disconnected", flush=True)

    # ------------------------------------------------------------------
    # I/O
    # ------------------------------------------------------------------
    def write_joints(self, positions: list, names: Optional[list] = None) -> None:
        if not self._connected or self._gripper is None:
            return
        if not positions:
            return

        # 단일 knuckle joint. 첫 값만 사용.
        target_bit = _rad_to_bit(float(positions[0]))

        if not hasattr(self, '_write_log_count'):
            self._write_log_count = 0
        self._write_log_count += 1
        if self._write_log_count <= 5 or self._write_log_count % 250 == 0:
            print(f"[RobotiqSDK] write_joints rad={positions[0]:.4f} → bit={target_bit}", flush=True)

        try:
            # pyRobotiqGripper 2.x: move signature는 (position, speed, force, wait, readStatus).
            self._gripper.move(
                position=target_bit,
                speed=self._speed,
                force=self._force,
                wait=self._wait,
                readStatus=False,
            )
        except Exception as e:
            # 200Hz로 호출될 수 있으므로 throttle.
            if self._write_log_count % 100 == 0:
                print(f"[RobotiqSDK] move error: {e}", flush=True)

    def read_joints(self) -> tuple:
        if not self._connected or self._gripper is None:
            return JOINT_NAMES, [0.0]

        try:
            # pyRobotiqGripper 2.x: getPosition() — 내부 status 갱신 후 bit 반환.
            bit = self._gripper.getPosition()
            rad = _bit_to_rad(bit)
            if not hasattr(self, '_read_log_count'):
                self._read_log_count = 0
            self._read_log_count += 1
            if self._read_log_count <= 5 or self._read_log_count % 250 == 0:
                print(f"[RobotiqSDK] read_joints bit={bit} → rad={rad:.4f}", flush=True)
            return JOINT_NAMES, [rad]
        except Exception as e:
            print(f"[RobotiqSDK] read_joints error: {e}", flush=True)
            return JOINT_NAMES, [0.0]
