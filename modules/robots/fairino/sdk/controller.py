# -*- coding: utf-8 -*-
"""
Fairino SDK Controller.

fairino-python-sdk (XML-RPC + UDP)를 사용하여 Fairino 협동로봇을 직접 제어한다.
ROS2 fairino_hardware 노드 없이 Python SDK로 바로 servo 명령을 송신.

Interpolation node가 control_mode='sdk'로 띄워졌을 때:
  - 200Hz로 들어오는 보간된 라디안 명령을 도(degree)로 변환해 ServoJ 호출
  - 50Hz로 GetActualJointPosRadian → ROS2 토픽 'interpolated_joint_cmd' 퍼블리시

ServoJ는 RPC가 아닌 UDP transparent path (cmdType=1)를 사용해 200Hz 송신에서도
XML-RPC overhead를 피한다. 첫 명령 전 ServoMoveStart, disconnect 시 ServoMoveEnd.

EasyTrainer의 sdk_controllers dispatcher가 SDK_TYPE 상수를 보고 이 파일을 매칭한다.
"""
import math
import os
import sys
import time
from typing import Optional

try:
    from base import BaseSDKController  # sdk_controllers/base.py가 sys.path에 추가됨
except ImportError:
    # dispatcher가 BaseSDKController를 모듈 글로벌에 주입하므로 import 없이도 동작.
    BaseSDKController = BaseSDKController  # type: ignore[name-defined]


SDK_TYPE = "fairino"

# fairino python sdk(`from fairino import Robot`)는 controller.py 옆 `linux/` 하위에 있다.
# - dev 트리:     modules/robots/fairino/sdk/{controller.py, linux/fairino/...}
# - 설치 후:      ros2/robot_sdk/robot_fairino/{controller.py, linux/fairino/...}
_SDK_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'linux')
if os.path.isdir(_SDK_PATH) and _SDK_PATH not in sys.path:
    sys.path.insert(0, _SDK_PATH)

# 라디안 ↔ 도 변환
_RAD_TO_DEG = 180.0 / math.pi
_DEG_TO_RAD = math.pi / 180.0

# Fairino fr5 6축
JOINT_NAMES = ["j1", "j2", "j3", "j4", "j5", "j6"]


class FairinoSDKController(BaseSDKController):
    """Fairino 로봇 SDK 직접 제어 컨트롤러 (XML-RPC + UDP)."""

    def __init__(self, config: dict):
        # 보간 노드는 'sdk_can_port' 등 piper용 키만 자동 전달하므로,
        # ip_address는 별도로 설정에서 꺼내거나 fallback 사용.
        self._ip = config.get('ip_address') or config.get('ip') or '192.168.58.2'
        # 명령 전송 cmdType: 0=XML-RPC, 1=UDP (200Hz용 권장)
        self._cmd_type = int(config.get('cmd_type', 1))
        # ServoJ 명령 주기 (s) — interpolation_node publish_rate(200Hz=5ms)와 매칭해야
        # 큐 적체/명령 잔류로 인한 stop 후 잔여 동작을 막을 수 있다.
        self._cmd_t = float(config.get('cmd_t', 0.005))
        # 추가 축 (Fairino fr5는 0)
        self._exaxis_pos = [0.0, 0.0, 0.0, 0.0]

        self._robot = None
        self._connected = False
        self._servo_started = False

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------
    def connect(self) -> bool:
        try:
            from fairino import Robot
        except Exception as e:
            print(f"[FairinoSDK] Import failed: {e}", flush=True)
            return False

        try:
            self._robot = Robot.RPC(self._ip)
            # is_connect 클래스 변수가 True면 RPC + CNDE 모두 OK
            connected = bool(getattr(Robot.RPC, 'is_connect', False))
            if not connected:
                print(f"[FairinoSDK] RPC connect failed (ip={self._ip})", flush=True)
                return False
            self._connected = True
            print(f"[FairinoSDK] Connected to {self._ip}", flush=True)
            return True
        except Exception as e:
            print(f"[FairinoSDK] Connect exception: {e}", flush=True)
            return False

    def enable(self) -> bool:
        """자동모드 진입 + 서보 enable + 서보 모드 시작."""
        if not self._connected or self._robot is None:
            return False
        try:
            # Mode: 0=자동, 1=수동
            err = self._robot.Mode(0)
            if err != 0:
                print(f"[FairinoSDK] Mode(0) failed: {err}", flush=True)
            time.sleep(0.5)

            # 서보 enable (1=enable, 0=disable)
            err = self._robot.RobotEnable(1)
            if err != 0:
                print(f"[FairinoSDK] RobotEnable(1) failed: {err}", flush=True)
                return False
            time.sleep(0.5)

            # ServoMoveStart — ServoJ를 호출하기 전에 한 번 진입해야 함
            err = self._robot.ServoMoveStart()
            if err != 0:
                print(f"[FairinoSDK] ServoMoveStart failed: {err}", flush=True)
                return False
            self._servo_started = True

            print(f"[FairinoSDK] Enabled (servo mode active)", flush=True)
            return True
        except Exception as e:
            print(f"[FairinoSDK] Enable exception: {e}", flush=True)
            return False

    def disconnect(self) -> None:
        if self._robot is not None:
            try:
                if self._servo_started:
                    self._robot.ServoMoveEnd()
                    self._servo_started = False
            except Exception:
                pass
            try:
                self._robot.RobotEnable(0)
            except Exception:
                pass
            try:
                # CloseRPC가 있으면 호출
                if hasattr(self._robot, 'CloseRPC'):
                    self._robot.CloseRPC()
            except Exception:
                pass
            self._robot = None
        self._connected = False
        print("[FairinoSDK] Disconnected", flush=True)

    # ------------------------------------------------------------------
    # I/O
    # ------------------------------------------------------------------
    def write_joints(self, positions: list, names: Optional[list] = None) -> None:
        if not self._connected or self._robot is None:
            return

        # 6축만 사용 (그리퍼 미지원)
        joint_count = min(len(positions), 6)
        joint_deg = [0.0] * 6
        for i in range(joint_count):
            joint_deg[i] = positions[i] * _RAD_TO_DEG

        if not hasattr(self, '_write_log_count'):
            self._write_log_count = 0
        self._write_log_count += 1
        if self._write_log_count <= 5 or self._write_log_count % 250 == 0:
            print(f"[FairinoSDK] write_joints deg={[f'{v:.3f}' for v in joint_deg]}", flush=True)

        try:
            # ServoJ는 도(degree) 단위
            self._robot.ServoJ(
                joint_deg, self._exaxis_pos,
                acc=0.0, vel=0.0, cmdT=self._cmd_t,
                cmdType=self._cmd_type,
            )
        except Exception as e:
            # 200Hz로 호출되므로 throttle
            if self._write_log_count % 100 == 0:
                print(f"[FairinoSDK] ServoJ error: {e}", flush=True)

    def read_joints(self) -> tuple:
        if not self._connected or self._robot is None:
            return JOINT_NAMES, [0.0] * 6

        try:
            ret = self._robot.GetActualJointPosRadian(flag=1)
            # 성공 시 (0, [j1..j6]), 실패 시 (errcode, None) 또는 (errcode,)
            if isinstance(ret, tuple) and len(ret) >= 2 and ret[0] == 0 and ret[1] is not None:
                positions = list(ret[1])
            else:
                return JOINT_NAMES, [0.0] * 6

            if not hasattr(self, '_read_log_count'):
                self._read_log_count = 0
            self._read_log_count += 1
            if self._read_log_count <= 5 or self._read_log_count % 250 == 0:
                print(f"[FairinoSDK] read_joints rad={[f'{p:.4f}' for p in positions]}", flush=True)
            return JOINT_NAMES[:len(positions)], positions
        except Exception as e:
            print(f"[FairinoSDK] read_joints error: {e}", flush=True)
            return JOINT_NAMES, [0.0] * 6
