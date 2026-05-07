"""Virtual 6-DOF arm client.

vendor 가 흔히 제공하는 connect/enable/read/write/disconnect 패턴을 그대로 모방하되,
내부적으로는 메모리에 joint state 만 보관하는 시뮬레이션. 외부 종속성 없음.
"""
from __future__ import annotations

import time
from typing import Sequence


class VirtualArmClient:
    """가상 6 자유도 로봇팔 클라이언트.

    실제 SDK 와 동일한 시그니처(connect / enable / read_joint_positions /
    write_joint_positions / disconnect)를 제공해, EasyTrainer 의 `controller.py`
    가 이 클래스를 그대로 활용 가능하다.
    """

    JOINT_NAMES = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']

    def __init__(self, host: str = '127.0.0.1', port: int = 0):
        self._host = host
        self._port = port
        self._connected = False
        self._enabled = False
        self._positions = [0.0] * len(self.JOINT_NAMES)
        self._connect_timestamp: float | None = None

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------
    def connect(self) -> bool:
        """가짜 핸드셰이크 (50 ms 대기)."""
        time.sleep(0.05)
        self._connected = True
        self._connect_timestamp = time.time()
        return True

    def enable(self) -> bool:
        if not self._connected:
            return False
        self._enabled = True
        return True

    def disable(self) -> None:
        self._enabled = False

    def disconnect(self) -> None:
        self._enabled = False
        self._connected = False
        self._connect_timestamp = None

    @property
    def connected(self) -> bool:
        return self._connected

    @property
    def enabled(self) -> bool:
        return self._enabled

    # ------------------------------------------------------------------
    # Read
    # ------------------------------------------------------------------
    def get_joint_names(self) -> list[str]:
        return list(self.JOINT_NAMES)

    def read_joint_positions(self) -> list[float]:
        """현재 joint 위치 (라디안). connect 안 돼 있으면 RuntimeError."""
        if not self._connected:
            raise RuntimeError("Not connected. Call connect() first.")
        return list(self._positions)

    # ------------------------------------------------------------------
    # Write
    # ------------------------------------------------------------------
    def write_joint_positions(self, positions: Sequence[float]) -> None:
        """joint 명령 전송 (라디안). 즉시 내부 상태에 반영 (가상)."""
        if not self._connected:
            raise RuntimeError("Not connected.")
        if not self._enabled:
            raise RuntimeError("Not enabled. Call enable() first.")
        if len(positions) != len(self.JOINT_NAMES):
            raise ValueError(
                f"Expected {len(self.JOINT_NAMES)} joints, got {len(positions)}"
            )
        self._positions = [float(p) for p in positions]
