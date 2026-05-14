# -*- coding: utf-8 -*-
"""
Base SDK Controller.
모든 로봇 SDK 컨트롤러의 추상 베이스 클래스.
새로운 로봇 SDK를 추가할 때 이 클래스를 상속하여 구현한다.
"""
from abc import ABC, abstractmethod
from typing import Optional


class BaseSDKController(ABC):
    """로봇 SDK 제어 인터페이스."""

    @abstractmethod
    def connect(self) -> bool:
        """SDK 연결 및 초기화. 성공 시 True 반환."""
        ...

    @abstractmethod
    def enable(self) -> bool:
        """모터 활성화. 성공 시 True 반환."""
        ...

    @abstractmethod
    def write_joints(self, positions: list, names: Optional[list] = None) -> None:
        """
        관절 위치 명령 전송.
        Args:
            positions: 라디안 단위 관절 각도 리스트 (그리퍼 포함 가능)
            names: 관절 이름 리스트 (선택)
        """
        ...

    @abstractmethod
    def read_joints(self) -> tuple:
        """
        관절 상태 읽기.
        Returns:
            (names: list[str], positions: list[float]) - 라디안 단위
        """
        ...

    def read_joints_extended(self) -> tuple:
        """
        관절 상태 읽기 (확장: 속도/토크 포함). 기본 구현은 read_joints() 결과에
        빈 velocity/effort 를 붙여 반환. 지원하는 SDK 는 override.

        Returns:
            (names: list[str], positions: list[float],
             velocities: list[float] (rad/s, 미지원이면 []),
             efforts: list[float] (Nm 등, 미지원이면 []))
        """
        names, positions = self.read_joints()
        return names, positions, [], []

    @abstractmethod
    def disconnect(self) -> None:
        """SDK 연결 해제 및 리소스 정리."""
        ...
