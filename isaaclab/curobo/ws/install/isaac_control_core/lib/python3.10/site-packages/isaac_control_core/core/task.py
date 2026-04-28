"""Task 추상 클래스."""

from __future__ import annotations

from abc import ABC, abstractmethod
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from isaac_control_core.core.controller import RobotController


class BaseTask(ABC):
    """로봇이 수행할 작업의 추상 베이스 클래스.

    모든 Task는 RobotController를 받아서 execute()를 구현합니다.
    """

    def __init__(self, controller: RobotController):
        self._controller = controller

    @property
    def controller(self) -> RobotController:
        return self._controller

    @property
    def logger(self):
        return self._controller.get_logger()

    @abstractmethod
    def validate(self) -> bool:
        """Task 실행 전 필요한 조건을 검증합니다.

        Returns:
            True이면 execute() 가능, False이면 불가.
        """

    @abstractmethod
    def execute(self) -> bool:
        """Task를 실행합니다.

        Returns:
            성공 여부.
        """

    def evaluate(self) -> bool:
        """Task 실행 후 성공/실패를 판정합니다.

        서브클래스에서 오버라이드하여 구체적인 판정 로직을 구현합니다.
        기본 구현은 항상 True를 반환합니다.
        """
        return True

    def run(self) -> bool:
        """validate → execute → evaluate 전체 파이프라인 실행."""
        if not self.validate():
            self.logger.error("Task 검증 실패.")
            return False

        success = self.execute()

        result = self.evaluate()
        if result:
            self.logger.info("✅ Task 판정: 성공")
        else:
            self.logger.error("❌ Task 판정: 실패")

        return success and result
