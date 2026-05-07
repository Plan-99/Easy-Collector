"""test_arm SDK Controller.

EasyTrainer 의 `ros2/ros2_bridge/sdk_controllers/__init__.py` 디스패처가 이 파일을 자동으로
임포트해서 `BaseSDKController` 의 서브클래스를 찾는다. SDK_TYPE 상수 또는 폴더명이
`module.json` 의 `driver.sdk_type` 과 일치하면 매칭된다.

이 파일은 사용자가 모듈 위자드에서 SDK 모드로 모듈을 만들 때 자동 생성되는 템플릿과
같은 형태의 *완성된* 예시다. test_arm_sdk.VirtualArmClient 를 5 개 메서드에 위임.
"""
try:
    from base import BaseSDKController  # noqa: F401  (sdk_controllers/base.py)
except ImportError:
    BaseSDKController = BaseSDKController  # type: ignore[name-defined]

from test_arm_sdk import VirtualArmClient


# module.json driver.sdk_type 과 일치해야 디스패처가 찾는다
SDK_TYPE = "test_arm"


class TestArmSDKController(BaseSDKController):
    """가상 6 자유도 로봇팔 컨트롤러 (SDK 직접 제어 데모)."""

    def __init__(self, config: dict):
        # config 는 module.json + 사용자가 추가한 custom_fields(예: ip_address) 를 합친 dict
        host = config.get("ip_address", "127.0.0.1")
        port = int(config.get("port", 0))
        self._client = VirtualArmClient(host=host, port=port)

    def connect(self) -> bool:
        return self._client.connect()

    def enable(self) -> bool:
        return self._client.enable()

    def write_joints(self, positions, names=None) -> None:
        # interpolation_node 가 6 개 (또는 그 이상) 의 라디안을 넘김.
        # 우리 가상 로봇은 정확히 6 개만 받으므로 앞 6개만 쓴다.
        self._client.write_joint_positions(list(positions[:6]))

    def read_joints(self) -> tuple:
        names = self._client.get_joint_names()
        positions = self._client.read_joint_positions()
        return names, positions

    def disconnect(self) -> None:
        self._client.disconnect()
