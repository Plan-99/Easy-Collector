# test_arm_sdk

EasyTrainer 모듈 위자드의 SDK 모드 검증용 가짜 vendor SDK.
실제 하드웨어 없이 `connect / enable / read / write / disconnect` 흐름을 시뮬레이션한다.

## 사용 예

```python
from test_arm_sdk import VirtualArmClient

client = VirtualArmClient()
client.connect()
client.enable()

# 현재 6 자유도 joint 상태 읽기 (라디안)
positions = client.read_joint_positions()

# 명령 송신 (즉시 반영)
client.write_joint_positions([0.1, -0.2, 0.3, 0.0, 0.5, -0.1])

client.disconnect()
```

EasyTrainer 와 통합되는 진입점은 같은 폴더의 [controller.py](./controller.py) — `BaseSDKController`
를 상속해서 `connect/enable/write_joints/read_joints/disconnect` 다섯 메서드를 이 클라이언트에
위임한다.
