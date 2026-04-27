# MuJoCo Tutorial World

EasyTrainer 튜토리얼 모드를 위한 MuJoCo 시뮬레이션 환경입니다.

## 구성
- 6DOF 매니퓰레이터 + 1DOF 그리퍼 (총 7 joint, Piper와 동일 구성)
- 테이블 + 잡기 연습용 블럭 1개
- 외부 RGB 카메라 (작업 공간 위쪽)

## 발행/구독 토픽
| 방향 | 토픽 | 메시지 |
|------|------|--------|
| Pub  | `/tutorial/joint_states` | `sensor_msgs/JointState` |
| Pub  | `/tutorial/camera/image_raw/compressed` | `sensor_msgs/CompressedImage` |
| Sub  | `/tutorial/joint_command` | `sensor_msgs/JointState` (position 사용) |

## 실행
```bash
ros2 launch mujoco_world mujoco_world.launch.py
```

## 확장
다른 sim 환경(IsaacSim, PyBullet 등)을 추가하려면 `modules/sim/<name>/` 패턴으로 새 모듈을 만들고
동일한 토픽 인터페이스를 따르면 됩니다.
