# Easy Trainer Modules

Easy Trainer의 모듈화된 로봇, 센서, 확장 패키지 저장소입니다.

## 구조

```
robots/           로봇 드라이버 (ROS 2 패키지)
sensors/          센서 드라이버 (ROS 2 패키지)
extensions/       확장 기능 (Python 패키지)
```

## 모듈 목록

### 로봇
| 모듈 | ID | 설명 |
|------|-----|------|
| Piper | `robot_piper` | Agilex Piper 로봇 |
| Dynamixel | `robot_dynamixel` | Robotis 서보 모터 |
| Unitree | `robot_unitree` | Unitree 로봇 |
| Jaka | `robot_jaka` | Jaka 협동로봇 |
| Fairino | `robot_fairino` | Fairino 협동로봇 |
| RBPodo | `robot_rbpodo` | Rainbow Robotics RBPodo |
| Kinova Kortex | `robot_kinova` | Kinova Kortex |
| Techman TM | `robot_techman` | Techman TM 로봇 |

### 센서
| 모듈 | ID | 설명 |
|------|-----|------|
| Webcam | `sensor_webcam` | USB 웹캠 |

### 확장
| 모듈 | ID | 설명 |
|------|-----|------|
| VR Teleop | `vr_teleop` | VR 텔레오퍼레이션 |

## 릴리즈

태그를 푸시하면 CI가 각 모듈을 `module-{id}-{version}.tar.gz`로 패키징하여 GitHub Release에 업로드합니다.

```bash
git tag v1.0.0
git push origin v1.0.0
```

## module.json 형식

```json
{
  "id": "robot_piper",
  "name": "Piper",
  "version": "1.0.0",
  "category": "robot",
  "target": "ros2",
  "install_path": "ros2_ws/src",
  "build": "colcon",
  "description": "Agilex Piper 로봇 드라이버",
  "dependencies": {
    "apt": ["ros-humble-moveit"],
    "pip": ["piper_sdk"],
    "modules": ["custom_interfaces"]
  }
}
```
