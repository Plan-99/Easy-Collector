# IsaacLab — Piper Robot Manipulation Framework

NVIDIA Isaac Sim 4.5 기반의 로봇 매니퓰레이션 프레임워크.
**Piper 6-DOF 로봇**으로 Pick-and-Place, Motor-in-Box 등의 태스크를 수행하며,
모션 플래너로 **cuRobo** 또는 **MoveIt2**를 선택할 수 있다.

## Architecture

```
IsaacLab/
├── ros2_env/                  # Isaac Sim + ROS2 시뮬레이션 환경
│   ├── base_env.py            #   BaseEnv (씬 구성, ROS2 퍼블리싱, 도메인 랜덤화)
│   ├── objects.py             #   오브젝트 관리 (랜덤 배치, 마커 퍼블리싱)
│   └── envs/                  #   환경 구현체
│       ├── pi0_env.py         #     Piper V100 + 큐브 + AlienDoll
│       ├── piper_env_custom.py#     Piper + 스펀지/박스/마우스 등
│       └── ...
│
├── isaac_control_core/        # 플래너 독립적 공통 추상화 (symlink으로 공유)
│   ├── core/                  #   MotionController ABC, RobotConfig, BaseTask
│   ├── robots/                #   PiperConfig, PiperV100Config, FrankaConfig
│   ├── tasks/                 #   PickAndPlaceTask, MotorInBoxTask, StackCubeTask
│   ├── utils/skills.py        #   RobotSkills (pick, place, place_on_object)
│   └── services/              #   서비스 러너 + GripperScaleBridge
│
├── curobo/                    # cuRobo 모션 플래너
│   ├── Dockerfile             #   CUDA 12.4 + PyTorch + ROS2 + cuRobo
│   ├── docker_start.sh        #   컨테이너 실행 스크립트
│   └── ws/src/curobo_control/ #   CuroboController 패키지
│
├── moveit/                    # MoveIt2 모션 플래너
│   ├── Dockerfile
│   └── ws/src/                #   MoveItController + launch 파일
│
├── ros_pkgs/
│   └── piper_description/     # Piper URDF + 메쉬 (symlink으로 공유)
│
└── docker/                    # Isaac Sim Docker 설정
    ├── docker-compose.yaml
    ├── .env.base
    └── .env.ros2
```

## Prerequisites

- **OS**: Ubuntu 22.04
- **GPU**: NVIDIA GPU (RTX 3070 이상 권장, 8GB VRAM 최소)
- **Docker**: Docker + NVIDIA Container Toolkit
- **NGC**: NVIDIA NGC 계정 (Isaac Sim 이미지용)

## Quick Start

### 1. 저장소 클론

```bash
git clone <repository-url> IsaacLab
cd IsaacLab
```

### 2. Isaac Sim Docker 빌드 + 실행

```bash
# NGC 로그인 (최초 1회 — Isaac Sim 베이스 이미지 pull에 필요)
# https://ngc.nvidia.com/setup 에서 API Key 발급
docker login nvcr.io
# Username: $oauthtoken
# Password: <NGC API Key>

# Isaac Sim 이미지 빌드
cd docker
docker compose --profile base --env-file .env.base build
docker compose --profile ros2 --env-file .env.base --env-file .env.ros2 build
cd ..

# X11 포워딩 허용
xhost +local:docker

# Isaac Sim 컨테이너 실행
bash docker_start.sh
```

**컨테이너 안에서 초기 설정** (최초 1회):
```bash
# isaac_lab_dev 컨테이너 안에서
ln -s /isaac-sim /workspace/isaaclab/_isaac_sim
./isaaclab.sh --install
```

### 3. Isaac Sim 환경 실행

```bash
# isaac_lab_dev 컨테이너 안에서

# GUI 모드
/workspace/isaaclab/_isaac_sim/python.sh ros2_env/envs/pi0_env.py

# Headless 모드
/workspace/isaaclab/_isaac_sim/python.sh ros2_env/envs/pi0_env.py --headless
```

환경이 실행되면 아래 ROS2 토픽이 자동 퍼블리싱됩니다:
- `/simulation/joint_states` — 관절 상태
- `/simulation/tf` — 로봇 + 오브젝트 TF
- `/simulation/joint_command` — 관절 명령 (구독)
- `/simulation/object_markers/{name}` — 오브젝트 위치/방향 마커
- `/simulation/{cam_name}_rgb` — 카메라 RGB 이미지
- `/simulation/reset_env` — 환경 리셋 서비스

### 4. cuRobo Docker 빌드 + 실행

```bash
# 호스트에서 (새 터미널)
cd IsaacLab/curobo

# cuRobo 이미지 빌드 (최초 1회, 10~15분 소요)
docker build -t curobo-ros2:latest .

# cuRobo 컨테이너 실행
bash docker_start.sh
```

### 5. cuRobo 서비스 노드 실행

```bash
# curobo_dev 컨테이너 안에서
cd /root/ws
colcon build
source install/setup.bash

# Pick-and-Place 서비스 실행
ros2 run curobo_control pick_and_place_service

# 또는 Motor-in-Box 서비스 실행
ros2 run curobo_control motor_in_box_service
```

### 6. 서비스 호출 (태스크 실행)

```bash
# 아무 터미널에서 (ROS2 네트워크 공유)
ros2 service call /pick_and_place std_srvs/srv/Trigger
ros2 service call /motor_in_box std_srvs/srv/Trigger
```

### 7. (선택) MoveIt2 Docker 빌드 + 실행

```bash
cd IsaacLab/moveit

# MoveIt 이미지 빌드
docker build -t moveit2-isaac:latest .

# MoveIt 컨테이너 실행
bash docker_start.sh

# 컨테이너 안에서
cd /root/ws
colcon build
source install/setup.bash

# MoveIt + RViz launch
ros2 launch isaac_robot_control isaac_moveit_piper.launch.py

# 서비스 실행
ros2 run isaac_robot_control pick_and_place_service_piper
```

## ROS2 파라미터

```bash
# pick/place 대상 오브젝트 변경
ros2 run curobo_control pick_and_place_service \
  --ros-args -p pick_object:=RedCube -p place_target:=WhitePlate

# grasp 방향 설정
ros2 run curobo_control pick_and_place_service \
  --ros-args -p grasp_yaw:=auto       # 오브젝트 yaw 자동 정렬
  # -p grasp_yaw:=vertical            # 세로 (90도)
  # -p grasp_yaw:=horizontal          # 가로 (0도)
  # -p grasp_yaw:=45                  # 특정 각도
```

## 데이터 수집용 토픽

녹화/재생 호환을 위해 gripper 값이 **실제 로봇 스케일**로 변환된 토픽이 별도 제공됩니다:

| 토픽 | 스케일 | 용도 |
|---|---|---|
| `/simulation/joint_states` | sim (0~0.035) | IsaacSim 내부 |
| `/simulation/joint_command` | sim (0~0.035) | IsaacSim 명령 |
| `/joint_states` | **real (0~0.085)** | 데이터 수집/재생 |
| `/joint_command` | **real (0~0.085)** | 데이터 수집/재생 |

## 도메인 랜덤화

`setup_scene()`에서 설정:

```python
# 카메라 위치/방향 랜덤화 (기준점 ± delta)
self.add_camera(position=[0.4, 1.2, 1.6], orientation=[38, 1, 173],
                position_delta=[0.08, 0.05, 0.05],
                orientation_delta=[2, 2, 5],
                focal_length=13.0)  # 광각

# 조명 랜덤화
self.set_lighting_randomization(
    intensity_range=[800, 2500],
    color_temp_range=[3500, 6500],
    additional_lights=2)

# 배경(바닥) 색상 랜덤화 (흑백)
self.set_background_randomization(brightness_range=[0.0, 0.3])

# 오브젝트 위치 + 방향 랜덤화
self.add_object("RedCube", size=0.05, shape="cube",
                position_range=[[0.2, -0.18, 0.052], [0.40, 0, 0.052]],
                orientation_range=[[0, 0, -45], [0, 0, 45]])
```

## 프로젝트 구조 상세

자세한 구현 설명은 각 모듈의 CLAUDE.md 참고:
- `ros2_env/CLAUDE.md` — 시뮬레이션 환경 프레임워크
- `isaac_control_core/CLAUDE.md` — 모션 컨트롤러 추상화 + 태스크
- `curobo/CLAUDE.md` — cuRobo 컨트롤러 구현 세부사항

## Swap 설정 (VRAM 부족 시)

```bash
sudo swapoff -a
sudo fallocate -l 30G /swapfile
sudo chmod 600 /swapfile
sudo mkswap /swapfile
sudo swapon /swapfile
```

## License

BSD-3 License. See [LICENSE](LICENSE) for details.
