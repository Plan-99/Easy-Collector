# cuRobo Integration - Project Guide

cuRobo 기반 모션 플래닝 환경. Isaac Sim과 직접 통신하여 trajectory를 생성/실행한다.
MoveIt Bridge 불필요 — cuRobo가 trajectory를 직접 생성하므로 FollowJointTrajectory 액션 서버를 거치지 않는다.

## Docker Environment

- **Base image**: `nvidia/cuda:12.4.0-devel-ubuntu22.04` (NOT pytorch image — conda conflict 발생)
- **PyTorch**: pip install with `--index-url https://download.pytorch.org/whl/cu124` (CUDA 12.4 전용)
- **cuRobo install**: `git clone` + `pip install --no-build-isolation --no-deps`
  - `TORCH_CUDA_ARCH_LIST="8.6"` 필수 (RTX 3070 기준, 다른 GPU는 해당 arch 확인)
- **warp-lang**: 최신 버전 사용 (pinned 버전 X, cuRobo와 호환성 문제 있음)

## Container Structure

```
/root/ws/src/
  curobo_control/          # cuRobo 컨트롤러 패키지
  isaac_control_core/      # symlink -> 호스트 isaac_control_core/
  piper_description/       # symlink -> 호스트 ros_pkgs/piper_description/
```

`docker_start.sh`가 volume mount + symlink 설정 처리.

## curobo_control Package

### CuroboController 설계 결정 사항

#### ee_link 설정
- piper.yml에서 `ee_link`는 반드시 **"gripper_base"** 사용 (tcp 아님)
- **중요**: `tcp`로 설정하면 top-down IK가 모두 실패함 (cuRobo에서 tcp는 IK solver와 호환 안 됨)
- `gripper_length_override = -0.03` (PiperConfig 기본값 -0.05 대신 사용)

#### MotionGenConfig 임계값
- `rotation_threshold = 0.5` (~29도) — 목표 도달 판정 회전 허용 오차
- `position_threshold = 0.005` (5mm) — 목표 도달 판정 위치 허용 오차

#### move_to_pose vs move_linear
- `move_to_pose`: **MotionGen** 사용 — 큰 이동, 장애물 회피 가능
- `move_linear`: **IK + joint space 보간** 사용 (MotionGen 아님) — 짧은 직선 이동에 적합
  - IK로 목표 관절값만 구하고, 현재→목표를 linear interpolation
  - 20 steps, dt는 velocity_scaling에 반비례
  - grasp_orientation 사용 (TF 아님) — TF orientation drift 방지
  - 가속/감속 프로파일: 처음/마지막 15% 구간에서 점진적 가속/감속
  - MotionGen fallback: IK 실패 시 (max_diff > 2.0 rad) MotionGen으로 자동 전환
    - fallback 시 **동일한 orientation** 그대로 전달 (grasp_orientation + low weight 방식 아님)

#### IK Solver
- `retract_config`와 `seed_config` 모두 사용하여 가장 가까운 해 탐색
- 두 config를 배치로 전달하여 cuRobo가 최적 해를 선택

#### Orientation 처리
- `move_linear`에서 orientation이 None일 때: TF 대신 **grasp_orientation** 사용
  - TF 데이터 stale 문제 및 orientation drift 방지
- `needs_orientation_correction = False`
  - cuRobo는 이미 orientation을 포함하여 planning하므로 MoveIt 스타일 보정 적용 시 config jump 발생

#### Gripper 처리
- **Mimic joint**: `joint8 = -joint7` 을 모든 command에 명시적으로 설정
  - MoveIt은 Bridge가 mimic을 처리하지만, cuRobo는 직접 publish하므로 수동 설정 필수
- **Gripper scale**: 로봇별로 다름 (`_GRIPPER_SIM_LIMITS` dict)
  - `piper`: 시뮬 0~0.035, `piper_v100`: 시뮬 0~0.05
  - `GRIPPER_REAL_MAX` / `GRIPPER_SIM_MAX`: `__init__`에서 `robot_config`에 따라 동적으로 설정
  - `_real_to_sim_gripper()`로 변환
- `_joint_state_cb` override: 시뮬레이션에서 받은 gripper joint_states (sim scale)를 실제 스케일로 변환하여 저장

#### Trajectory 실행 안정화 (`_execute_cu_trajectory`)
- **Blending**: 현재 위치 → trajectory 시작점까지 보간 구간을 앞에 추가
  - trajectory 시작점으로의 갑작스러운 점프 방지
- **가속/감속 프로파일**: 처음/마지막 10% 구간에서 점진적 가속/감속 (dt 조정)
- 실행 완료 후: 최종 position을 **40회** publish + `spin_once` **10회**로 상태 동기화
- **⚠️ hold 횟수/시간을 절대 줄이지 말 것!** 줄이면 모션 사이에 로봇이 튀는 현상 발생.
  현재 값이 안정화 최소 기준이며, 속도를 올리려면 hold가 아닌 trajectory 자체의 dt를 조정할 것.

#### move_linear hold 안정화
- IK 보간 후 최종 위치를 **50회 (1초)** hold + `spin_once` **30회**로 상태 동기화
- **⚠️ 이 값도 절대 줄이지 말 것!** move_linear 후 hold가 부족하면 descend→close_gripper 사이에 로봇이 위로 튀어오름.

#### hold_position() 메서드
- 현재 위치를 **60Hz**로 지속 publish하는 메서드
- 대기 시간 동안 로봇이 현재 위치를 유지하도록 함
- `time.sleep()` 대신 사용하여 물리 시뮬레이션 안정화

#### spin_once 호출 시점
- 모든 모션 메서드 (`move_to_joint`, `move_to_pose`, `move_linear`) 시작 전에 `spin_once` 호출
- 플래닝 전 최신 상태 데이터 확보 목적

#### cuRobo quaternion 순서
- cuRobo는 **wxyz** 순서 (ROS의 xyzw와 다름)
- `CuPose(quaternion=[[ow, ox, oy, oz]])` 형태로 전달

### GripperScaleBridge (service_runner.py)

시뮬레이션 gripper 스케일 ↔ 실제 로봇 스케일을 ROS 토픽 레벨에서 변환하는 브리지 클래스.

#### 토픽 매핑

| 방향 | 시뮬 토픽 | 실제 토픽 |
|------|-----------|-----------|
| joint_states (구독) | `/simulation/joint_states` | `/joint_states` |
| joint_command (발행) | `/simulation/joint_command` | `/joint_command` |

#### _publish_cmd() 동작
- IsaacSim용 (sim scale)과 외부 토픽 (real scale) **양쪽에 동시에 발행**
- sim scale → real scale 변환 후 `/joint_command`로 발행

#### _joint_state_cb override
- `/simulation/joint_states`에서 gripper 값(sim scale)을 수신
- real scale로 변환하여 저장 (`_joint_state_cb` 내부에서 처리)

#### 로봇별 gripper 시뮬 한계값 (_GRIPPER_SIM_LIMITS dict)
- `piper`: 0.035
- `piper_v100`: 0.05
- `GRIPPER_REAL_MAX` / `GRIPPER_SIM_MAX`는 `__init__`에서 `robot_config` 이름으로 조회하여 동적 설정

### PiperV100Config (robots/piper_v100.py)

Piper V100 로봇 설정:
- `ee_link = "link6"`
- `base_frame = "arm_base"`
- gripper range: 0 ~ 0.10 (real scale)
- cuRobo config: `piper_v100.yml` (별도 파일)
