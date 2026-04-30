# ros2_env - Project Guide

IsaacSim + ROS2 시뮬레이션 환경 프레임워크.
BaseEnv를 상속하여 로봇, 테이블, 오브젝트, 카메라를 선언적으로 배치하고
ROS2 토픽을 자동으로 퍼블리쉬한다.

## Architecture

```
ros2_env/
  base_env.py       # BaseEnv - 시뮬레이션 루프, 씬 구성, ROS2 연동, 리셋 서비스
  objects.py         # TargetObject (개별 오브젝트) + ObjectManager (배치/마커 퍼블리쉬)
  ros2_bridge.py     # OmniGraph 기반 ROS2 퍼블리셔/구독자 설정 함수들

  envs/              # 구체적인 환경 구현 (BaseEnv 서브클래스)
    franka_env1.py   #   Franka + 프리미티브 오브젝트
    franka_env2.py   #   Franka + Nucleus USD 에셋
    piper_env1.py    #   Piper (URDF 임포트) + 프리미티브 오브젝트
    piper_env_custom.py  # Piper + Custom USD 오브젝트 (스펀지, 박스, 마우스 등)
    piper_peg_in_hole.py # Piper + Peg-in-Hole 태스크 환경
```

## BaseEnv (base_env.py)

### 씬 구성 메서드 (서브클래스의 setup_scene()에서 호출)

```python
add_robot("franka")                           # 빌트인 Franka
add_robot("piper", usd_path="/path/to.usd")   # 커스텀 USD
add_robot_from_urdf("piper", urdf_path,       # URDF 자동 변환
                    position=[0, 0, 0.032])   # 로봇 위치 지정 (테이블 위 등)
add_mimic_joint("joint8", "joint7", multiplier=-1.0)  # mimic joint 수동 등록

add_table(center=[0.18, 0, 0.012], scale=[0.6, 0.5, 0.04])

add_object("RedCube", size=0.03, shape="cube", color=(1, 0.3, 0.2))
add_object("Mug", usd_path="/path/to/mug.usd", size=0.1, scale=0.01)
add_object("Basket", usd_path="/path/to.usd", size=0.2, scale=0.5,
           collision_approximation="convexDecomposition")  # 오목한 형상

add_camera(position=[1.8, 0, 1.6], orientation=[0, 40, 180],
           publish_rgb=True, publish_depth=False,
           position_delta=[0.1, 0.1, 0.05],    # ±offset 랜덤화
           orientation_delta=[5, 5, 0],         # ±deg 랜덤화
           focal_length=10)                      # 5=ultra-wide, 10=wide, 24=normal, 50=telephoto
```

### URDF 임포트 (add_robot_from_urdf)

자동으로 수행되는 작업:
1. IsaacSim URDF Importer로 USD 변환
2. `set_parse_mimic(True)` - mimic joint를 일반 joint로 변환
3. `_parse_mimic_joints()` - URDF mimic 태그 파싱 (시뮬레이션 동기화용)
4. `_apply_gripper_friction()` - link7/link8에 마찰력 1.0 적용
5. `_tune_joint_drives()` - 관절 강성/감쇠 튜닝 (진동 방지)
6. ArticulationRootAPI 자동 적용 (없을 경우)
7. `position` 파라미터 지정 시 기존 xformOp:translate를 덮어써서 로봇 위치 이동
8. `_set_robot_color()` - color 파라미터 지정 시 기존 머티리얼 셰이더 색상 변경

**position 주의사항**: URDF 임포트 시 이미 `xformOp:translate`가 생성됨.
`AddTranslateOp()`을 호출하면 중복 에러 발생. 기존 op을 찾아 값을 덮어쓰는 방식 사용.

### 로봇 색상 변경 (`_set_robot_color`)

URDF 임포트 시 `{robot}/Looks/` 하위에 머티리얼이 자동 생성됨.
새 머티리얼을 만들어 바인딩하는 방식은 동작하지 않으며 (참조된 메쉬에 전파 안됨),
**기존 셰이더의 diffuse 색상 입력을 직접 덮어쓰는 방식**이 유일한 해결책.

### 테이블 (add_table)

`center`는 테이블 중심의 실제 월드 좌표, `scale`은 UsdGeom.Cube(size=1.0)에 적용되는 스케일.
xformOp 순서는 **Translate→Scale** (center가 곧 실제 위치).

```python
add_table(center=[0.18, 0, 0.012], scale=[0.6, 0.5, 0.04])
# → 테이블 중심: (0.18, 0, 0.012), 크기: 0.6 x 0.5 x 0.04m
# → 표면 z = 0.012 + 0.04/2 = 0.032

# spawn_range로 오브젝트 배치 범위 직접 지정: [[x_min, y_min], [x_max, y_max]]
add_table(center=[0.18, 0, 0.012], scale=[0.6, 0.5, 0.04],
          spawn_range=[[0.05, -0.10], [0.30, 0.10]])
# → 생략 시 테이블 면적의 50% 범위를 자동 사용
```

### 오브젝트 (add_object)

```python
add_object("Name", size=0.06, shape="cube", color=(1, 0, 0))  # 프리미티브
add_object("Name", size=0.06, shape="cube", color=(1, 0, 0),
           orientation_range=[[0,0,0],[0,0,180]])              # 프리미티브도 orientation_range 지원
add_object("Name", usd_path="...", size=0.1, scale=0.5)       # USD 에셋
add_object("Name", usd_path="...", size=0.1, scale=0.5,
           position_range=[[x_min, y_min, z], [x_max, y_max, z]])  # 개별 배치 범위
add_object("Name", usd_path="...", size=0.1, scale=0.5,
           collision_approximation="convexDecomposition")  # 충돌 근사 방식
```

**orientation_range**: 프리미티브(cube/sphere/cylinder) 오브젝트에서도 지원 (이전에는 USD 오브젝트 전용).
`[[roll_min, pitch_min, yaw_min], [roll_max, pitch_max, yaw_max]]` (도 단위)

**collision_approximation** 옵션:
- `"convexHull"` (기본) - 볼록 외곽선, 빠름. 일반 물체용.
- `"convexDecomposition"` - 볼록 분해. 바구니/컵 등 오목한 형상에서 내부 공간 인식.
- `"meshSimplification"` - 메쉬 단순화.
- `"none"` - 원본 트라이앵글 메쉬 그대로. 가장 정확, 가장 느림.

**position_range z 좌표 주의**: z는 월드 좌표. 반드시 로봇 base z보다 높아야 한다.
로봇 base보다 낮으면 MoveIt Cartesian descend가 실패한다.
공식: `z = 테이블_표면_z + 오브젝트_반높이`. 테이블 표면 z는 로봇 base z와 일치시켜야 한다.

### Joint Drive 튜닝 값
- Revolute: stiffness=10000, damping=1000
- Prismatic: stiffness=5000, damping=500
- 기존에 drive가 있는 관절만 적용 (이미 튜닝된 로봇은 건너뜀)

### Mimic Joint Relay

`/simulation/joint_command`에 joint7만 들어와도 joint8을 자동으로 추가하여
`/simulation/joint_command_internal`로 중계하는 메커니즘.

- **토픽 흐름**: 외부 → `/simulation/joint_command` → relay 노드 (joint8 추가) → `/simulation/joint_command_internal` → OmniGraph ArticulationController
- **등록 방법**: `add_mimic_joint("joint8", "joint7", multiplier=-1.0)` — setup_scene()에서 호출
- URDF에 mimic 태그가 있으면 `_parse_mimic_joints()`가 자동 등록하지만, 태그가 제거된 경우 수동 등록 필요
- relay 노드는 시뮬레이션 루프에서 `spin_once` (최대 10회/프레임)로 콜백 처리
- **주의**: `rclpy.spin`을 별도 스레드에서 사용하면 IsaacSim rclpy context와 충돌 (`IndexError: wait set index too big`). 반드시 `spin_once` 사용.

### 시뮬레이션 루프 (run())

```
World 생성 → setup_scene() → world.reset() →
오브젝트 랜덤 배치 → 카메라 초기화 → 렌더링 워밍업 (15 step) →
ROS2 퍼블리셔 설정 → 리셋 서비스 설정 → mimic relay 설정 →
메인 루프:
  world.step(render=True)
  publish_markers()           # 오브젝트 위치 마커
  spin_once(relay_node) x10   # mimic joint relay 콜백 처리
  spin_once(reset_node)       # 리셋 서비스 콜백 처리
  if reset_requested:
    world.reset()
    randomize_all()
    15 step 안정화
```

### 환경 리셋 서비스
- 토픽: `/simulation/reset_env` (std_srvs/Trigger)
- `rclpy.create_node("env_reset_service")`로 별도 노드 생성
- 시뮬레이션 루프에서 `spin_once`로 콜백 처리
- 리셋 시 world.reset() + 오브젝트 재배치 + 15 step 안정화

## TargetObject & ObjectManager (objects.py)

### TargetObject
- 프리미티브(cube/sphere/cylinder) 또는 USD 에셋 지원
- 경로: `/World/{name}`
- 물리: RigidBody + Collision + 고마찰 PhysicsMaterial (static=1.0, dynamic=1.0)
- USD 에셋의 충돌 근사: `collision_approximation` 파라미터로 설정 (기본 "convexHull")
- `set_position(np.array)` / `get_position() -> np.array`

### ObjectManager
- 테이블 위 coverage_ratio=0.5 영역에 랜덤 배치 (겹침 방지)
- `randomize_all()` - 최소 거리: `(size1 + size2) / 2 + 0.02m`
- 마커 퍼블리쉬: `/simulation/object_markers/{name}` (Marker.SPHERE)
- **좌표계**: 로봇 base의 역변환 적용 → 로봇 base 기준 상대 좌표
- `setup_marker_publisher(robot_prim_path)` 호출 시 rclpy.init() + 노드 생성
- **frame_id**: "panda_link0"으로 하드코딩 (실제 위치 계산에는 영향 없음, 표시용)

#### 마커 orientation 계산 (objects.py)

마커에 실제 오브젝트 orientation 포함 (이전의 hardcoded `w=1.0` 방식 폐기):
- `inv_base_tf * world_tf`로 로봇 base 기준 상대 orientation 계산
- 결과를 quaternion xyzw로 마커 메시지에 포함
- `MotionController.object_orientations`로 컨트롤러 측에서 조회 가능

## ros2_bridge.py (OmniGraph 설정)

네임스페이스: `NS = "/simulation"`

모든 퍼블리셔는 OmniGraph 노드로 구현 (IsaacSim 내장).
OnTick → ReadSimTime → Publish 패턴.

### 설정 함수들

| 함수 | 토픽 | 설명 |
|------|------|------|
| `setup_clock_and_joint_state_publisher(robot)` | `/simulation/clock`, `/simulation/joint_states` | 시뮬레이션 시간 + 관절 상태 |
| `setup_robot_tf_publisher(robot)` | `/simulation/tf` | 로봇 링크 TF |
| `setup_joint_command_subscriber(robot)` | `/simulation/joint_command_internal` 구독 | ArticulationController로 관절 명령 수신 (relay 경유) |
| `setup_object_tf_publisher(paths)` | `/simulation/tf` | 오브젝트 TF |
| `publish_camera_tf(camera)` | `/simulation/tf` | 카메라 + optical 프레임 TF |
| `publish_camera_info(camera, freq)` | `/simulation/{name}_camera_info` | 카메라 내부 파라미터 |
| `publish_rgb(camera, freq)` | `/simulation/{name}_rgb` | RGB 이미지 |
| `publish_depth(camera, freq)` | `/simulation/{name}_depth` | 뎁스 이미지 |
| `publish_pointcloud(camera, freq)` | `/simulation/{name}_pointcloud` | 포인트클라우드 |

주파수 제어: `step_size = int(60 / freq)` (시뮬레이션 60Hz 기준)

## 환경 예제

### PiperEnv1 (piper_env1.py)
```python
class PiperEnv1(BaseEnv):
    def setup_scene(self):
        self.add_table(center=[0.18, 0, 0.012], scale=[0.6, 0.5, 0.04],
                      color=(0.1, 0.1, 0.1),
                      spawn_range=[[0.43, 0.18], [0.18, -0.18]])
        # 테이블 표면 z = 0.012 + 0.04/2 = 0.032
        self.add_robot_from_urdf("piper", PIPER_URDF,
                                position=[0, 0, 0.032])
        self.add_object("RedCube", size=0.03, shape="cube")
        ...
        self.add_camera(position=[1.8, 0, 1.6], orientation=[0, 40, 180])
```

### PiperEnvCustom (piper_env_custom.py)
```python
class PiperEnvCustom(BaseEnv):
    def setup_scene(self):
        self.add_table(center=[0.385, 0, 0.012], scale=[0.9, 1.0, 0.04], ...)
        self.add_robot_from_urdf("piper", PIPER_URDF, position=[0, 0, 0.032])
        # position_range로 오브젝트별 개별 배치 범위 설정
        # z = 테이블 표면(0.032) + 오브젝트 반높이
        self.add_object("WhiteBox", usd_path=..., size=0.2, scale=0.6,
                        position_range=[[0.18, -0.18, 0.052], [0.43, 0.18, 0.052]],
                        collision_approximation="convexDecomposition")  # 바구니 내부 인식
        self.add_object("GreyCube", usd_path=..., size=0.08, scale=0.5,
                        position_range=[[0.18, -0.18, 0.052], [0.43, 0.18, 0.052]])
        # 테이블 위 범위 바깥 (큐브 범위와 겹치지 않는 곳)
        self.add_object("Mouse", usd_path=..., size=0.1, scale=0.5,
                        position_range=[[0.50, -0.18, 0.052], [0.75, 0.18, 0.052]])
```

### 실행
```bash
# IsaacSim Docker 컨테이너 내부
/workspace/isaaclab/_isaac_sim/python.sh ros2_env/envs/piper_env1.py
/workspace/isaaclab/_isaac_sim/python.sh ros2_env/envs/piper_env_custom.py

# headless 모드 (렌더링 없이 실행)
/workspace/isaaclab/_isaac_sim/python.sh ros2_env/envs/piper_env_custom.py --headless
```

`--headless` CLI 플래그: `pi0_env.py` 및 `piper_env_custom.py`에서 지원.
`argparse`로 파싱하여 `SimulationApp({"headless": True})`로 전달.

## 중요 구현 세부사항

- **IsaacSim import 지연**: 모든 isaacsim import는 메서드 내부에서 수행 (SimulationApp 생성 후)
- **rclpy.init() 충돌 방지**: ObjectManager와 리셋 서비스 모두 `try: rclpy.init() except RuntimeError: pass`
- **Mimic joint**: URDF mimic 태그가 IsaacSim physics constraint를 생성하여 joint가 멈추는 버그 있음.
  URDF에서 mimic 제거 + bridge의 `_append_mimic()`으로 소프트웨어 동기화가 해결책.
- **OmniGraph vs rclpy**: 센서/TF/joint_states는 OmniGraph (C++ 성능), 마커/서비스는 rclpy (유연성)
- **마커 좌표계**: `publish_markers()`에서 로봇 base의 역변환을 적용하여 상대 좌표로 변환.
  `header.frame_id = "panda_link0"` (하드코딩됨 — 실제 위치 계산에는 영향 없음, 컨트롤러가 frame_id를 사용하지 않음)
- **로봇 위치와 오브젝트 z 정렬**: 로봇 base z = 테이블 표면 z로 맞추고,
  position_range z = 테이블 표면 z + 오브젝트 반높이로 설정해야 MoveIt 플래닝이 성공한다.
  로봇을 원점(0,0)에서 벗어나게 배치할 경우, position_range의 XY도 로봇 도달 범위 내로 조정 필요.
- **카메라 랜덤화 (position_delta / orientation_delta)**: `position_range` / `orientation_range` 방식에서
  기준 위치로부터 ±offset으로 샘플링하는 `position_delta` / `orientation_delta` 방식으로 변경.
- **배경 랜덤화**: 기본 `ground_plane` 대신 Custom `BackgroundFloor` + OmniPBR material 사용.
  `DisplayColorAttr`은 RTX renderer에서 동작하지 않으므로 OmniPBR이 필수.
  `set_background_randomization()`으로 DomeLight grayscale 색상 랜덤화 + texture 제거.
