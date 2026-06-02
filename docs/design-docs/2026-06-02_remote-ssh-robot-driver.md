# 원격(SSH) 로봇 드라이버

## 결정

온보드 PC 를 가진 로봇(예: ROBOTIS OMY)은 드라이버를 **로봇 PC 에서 직접** 실행하고,
EasyTrainer 는 같은 DDS 도메인에서 **토픽만** 주고받는다. 로봇이 `module.json` 의
`driver.remote` 블록을 선언하고 robot 설정에 `ssh_host` 가 채워지면, EasyTrainer 의
ros2 컨테이너가 로봇 PC 로 SSH 접속해 ① 멱등 provisioning ② 드라이버 launch 를
순차 수행한다. `ssh_host` 가 비면 기존 로컬 launch 로 폴백한다 — 동일 manifest 가
원격/로컬 양쪽을 커버한다.

## 배경 (Why)

기존 드라이버는 전부 ros2 컨테이너 안의 subprocess(`ros2 launch …`)로 떴고, 로봇과의
통신은 100% DDS 토픽이다. 따라서 그 launch 가 로컬이든 원격이든 **같은
`ROS_DOMAIN_ID` 의 DDS 망에 토픽만 올라오면 EasyTrainer 입장에선 동일**하다. OMY 처럼
자체 PC(온보드)에서 ROS 를 도커로 돌리도록 설계된 로봇은 USER PC 가 직접 Dynamixel
버스에 접근할 수 없으므로(이더넷 분리), "USER PC 가 SSH 로 로봇 PC 의 드라이버를
띄우고 토픽만 소비" 하는 운용이 일반적이다.

대안:
- **토픽 패스스루(런치 없음)**: 로봇 PC 에서 사용자가 수동으로 드라이버를 띄우고
  EasyTrainer 는 구독만. → 최초 셋업·기동을 사람이 매번 해야 함. 기각.
- **U2D2 로컬 번들**: OMX 처럼 컨테이너 안에서 직접 구동. → OMY 의 ttyAMA2/ttyAMA4 는
  로봇 온보드 UART 라 USER PC 에 없음. 기각(로컬 폴백으로만 유지).

## 구조 / 적용

- **manifest 스키마**: `robots[].driver.remote`
  (`ros2/ros2_bridge/configs/module_loader.py:get_robot_driver_remote`)
  - `enabled_when`/`host_field`(기본 `ssh_host`), `user_field`, `port_field`,
    `default_user`/`default_port`, `ros_domain_id`
  - `read_topic`/`write_topic`/`write_topic_msg` — **원격 모드 토픽 override**
  - `payload[]` `{src,dst}` — 로봇 PC 로 scp 할 파일(있으면)
  - `provision[]` `{name, check?, run, timeout?, optional?}` — `check` 가 exit 0 이면
    skip(멱등). 최초 1 회만 무겁고 이후엔 전부 skip → 사실상 launch 만 실행.
  - `launch` — 원격에서 blocking 으로 도는 드라이버 실행 sh 한 줄
- **실행기**: `ros2/ros2_bridge/services/driver_service.py`
  - `StartRobotDriver` 가 `_remote_is_active` 면 `_start_remote_robot_driver` 로 분기
    (로컬 pre_launch / SDK / ros2 launch 스킵).
  - `_ssh_base`: provision 은 `ssh -T`, launch 는 `ssh -tt`(TTY 강제 → 로컬 ssh 종료
    시 원격 프로세스 트리에 SIGHUP 전파). 옵션: `StrictHostKeyChecking=accept-new`,
    `BatchMode=yes`(키 기반 전제), `ServerAliveInterval`.
  - `_remote_inner`: 모든 원격 명령 앞에 `export ROS_DOMAIN_ID=<n>;` 를 붙이고
    `{key|default}` placeholder 치환.
  - launch 는 `_start_subprocess` 로 tracked → 기존 Stop/로그/보간 노드 경로 그대로.
    토픽 override 는 보간 노드 spawn 전에 `settings` 에 덮어써 반영.
- **프론트**: `frontend/src/pages/v2/RobotPage.vue` custom_fields 에 `ssh_host`/
  `ssh_user`/`ssh_port` (+`gripper_port`) 추가, i18n ko/en.
- **OMY 적용**(`modules/robots/omy/module.json`): 로봇 PC 의 docker 컨테이너
  `open_manipulator`(network/ipc/pid host) 안에서 **stock** 런치 실행 —
  3m: `omy_3m.launch.py` → `/arm_controller/joint_trajectory`,
  f3m: `omy_f3m_follower_ai.launch.py` → `/leader/joint_trajectory`(arm_controller 가
  joint1-6+rh_r1_joint 포함). 둘 다 `trajectory_msgs/JointTrajectory`. 보간 노드가
  JointTrajectory 1-point 를 흘리므로(이미 지원) **로봇에 커스텀 파일 배포 불필요**.

## 운영 시 주의

- **DDS 도메인 일치**: EasyTrainer 의 ros2 스택도 로봇과 같은 `ROS_DOMAIN_ID`(OMY 는 30)
  여야 토픽이 보인다. 이건 컨테이너 env(compose) 레벨 설정이라 per-robot 으로 바꿀 수
  없음 — 시스템에서 맞춰야 함. 서브넷이 갈리거나 멀티캐스트가 막히면
  `ROS_DISCOVERY_SERVER`/peers 설정 필요.
- **SSH 인증은 키/비밀번호 둘 다 지원**.
  - 키 기반(권장): robot 설정에 `ssh_password` 를 비워두면 `BatchMode=yes` 로 키 인증.
    로봇 PC 에 EasyTrainer 호스트 공개키를 미리 등록(`ssh-copy-id`)해 둘 것.
  - 비밀번호: `ssh_password` 를 채우면 `sshpass -p <pw>` 로 암호 인증
    (`PreferredAuthentications=password,keyboard-interactive`, `PubkeyAuthentication=no`).
    **ros2 컨테이너에 apt `sshpass` 필요** (모듈 `dependencies.apt` 에 추가 → 이미지 baked).
    비밀번호는 robot settings(DB)에 평문 저장되므로 가능하면 키 방식을 권장.
- **종료 전파**: `ssh -tt` + `docker exec -t` 로 SIGHUP 전파에 의존. 드물게 원격
  ros2 launch 가 남으면 로봇 PC 에서 수동 정리 필요(향후 stop 훅으로 보강 가능).
- **첫 기동은 느림**: 최초엔 `container.sh start`(이미지 pull 포함)·git 등으로 수 분.
  이후엔 provision check 가 전부 skip → launch 만.
- **컨테이너 exec 형식**: ROBOTIS `container.sh` 엔 `exec` 서브커맨드가 없어
  `docker exec open_manipulator …` 를 직접 사용한다. 컨테이너명/ROS distro(jazzy)/ws
  경로가 바뀌면 `module.json` 의 `remote.launch` 문자열만 고치면 된다(코드 변경 불필요).
- IK 는 항상 EasyTrainer 컨테이너에서 로컬 URDF(`omy_description`)로 계산 — 원격/로컬
  무관하게 `omy_description` 패키지는 EasyTrainer 쪽에 설치돼 있어야 한다.
