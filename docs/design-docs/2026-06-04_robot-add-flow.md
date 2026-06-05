# 모듈 설치 → 로봇 추가 → 실행: 전체 플로우

## 결정

로봇은 **모듈 매니페스트(`module.json`)가 정의**하고, 사용자가 **로봇 페이지 폼**으로
인스턴스를 만들며, 설정값은 **`robots.settings`(JSON 한 컬럼)** 에 저장된다. 실행은
**robot:start → ros2_bridge `StartRobotDriver`** 가 매니페스트 driver 블록을 보고
local / sdk / remote 중 하나로 분기한다. 이 문서는 그 end-to-end 경로와, 새 필드를
추가할 때 반드시 같이 손대야 하는 지점(특히 **settings 화이트리스트**와 **편집폼
로딩**)을 못박는다.

## 배경 (Why)

로봇 종류가 모듈로 무한히 늘어나므로, 코드에 로봇을 하드코딩하지 않고 매니페스트로
기술한다. 폼 필드(`custom_fields`)·드라이버·IK 전부 `module.json` 데이터다. 단,
"매니페스트가 선언한 임의의 필드"가 저장·로드·실행까지 흐르려면 **여러 레이어가 같은
키를 알아야** 한다. 이 키 동기화를 빠뜨리면 "입력해도 저장 안 됨 / 저장됐는데 편집폼에
안 뜸 / 실행 시 무시됨" 같은 조용한 버그가 난다 (실제로 ssh_* 필드에서 셋 다 겪음).

## 구조 / 적용 (데이터 흐름)

### 1) 매니페스트 (소스 오브 트루스)
`modules/<robots|sensors|extensions>/<name>/module.json` →
설치 시 `${EASYTRAINER_DATA_DIR}/project/modules/<id>.json` + ros2 패키지는
`ros2_ws/src/<id>/` 로 복사 (release/ui/modules.py 의 download_module).
- `robots[].spec`: `joint_names / joint_*_bounds / role(single_arm|dual_arm|tool) /
  tool_inner / tool_index / ik_available / custom_fields[]`
- `robots[].driver`: `kind, read_topic, write_topic, write_topic_msg,
  interpolation, launch{}, remote{}, pre_launch[], post_launch[]`
- `robots[].ik`: `urdf_path / urdf_package_dir({ros2_root}) / ee_definitions / joints_to_lock`
- 런타임 로더: [ros2/ros2_bridge/configs/module_loader.py](../../ros2/ros2_bridge/configs/module_loader.py)
  (`get_robot_driver_launch/hooks/remote`, `get_robot_config` for IK, `_modules_dir`).

### 2) 프론트 폼 — 추가/편집 ([frontend/src/pages/v2/RobotPage.vue](../../frontend/src/pages/v2/RobotPage.vue))
- `robotForm` 가 모든 입력 필드를 정의. **custom_fields 는 `show:` 술어로
  `getFormRobotInfo(form).custom_fields.includes('<key>')` 일 때만 렌더**.
  새 custom field 를 매니페스트에 추가하면 **여기 RobotPage 의 robotForm 목록에도
  대응 항목(label/type/default)을 추가**해야 화면에 뜬다. (type: text/number/
  password/select → [FormDialog.vue](../../frontend/src/components/v2/FormDialog.vue) 가 분기 렌더.)
  i18n 키는 ko-KR/en-US 양쪽 필수.
- `openEditRobotForm(robot)`: 값 로드 우선순위 **robot 최상위 → `robot.settings[key]`
  → field.default**. custom_fields 는 모델에 @property 가 없어 최상위에 안 뜨므로
  **반드시 `robot.settings` 폴백이 있어야 채워진다** (이게 빠져서 ssh_* 가 편집폼에
  안 뜨는 버그가 있었음 → 2026-06-04 수정).
- `saveRobot(formData)` → 신규 `POST /robot`, 수정 `PUT /robot/:id`. formData 는 폼의
  모든 키를 그대로 보냄.

### 3) 백엔드 저장 ([backend/api/routes/robot.py](../../backend/api/routes/robot.py))
- `create_robot` / `update_robot` 가 `settings` dict 를 조립:
  role/read·write topic/joints/bounds/tool_index/interpolation + `_apply_ik_settings`,
  그리고 **custom_fields 화이트리스트 루프**:
  ```python
  custom_fields = ['can_port','ip_address','port','changer_address','serial_port',
                   'gripper_port','ssh_host','ssh_user','ssh_port','ssh_password']
  for field in custom_fields:
      if field in request.json: settings[field] = request.json.get(field)
  ```
  **이 리스트에 없는 키는 조용히 버려진다.** 새 custom field 추가 시 create/update
  **양쪽** 에 키를 넣어야 한다 (안 넣어서 ssh_* 저장이 안 되던 버그 → 2026-06-04 수정).
- 저장: `RobotModel.settings` (JSON TextField). 조회: `to_dict()` 가 `settings` 를
  **중첩 dict** 로 반환 + 일부 키(serial_port/can_port/role 등)는 @property 로 최상위
  노출. **ssh_*/gripper_port 는 @property 가 없어 `settings` 안에만 존재** →
  프론트는 settings 에서 읽어야 함(2)번 참고).
- 백엔드는 Flask 리로더 없이 떠 있어 라우트 변경은 **backend 컨테이너 재시작** 필요.

### 4) 실행 (robot:start → bridge)
- `POST /robot:start` 가 robot row + 매니페스트 driver 필드(interpolation/write_topic/
  write_topic_msg/sdk_control/sdk_type)를 settings 에 합쳐 `StartRobotDriver(settings_json)`
  gRPC 호출.
- [ros2/ros2_bridge/services/driver_service.py](../../ros2/ros2_bridge/services/driver_service.py)
  `StartRobotDriver` 분기:
  1. **remote** (`_remote_is_active`: settings 에 `ssh_host` 존재) →
     `_start_remote_robot_driver`: provision(멱등 step, check 성공 시 skip) → payload
     scp → `ssh -tt` launch(tracked). 토픽만 DDS 로 흐름. 키/비번(sshpass) 인증.
  2. **sdk_control** → ROS 드라이버 없이 Agent 가 SDK 직접 제어.
  3. **local** → `_build_argv_from_launch`(매니페스트 driver.launch) → `ros2 launch`.
  공통: `interpolation` 이면 interpolation_node 기동, post_launch 훅 실행.
- 명령 경로: Agent → `ec_joint_cmd` → interpolation_node → `write_topic`
  (`std_msgs/Float64MultiArray` 또는 `trajectory_msgs/JointTrajectory`, settings.write_topic_msg).
  관측: `read_topic`(보통 `/joint_states`). dual_arm 은 좌+우 관절을 한 벡터로 합쳐
  단일 write_topic 으로 보냄(agent.py) — 로봇 쪽에 그 전 관절을 받는 컨트롤러가 있어야 함.
- bridge 도 장기 실행 프로세스 → 코드 변경은 **ros2 컨테이너 재시작** 후 반영.

## 운영 시 주의 (새 필드/로봇 추가 체크리스트)

새 **custom field** 를 추가할 때 동기화해야 하는 곳 (하나라도 빠지면 조용히 깨짐):
1. `module.json` `spec.custom_fields[]`
2. RobotPage.vue `robotForm` 항목 (+ FormDialog 가 그 `type` 지원하는지)
3. i18n ko-KR + en-US 라벨
4. robot.py `custom_fields` 화이트리스트 **create + update 둘 다**
5. 편집폼이 `robot.settings` 폴백으로 로드하는지 (openEditRobotForm)

런타임 반영:
- frontend → quick_apply + 브라우저 새로고침(HMR)
- backend(robot.py) → quick_apply + **backend 재시작**
- ros2_bridge → quick_apply + **ros2 재시작**
- module.json → 매니페스트는 start 때마다 새로 읽힘(캐시 없음)이라, 런타임
  `project/modules/<id>.json` 갱신만으로 즉시 반영. 단 정식 배포는 재릴리즈+재설치
  (see [/module-deploy], 2026-06-02 remote-ssh-robot-driver design-doc).

원격 로봇 특이사항:
- `ssh_host` 비면 local 분기 → 원격 전용 로봇(AI Worker 등)은 ssh_host 없으면
  "Unsupported robot type" 으로 실패.
- 대상은 **실제 그 로봇 PC** 여야 함. 예: AI Worker 의 `container.sh start` 는
  `robotis/ai-worker:*`(Orin/arm64+CUDA) 이미지를 pull → 일반 x86 PC 에선 받지도 돌지도
  못함. ssh_user 도 로봇 기본값(AI Worker=robotis, OMY=root)과 맞아야 함.
- EasyTrainer ros2 스택과 로봇이 **같은 ROS_DOMAIN_ID** 여야 토픽이 보임.
