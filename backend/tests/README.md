# backend/tests — integration & dual-arm sim tests

이 디렉터리는 **실행 중인 시스템에 붙는 클라이언트 테스트**다. backend(5000),
ROS2 bridge, (UI 테스트의 경우) frontend dev server(5173)가 이미 떠 있다고
가정한다. 컨테이너는 `network_mode: host` 라 호스트에서 `localhost`로 바로 접근된다.

> 위치 근거: top-level `tests/`는 `quick_apply.sh` 동기화 대상이 아니라
> `/opt/easytrainer/project` 에 들어가지 않는다. `backend/` 아래에 두면 기존
> `test_e2e_pipeline.py` 처럼 동기화·컨테이너 반영되어 설치본에서도 재현 가능하다.

## 구조

```
backend/tests/
├── test_e2e_pipeline.py          # 기존 단일팔 전체 파이프라인 e2e (record→train→plan)
├── helpers/
│   ├── api.py                    # REST 헬퍼 (start/stop/status, subscribe)
│   └── control.py                # Socket.IO 제어 (move_robot_*_delta, robot_status 구독)
├── sim_dual_arm/                 # TASK1: 단일 토픽 role='dual_arm' (휴머노이드형)
│   ├── test_keyboard_ik.py       #   키보드/IK 제어 (Python, socketio)
│   └── test_pendant.js           #   RobotPendant IK (Playwright)
└── sim_dual_arm_assembly/        # TASK2: 서로 다른 토픽 두 로봇 + Assembly
    ├── test_keyboard_ik.py
    └── test_pendant.js
```

## 사전 준비 (시스템 단계 — 사용자 실행)

dual-arm 씬/URDF/launch 와 노드 멀티그룹 확장은 `ros2/ros2_ws/src/mujoco_world/`
에 새로 추가됐다. 런타임(`/opt/easytrainer/project`)에 반영하려면:

```bash
# 1) dev 체크아웃 → 런타임 프로젝트로 동기화 (backend + ros2_ws/src 포함)
bash scripts/quick_apply.sh ./ /opt/easytrainer/project

# 2) ros2 컨테이너에서 mujoco_world 재빌드 (새 launch/assets/urdf 가 install/share 로)
docker exec easytrainer_ros2 bash -lc \
  "cd /root/ros2_ws && colcon build --packages-select mujoco_world && source install/setup.bash"

# 3) backend 재기동 (새 blueprint + 부트 시드 반영)
docker restart easytrainer_backend
```

## 실행

```bash
# 단독 스모크: dual_arm 토픽 14개 확인 (ros2 컨테이너 안)
docker exec easytrainer_ros2 bash -lc \
  "source /root/ros2_ws/install/setup.bash && ros2 topic echo --once /dual_arm_test/joint_states"

# Python 키보드/IK 테스트 — backend 컨테이너 안에서 실행
#   (호스트엔 requests/socketio 가 없음. 컨테이너는 host-net 이라 127.0.0.1:5000 동일)
docker exec -w /root easytrainer_backend python -m backend.tests.sim_dual_arm.test_keyboard_ik
docker exec -w /root easytrainer_backend python -m backend.tests.sim_dual_arm_assembly.test_keyboard_ik
#   --keep  : 테스트 후 sim 을 끄지 않고 남겨둠 (Playwright/디버깅용)

# RobotPendant IK (Playwright; 호스트에서, skill executor 가 playwright 모듈을 해석)
cd .claude/skills/playwright-skill
node run.js ../../../backend/tests/sim_dual_arm/test_pendant.js
node run.js ../../../backend/tests/sim_dual_arm_assembly/test_pendant.js
#   스크린샷: /tmp/dual_arm_test_*.png, /tmp/dual_arm_assembly_test_*.png
```

## 무엇을 검증하나

- **sim_dual_arm** — 한 토픽 14관절(`[L1..L6,Lgrip,R1..R6,Rgrip]`) role='dual_arm'
  로봇이 키보드(=`move_robot_ee_delta`) 와 RobotPendant 로 **양팔 각각** 움직이고,
  L_ee/R_ee IK 가 올바른 팔로 매핑되는지(좌측 delta → 좌측 관절만) 확인.
- **sim_dual_arm_assembly** — 서로 다른 토픽의 두 single_arm 로봇을 Assembly 로
  묶었을 때, 각 로봇이 자기 토픽으로 독립 제어되고(크로스토크 없음) 각자의 IK 가
  동작하는지 확인.

REST 엔드포인트: `/<env>:start`, `/<env>:stop`, `/<env>/status`, `/<env>:reset`,
`/<env>:reset_world`  (`<env>` = `dual_arm_test` | `dual_arm_assembly_test`).
