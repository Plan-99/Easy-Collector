# demo/peg_in_hole — 파이프라인 실행 결과

날짜: 2026-06-04 · 환경: tutorial peg-in-hole (scene_peg.xml), ROS_DOMAIN_ID=0, 3-cam (top/front/wrist)

## 요약

| 단계 | 상태 |
|------|------|
| 데모 환경 등록 (`demo/peg_in_hole`) | ✅ 완료 |
| Sim Activation 버튼 → 다이얼로그 → 실행 (UI/API) | ✅ 검증 (playwright + API) |
| sim 기동 (bridge `StartLaunch`) + 토픽 publish | ✅ |
| `/tutorial/randomize` · `/tutorial/check_success` · `/tutorial/object_poses_json` 서비스 | ✅ **복원**됨 (아래 참고) |
| 자동 수집기 plumbing (randomize→run_episode→check_success) | ✅ end-to-end 동작 |
| 100 에피소드 수집 | ❌ **미완료** — planner 성공률 0% (아래 근본원인) |
| LeRobot 어셈블 → ACT 학습 → 평가 → 성공률 | ⛔ 수집 데이터 없음으로 미실행 |

## 무엇을 고쳤나 (model-tester 서비스 복원)

`/model-test`·`/policy-tournament`가 의존하는 데이터 수집 파이프라인은 이 브랜치의
**Wrist View Reach 작업**이 `mujoco_world_node.py` / `sim_runner.py`를 재작성하면서
**조용히 깨져** 있었다. 다음 3개가 노드에서 제거돼 있었고, 이를 복원했다:

1. `/tutorial/randomize` (Trigger) — peg/hole_base freejoint 재샘플 + 팔 home 스냅.
   안착(settle) 후 **실제 안착 pose**를 `sampled`로 반환 (planner가 정확히 grasp 하도록).
2. `/tutorial/check_success` (Trigger) — `task_success.check_success()` ground-truth 체크.
3. `/tutorial/object_poses_json` (std_msgs/String) — planner가 plan 전 대기하는 객체 pose 스트림.

또한 `ros2/ros2_ws/src/mujoco_world/setup.py`에 누락돼 있던 console_scripts
(`tutorial_planner_node`, `tutorial_collector_node`, `tutorial_eval_bridge`)와
모듈 파일들(`task_success.py`, `tutorial_*` 등)을 git 소스에 정식 편입했다 —
이전엔 컨테이너 install 공간에만 존재해 재빌드 시 사라지는 상태였다.

복원 결과: sim 기동 → 세 서비스 응답 → 수집기가 `randomize → run_episode → check_success`
전체 루프를 끝까지 실행한다 (구조적으로 완전 동작).

## 근본원인: 왜 수집이 0% 인가

수집기는 **ground-truth 성공 에피소드만** 저장한다. 그런데 scripted planner가
현재 sim에서 peg 삽입에 **단 한 번도 성공하지 못한다** (20회+ 시도 0/N).

진단 (planner waypoint 로그 + 직접 측정):

- sim의 **raw joint 추종은 완벽**하다. `joint_command`로 `[0,0.6,-0.6,0,0.3,0,0.02]`를
  보내면 정확히 그 값에 도달한다.
- 그런데 planner의 **실시간 ramp 명령에 대해 팔이 수십 mm 뒤처진다**
  (track_err 첫 waypoint 140~150mm). IK 해는 정확(ik_err 0.05mm)한데 팔이 못 따라간다.
- 원인: `SimRunner._run`이 **단일 스레드에서 물리(mj_step)와 카메라 렌더를 함께** 돌린다.
  Wrist View Reach가 추가한 **wrist depth 렌더**까지 더해지면서, 무거운 렌더 프레임이
  물리 루프를 real-time 밑으로 끌어내려, planner의 real-time 페이스 명령을 팔이 추종하지 못한다.
- 검증: `image_publish_hz`를 낮추면 track_err가 149mm→44mm로 **개선**된다(원인 확정).
  하지만 44mm도 5mm grasp 허용오차엔 못 미쳐 성공하지 못한다.
- 시도했으나 불충분: 객체 settle 후 실제 pose 반환 / 렌더 부하 감소 / `plan_hz` 하향 /
  물리 catch-up 스테핑(되돌림 — wrist-reach 활성 코드라 미검증 변경 회피).

요컨대 **scripted planner는 이전(렌더 가벼운) sim에 맞춰 튜닝**돼 있었고, wrist-reach가
바꾼 현재 sim에서는 물리-렌더 결합으로 추종이 깨진다. 이건 데모 기능과 무관한,
이미 존재하던 환경 drift다.

## 다음에 이걸 끝내려면

가장 깔끔한 해법은 `SimRunner`에서 **물리와 렌더를 분리**(물리는 고정 rate 전용 루프,
렌더는 별도 cadence)하거나, planner `_interp_to`가 고정 hold 대신 **joint 수렴을 폴링**해
sim 속도와 무관하게 도착을 보장하게 하는 것. 둘 다 wrist-reach가 활발히 개발 중인
코드를 건드리므로, 사용자 확인 후 진행 권장.

수집만 되면 이후는 그대로 동작한다:

```bash
# 1) 데모 기동 (UI Sim Activation 또는 API)
# 2) 수집 (ROS_DOMAIN_ID=0!  — 스킬 문서의 1은 구버전)
docker exec easytrainer_ros2 bash -lc 'source /opt/ros/humble/setup.bash && source /root/ros2_ws/install/setup.bash &&
  export ROS_DOMAIN_ID=0 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp &&
  ros2 run mujoco_world tutorial_collector_node --num-episodes 100 \
    --cameras top_cam,front_cam,wrist_cam \
    --output-dir /opt/easytrainer/training_data/raw_episodes'
# 3) backend/tools/model_tester: register_run → assemble_dataset(--cameras 2:top_cam,3:front_cam,4:wrist_cam)
#    → _run_mini_train (ACT) → tutorial_evaluator (--sensor-map sensor_2=top,sensor_3=front,sensor_4=wrist)
```
