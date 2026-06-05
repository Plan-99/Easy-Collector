---
description: policy-lab readiness 게이트 — EasyTrainer 컨테이너 + tutorial sim + eval bridge 가 살아있는지 확인
argument-hint: "[--no-bridge]"
---

# /lab-smoke — 랩 readiness 게이트

토너먼트/진화를 돌리기 전에 **한 번**, 그리고 야간 루프 각 라운드 전에 호출. EasyTrainer 의 기본
smoke 에 더해 **tutorial sim + eval bridge** 까지 확인한다 (정책 평가에 필수).

```bash
EASYTRAINER_ROOT="${EASYTRAINER_ROOT:-$(git rev-parse --show-toplevel)}"  # 랩이 레포 안 → EasyTrainer 루트
```

## 1. EasyTrainer 기본 게이트
```bash
bash "$EASYTRAINER_ROOT/scripts/smoke.sh" --quick
```
exit≠0 → 인프라 문제. `docker compose -f "$EASYTRAINER_ROOT/docker-compose.yml" up -d backend ros2` 후 30s 대기 재시도.

## 2. tutorial sim 기동 (없으면)
```bash
docker exec easytrainer_ros2 bash -lc "ros2 topic list 2>&1 | grep -q /tutorial/joint_states" \
  || docker exec -d easytrainer_ros2 bash -lc '
    source /opt/ros/humble/setup.bash &&
    source /root/ros2_ws/install/setup.bash &&
    export ROS_DOMAIN_ID=1 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp &&
    ros2 launch mujoco_world tutorial_pegtask.launch.py'
# /tutorial/joint_states, /tutorial/check_success, /tutorial/randomize 가 보일 때까지 폴링(~5s)
```

## 3. eval bridge 기동 (`--no-bridge` 아니면)
```bash
docker exec easytrainer_ros2 curl -s http://127.0.0.1:7799/health | grep -q ok \
  || docker exec -d easytrainer_ros2 bash -lc '
    source /opt/ros/humble/setup.bash &&
    source /root/ros2_ws/install/setup.bash &&
    export ROS_DOMAIN_ID=1 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp &&
    export PYTHONPATH=/opt/easytrainer/project/ros2/ros2_ws/src/mujoco_world:$PYTHONPATH &&
    python3 -u -m mujoco_world.tutorial_eval_bridge --port 7799'
docker exec easytrainer_ros2 curl -s http://127.0.0.1:7799/health   # → {"ok": true}
```

## 종료
- 모두 통과 → `READY` (토너먼트/진화 진행 가능).
- 어느 단계든 실패 → 무엇이 죽었는지 보고하고 **중단** (야간 루프면 그 라운드 skip + 연속 실패 카운트).

> EasyTrainer 내부 구조는 [../../docs/easytrainer-integration.md](../../docs/easytrainer-integration.md) 참고.
