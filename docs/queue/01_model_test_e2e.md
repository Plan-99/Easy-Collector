---
title: model-test 스킬 end-to-end 검증 (수집 → 어셈블 → 학습 → 평가)
size: large
verify: e2e
manual: skip
affected:
  - backend
  - ros2
prerequisite_specs: []
---

# /model-test end-to-end 검증

## 목표 (Why)

`/model-test` 스킬은 튜토리얼 peg-in-hole 태스크에서 한 줄 입력으로
정책 성공률을 산출하는 자율 벤치마크 파이프라인이다. 각 단계 컴포넌트는
스모크 검증되었지만 (collector 5/5, assembler, eval bridge, evaluator),
**진짜 학습된 정책이 비-제로 success rate을 내는가**는 아직 검증되지 않았다.

이 spec은 야간 자율 실행으로 50 에피소드 수집 → ACT 학습 → 30 trial 평가 →
JSON 리포트를 생성하고, success rate ≥ 60% 이면 통과로 본다.

## 변경 내용 (What)

코드 변경은 최소. 이 spec은 **벤치마크 실행**이 본질이다.

- (있다면) 어셈블러 / 평가기에서 발견되는 버그 fix
- `/opt/easytrainer/training_data/raw_episodes/` 를 fresh 로 비운 뒤
  50 에피소드 수집
- LeRobot v2.1 데이터셋 어셈블 → `model_tester_peg`
- ACT 30 epochs 학습 → checkpoint id 9001
- eval bridge 띄우고 evaluator 30 trials → JSON 리포트

## 영향 파일 / 폴더 (Where)

- `backend/tools/model_tester/` (assemble_dataset.py, _run_mini_train.py, tutorial_evaluator.py)
- `ros2/ros2_ws/src/mujoco_world/` (collector, planner, eval_bridge)
- `.claude/commands/model-test.md` (스킬 본문)

코드 변경이 필요하면 그 파일들만 손댄다. 데이터/체크포인트는 모두
`/opt/easytrainer/training_data/` 하위 — 리포 외부.

## Acceptance Criteria

- [ ] collector 50/50 success (motion plan 실패시 추가 시도, max 150)
- [ ] dataset directory contains 50 episodes
  ```bash
  test 50 -eq $(wc -l < /opt/easytrainer/training_data/datasets/model_tester_peg/meta/episodes.jsonl)
  ```
- [ ] checkpoint exists at `9001/` with `model.safetensors`
- [ ] evaluator JSON report exists in `/opt/easytrainer/training_data/model_tester_reports/`
- [ ] `success_rate >= 0.60` in the report (acceptable proxy for "policy learned the task")

> success_rate threshold is **not** a fixed truth — it's a sanity gate.
> If 60% turns out to be unreachable with 50 episodes / 30 epochs we
> downgrade to "any non-zero rate + report saved" and document the
> finding in the PR.

## 비-목표 (Out of Scope)

- Multi-policy A/B (one ACT run is enough for this spec)
- UI changes (수집/평가 모두 CLI)
- Distillation / hyperparameter sweep
- 새로운 시뮬레이션 씬

## 사전 셋업 (one-time, idempotent)

```bash
# 1. tutorial sim + planner 가 떠있어야 함 (도메인 1)
docker exec easytrainer_ros2 bash -lc '
  source /opt/ros/humble/setup.bash &&
  source /root/ros2_ws/install/setup.bash &&
  export ROS_DOMAIN_ID=1 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp &&
  ros2 topic list | grep -q /tutorial/joint_states
' || {
  docker exec -d easytrainer_ros2 bash -lc '
    source /opt/ros/humble/setup.bash &&
    source /root/ros2_ws/install/setup.bash &&
    export ROS_DOMAIN_ID=1 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp &&
    ros2 launch mujoco_world tutorial_pegtask.launch.py
  '
  sleep 10
}

# 2. training_server는 backend 컨테이너에서 /tmp/training_server 로 복사
#    (docker-compose 의 ./backend/training_server 마운트가 dev 환경에서 비어있는
#    경우 대비). _run_mini_train.py 가 이 경로를 fallback 으로 본다.
docker exec easytrainer_backend test -d /tmp/training_server/policies || {
  tar -czf - -C /home/hjhj/EasyTrainer_v2.3.1/training_server . \
    | docker exec -i easytrainer_backend tar -xzf - -C /tmp/training_server
}
```

## 검증 hooks (자동 실행됨)

```bash
set -euo pipefail

DATASET=/opt/easytrainer/training_data/datasets/model_tester_peg
RAW=/opt/easytrainer/training_data/raw_episodes
CKPT_DIR=/opt/easytrainer/training_data/checkpoints/ee252ad4323e056120fb3c8558176e52/9001
REPORTS=/opt/easytrainer/training_data/model_tester_reports

# 1. fresh collect 50 episodes
rm -rf "$RAW"
mkdir -p "$RAW"
docker exec easytrainer_ros2 bash -lc '
  source /opt/ros/humble/setup.bash &&
  source /root/ros2_ws/install/setup.bash &&
  export ROS_DOMAIN_ID=1 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp &&
  ros2 run mujoco_world tutorial_collector \
    --num-episodes 50 --max-attempts 150 \
    --output-dir /opt/easytrainer/training_data/raw_episodes
'
test 50 -eq "$(ls "$RAW" | wc -l)"  # 50 episode dirs

# 2. assemble v2.1 dataset
rm -rf "$DATASET"
docker exec easytrainer_backend bash -lc "
  cd /opt/easytrainer/project &&
  python3 -m backend.tools.model_tester.assemble_dataset \
    --raw-dir $RAW --dataset-dir $DATASET \
    --language 'insert the red peg into the yellow hole'
"
test 50 -eq "$(wc -l < $DATASET/meta/episodes.jsonl)"

# 3. mini-train 30 epochs (resnet18 backbone, chunk_size=10)
rm -rf "$CKPT_DIR"
docker exec easytrainer_backend bash -lc "
  cd /opt/easytrainer/project &&
  python3 -m backend.tools.model_tester._run_mini_train \
    --dataset-dir $DATASET \
    --checkpoint-dir $CKPT_DIR \
    --num-epochs 30 --batch-size 8 --chunk-size 10
"
test -f "$CKPT_DIR/model.safetensors"

# 4. launch eval bridge (idempotent — health check)
docker exec easytrainer_ros2 curl -sf -m 2 http://127.0.0.1:7799/health || {
  docker exec -d easytrainer_ros2 bash -lc '
    source /opt/ros/humble/setup.bash &&
    source /root/ros2_ws/install/setup.bash &&
    export ROS_DOMAIN_ID=1 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp &&
    ros2 run mujoco_world tutorial_eval_bridge --port 7799
  '
  sleep 4
}

# 5. evaluate 30 trials
mkdir -p "$REPORTS"
docker exec easytrainer_backend bash -lc "
  cd /opt/easytrainer/project &&
  python3 -m backend.tools.model_tester.tutorial_evaluator \
    --checkpoint-id 9001 \
    --num-trials 30 --max-steps 400 \
    --bridge-url http://127.0.0.1:7799 \
    --report-dir $REPORTS
"

# 6. acceptance gate
RATE=$(ls -t $REPORTS/ckpt-9001-*.json | head -1 | xargs python3 -c "
import json, sys
print(json.load(open(sys.argv[1]))['success_rate'])
")
echo "[acceptance] success_rate=$RATE"
python3 -c "import sys; sys.exit(0 if float('$RATE') >= 0.60 else 1)"
```

## PR 제목 / 본문

- **제목**: `model-test e2e 검증 — peg-in-hole 50ep / 30 trial 성공률 리포트`
- **본문 추가**:
  - 50 epochs (chunk_size=10, ACT, resnet18) 으로도 60%↑ 가 안 나오면
    `_run_mini_train.py` 의 epoch / chunk_size 를 키워야 할 수 있음
  - 리포트 JSON 경로를 PR description 에 명시
  - `/model-test` 스킬 본문 (`.claude/commands/model-test.md`) 의 디버깅 메모에
    이 spec 의 결과를 한 줄 추가 ("50ep/30ep ACT → N% baseline")

## 참고

- 스킬: [.claude/commands/model-test.md](../../.claude/commands/model-test.md)
- 메모리: `~/.claude/projects/-home-hjhj-EasyTrainer-v2-3-1/memory/model_tester.md`
- 컴포넌트별 검증 결과는 메모리의 "Phase 3 complete" 섹션 참조
