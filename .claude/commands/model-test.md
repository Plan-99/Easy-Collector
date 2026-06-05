---
description: 튜토리얼 peg-in-hole 태스크에서 정책을 end-to-end로 자동 평가 (데이터 수집 → 학습 → 시뮬레이션 평가, 성공률 출력)
argument-hint: "[--policy-id N] [--checkpoint-id N] [--num-episodes 30] [--num-trials 20] [--dataset-name model_tester_peg]"
---

# /model-test — Tutorial Peg-in-Hole 자동 벤치마크

**목표.** 사용자가 한 줄 입력으로 "튜토리얼 월드에서 이 정책 설정의 성공률" 을
얻는 것. 사람이 시뮬레이터를 만지거나 UI 를 띄울 필요가 없다. ground-truth
체크가 데이터 필터 + 추론 평가 양쪽에 모두 쓰인다.

## 인자 정리

| 인자 | 기본값 | 의미 |
|------|--------|------|
| `--policy-id N`            | 없음 (필수, `--checkpoint-id` 미지정 시) | 학습에 쓸 Policy DB id |
| `--checkpoint-id N`        | 없음 | 이미 학습된 체크포인트가 있으면 학습 단계 스킵 |
| `--num-episodes N`         | `30` | 수집할 *성공* 에피소드 수 |
| `--num-trials N`           | `20` | 평가 trial 수 |
| `--max-steps N`            | `400` | trial당 최대 정책 step |
| `--dataset-name STR`       | `model_tester_peg` | LeRobot 데이터셋 이름 |
| `--task-name STR`          | `model_tester_peg` | DB Task name |
| `--language STR`           | `"insert the red peg into the yellow hole"` | 데이터셋 language instruction |
| `--training-server-url URL`| `http://127.0.0.1:5100` | training_server 주소 |
| `--skip-data`              | off | 기존 raw_episodes 재사용 |
| `--skip-train`             | off | 데이터만 만들고 학습은 큐에 넣지 않음 |

`--checkpoint-id` 가 주어지면 **데이터/학습 모두 스킵**하고 곧장 평가만.

## 사전조건

- 컨테이너: `easytrainer_backend`, `easytrainer_ros2`, `easytrainer_frontend`
  모두 Up. `/smoke --quick` 으로 확인.
- ROS 도메인: 튜토리얼 sim 은 **`ROS_DOMAIN_ID=1`** + `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`
  로 띄운다. 모든 `docker exec easytrainer_ros2 ros2 ...` 호출 시 이 env 를 함께 export.
- 시뮬: [tutorial_pegtask.launch.py](../../ros2/ros2_ws/src/mujoco_world/launch/tutorial_pegtask.launch.py).
  서비스 목록에 `/tutorial/randomize`, `/tutorial/run_episode`,
  `/tutorial/check_success` 가 보여야 한다.
- training_server: backend 컨테이너의 `/root/backend/training_server/` 가
  비어있는 경우 (dev 환경 docker-compose bind 미스매치) `/tmp/training_server/`
  로 한 번 복사. `_run_mini_train.py` 가 이 fallback 경로를 본다.
  ```bash
  docker exec easytrainer_backend test -d /tmp/training_server/policies || {
    tar -czf - -C training_server . \
      | docker exec -i easytrainer_backend tar -xzf - -C /tmp/training_server
  }
  ```

## 실행 순서 (Claude가 직접 Bash로 진행)

### 0) Baseline smoke
```bash
bash scripts/smoke.sh --quick
```
exit code ≠ 0 이면 인프라 문제로 보고하고 **중단**.

### 1) 시뮬 + 플래너 기동
```bash
docker exec -d easytrainer_ros2 bash -lc '
  source /opt/ros/humble/setup.bash &&
  source /root/ros2_ws/install/setup.bash &&
  export ROS_DOMAIN_ID=1 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp &&
  ros2 launch mujoco_world tutorial_pegtask.launch.py
'
# `/tutorial/joint_states`, `/tutorial/check_success`, `/tutorial/randomize` 가
# 모두 보일 때까지 ros2 topic list 폴링 (~5s).
```

### 2) 데이터 수집 (`--skip-data` 또는 `--checkpoint-id` 미지정 시)

```bash
docker exec easytrainer_ros2 bash -lc '
  source /opt/ros/humble/setup.bash &&
  source /root/ros2_ws/install/setup.bash &&
  export ROS_DOMAIN_ID=1 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp &&
  ros2 run mujoco_world tutorial_collector \
      --num-episodes $NUM_EPISODES \
      --output-dir /opt/easytrainer/training_data/raw_episodes
'
```

- 콜렉터는 randomize → run_episode → check_success 를 반복.
- ground-truth success 인 에피소드만 디스크에 저장.
- 실패한 trial 은 attempts 카운터만 늘림.

### 3) **DB 등록 (UI 노출)** — assembly/train 전에 반드시 실행

스킬은 데이터셋/체크포인트 **모두 UI에서 보이게** 해야 한다 (사용자 요구).
DB 행을 미리 만들어 id 를 받아두면, 이후 단계가 그 id 로 디렉토리를
생성하므로 record_episode/UI 와 동일한 경로 규약을 따르게 된다.

```bash
DATASET_NAME=model_tester_peg_$(date +%s)
docker exec easytrainer_backend bash -lc "
  cd /opt/easytrainer/project &&
  python3 -m backend.tools.model_tester.register_run begin \
      --dataset-name $DATASET_NAME \
      --num-episodes $NUM_EPISODES --num-epochs 30 \
      --batch-size 8 --chunk-size 10 \
      --sensor-ids 2,3,4 \
      --json-out /tmp/model_test_run.json
" | tee /tmp/host_run.json
# Returned JSON: {task_id, dataset_id, dataset_dir, policy_id, checkpoint_id, checkpoint_dir, sensor_ids, machine_id}
```

이 시점부터 UI 의 Dataset 페이지에 빈 Dataset row 가 즉시 나타난다.
Checkpoint 페이지에는 status='queued' 인 행이 뜬다.

### 4) LeRobot v2.1 데이터셋 어셈블 (backend 컨테이너)

`register_run begin` 이 반환한 `dataset_dir` (= `${DATASET_DIR}/<dataset_id>`)
에 곧장 쓴다. UI 의 Dataset/<id> 페이지가 이 경로의 episodes.jsonl 을 읽는다.

```bash
DATASET_DIR=$(jq -r .dataset_dir /tmp/model_test_run.json)
docker exec easytrainer_backend bash -lc "
  cd /opt/easytrainer/project &&
  python3 -m backend.tools.model_tester.assemble_dataset \
      --raw-dir /opt/easytrainer/training_data/raw_episodes \
      --dataset-dir $DATASET_DIR \
      --cameras 2:top_cam,3:front_cam,4:wrist_cam \
      --language '$LANGUAGE'
"
```

생성되는 features: `observation.images.sensor_2` (top) / `sensor_3` (front) /
`sensor_4` (wrist). 이는 `tutorial_peg_in_hole` Task 의
`sensor_ids=[2,3,4]` 시드와 일치하므로 LeRobot 이 sensor row 와 dataset
feature 를 1:1 로 매칭한다.

### 5) 학습 — DB-aware 경로 (재 통합됨)

DB 행은 step 3 에서 이미 만들어졌으므로 `_run_mini_train.py` 가
`register_run begin` 이 알려준 `checkpoint_dir` 에 쓰면 자동으로 UI 가
인식한다 (`resolve_checkpoint_dir(<ckpt_id>)` 가 그 경로를 찾는다).

```bash
CKPT_ID=$(jq -r .checkpoint_id /tmp/model_test_run.json)
CKPT_DIR=$(jq -r .checkpoint_dir /tmp/model_test_run.json)
docker exec easytrainer_backend bash -lc "
  cd /opt/easytrainer/project &&
  python3 -m backend.tools.model_tester.register_run start --checkpoint-id $CKPT_ID
"
docker exec easytrainer_backend bash -lc "
  cd /opt/easytrainer/project &&
  python3 -m backend.tools.model_tester._run_mini_train \
      --dataset-dir $DATASET_DIR \
      --checkpoint-dir $CKPT_DIR \
      --sensor-ids 2,3,4 \
      --num-epochs 30 --batch-size 8 --chunk-size 10
" && docker exec easytrainer_backend bash -lc "
  cd /opt/easytrainer/project &&
  python3 -m backend.tools.model_tester.register_run finalize --checkpoint-id $CKPT_ID
" || docker exec easytrainer_backend bash -lc "
  cd /opt/easytrainer/project &&
  python3 -m backend.tools.model_tester.register_run fail --checkpoint-id $CKPT_ID
"
```

학습 진행률은 stdout 으로만 흐르고 UI 로 stream 되지는 않지만, finalize 가
끝나는 즉시 UI 의 Checkpoint 행이 `running → finished` 로 바뀐다.

### 6) 평가 브릿지 기동 (ros2 컨테이너)

```bash
docker exec -d easytrainer_ros2 bash -lc '
  source /opt/ros/humble/setup.bash &&
  source /root/ros2_ws/install/setup.bash &&
  export ROS_DOMAIN_ID=1 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp &&
  ros2 run mujoco_world tutorial_eval_bridge --port 7799
'
# 헬스체크 (컨테이너 내부에서)
docker exec easytrainer_ros2 curl -s http://127.0.0.1:7799/health
# → {"ok": true}
```

### 7) 평가 (backend 컨테이너)

`tutorial_peg_in_hole` Task 의 sensor 매핑
(sensor_2=top, sensor_3=front, sensor_4=wrist) 은 sensor-map 으로 명시.
어셈블러가 sensor_2/sensor_3/sensor_4 으로 video feature 를 기록했으므로
evaluator 의 positional fallback 도 동작하지만, 명시하면 실수가 없다.

```bash
docker exec easytrainer_backend bash -lc "
  cd /opt/easytrainer/project &&
  python3 -m backend.tools.model_tester.tutorial_evaluator \
      --checkpoint-id $CKPT_ID \
      --num-trials $NUM_TRIALS \
      --max-steps $MAX_STEPS \
      --bridge-url http://127.0.0.1:7799 \
      --sensor-map sensor_2=top,sensor_3=front,sensor_4=wrist
"
```

평가가 끝나면 JSON 리포트가
`/opt/easytrainer/training_data/model_tester_reports/ckpt-<id>-XXXXXXXX.json`
에 떨어진다. stdout 마지막 줄에 success rate.

### 8) 정리

```bash
docker exec easytrainer_ros2 curl -s -X POST http://127.0.0.1:7799/shutdown || true
docker exec easytrainer_ros2 pkill -f tutorial_pegtask.launch.py || true
```

## 최종 보고 (Claude → 사용자)

```
checkpoint: <id>
dataset:    <name> (<N> episodes)
training:   finished in <H:MM>
evaluation: <K>/<M> success (<rate%>)
report:     /opt/easytrainer/training_data/model_tester_reports/<file>
```

실패 trial 이 있으면 reasons (`policy_error`, `ground_truth_fail`) 분포 요약.

## 디버깅 메모

- **planner 가 잡지 못함 → grip force=0.** `tutorial_planner_node` 의
  `gripper_closed=0.000`, 핑거 inner face 가 y=±0.007 (페그 반폭 7mm) 인지
  확인. memory: `model_tester.md` 의 "Working components" 참고.
- **수집된 에피소드가 너무 짧다 (<200 frames).** 카메라 QoS 가
  BEST_EFFORT 인데 콜렉터가 RELIABLE 로 구독하면 모든 프레임이 drop
  된다. `tutorial_collector_node.py` 의 `img_qos` 확인.
- **평가 success rate ≈ 0 인데 학습 로스는 낮음.** input pipeline
  미스매치 가능성. `diagnose_inference.py` 로 학습 프레임에서의
  per-joint norm err 가 0.3 미만인지 먼저 확인.
- **/step latency 가 너무 길다.** 브릿지는 next state 도착까지 0.5s
  대기. 시뮬 publish rate (`mujoco_world_node`) 를 확인.

## 관련 코드

- 시뮬: [ros2/ros2_ws/src/mujoco_world/](../../ros2/ros2_ws/src/mujoco_world/)
- 콜렉터: [tutorial_collector_node.py](../../ros2/ros2_ws/src/mujoco_world/mujoco_world/tutorial_collector_node.py)
- 어셈블러: [assemble_dataset.py](../../backend/tools/model_tester/assemble_dataset.py)
- 평가 브릿지: [tutorial_eval_bridge.py](../../ros2/ros2_ws/src/mujoco_world/mujoco_world/tutorial_eval_bridge.py)
- 평가기: [tutorial_evaluator.py](../../backend/tools/model_tester/tutorial_evaluator.py)
- 기억: `~/.claude/projects/-home-hjhj-EasyTrainer-v2-3-1/memory/model_tester.md`
