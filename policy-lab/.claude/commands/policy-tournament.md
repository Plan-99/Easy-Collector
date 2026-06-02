---
description: 정책 후보 N개를 같은 데이터셋·같은 에폭으로 학습·평가해 챔피언을 갱신
argument-hint: "[--dataset-id 15] [--num-trials 30] [--max-steps 400] [--candidates PATH] [--bridge-url URL]"
---

# /policy-tournament — 정책 토너먼트

**목표.** N개의 정책 후보(베이스라인 ACT + 변형 ACT + Diffusion + `/policy-design`이
만든 새 정책 등)를 **같은 데이터셋·같은 에폭·같은 평가 trial 수**로 굴려, 평가
성공률이 가장 높은 후보를 챔피언으로 선언한다. **현재 챔피언보다 높을 때만**
챔피언 파일을 갱신한다.

이 스킬은 `/model-test` 와 다르다:
- `/model-test` 는 데이터 수집 + 모델 1개 평가 (단일 시도).
- `/policy-tournament` 는 데이터가 **이미 있다**고 가정하고 (e.g. Dataset 15),
  **여러 정책**을 같은 조건에서 비교한다.

## 인자

| 인자 | 기본값 | 의미 |
|------|--------|------|
| `--dataset-id N`       | `15`              | LeRobot 데이터셋 DB id. `tutorial_peg_in_hole` 시드 (Task 13) 와 sensor_ids 가 일치해야 함. |
| `--candidates PATH`    | `(필수)`          | 후보 recipe JSON. 아래 "candidates.json 스키마" 참조. |
| `--num-trials N`       | `30`              | 후보당 평가 trial 수. |
| `--max-steps N`        | `400`             | trial 당 최대 정책 step. |
| `--bridge-url URL`     | `http://127.0.0.1:7799` | tutorial_eval_bridge 주소. |
| `--sensor-map STR`     | `sensor_2=top,sensor_3=front` | evaluator sensor 매핑. |
| `--tournament-id STR`  | `tourn_<ts>`      | 결과를 저장할 디렉토리 이름. |
| `--declare-champion`   | off               | 토너먼트 끝나면 챔피언 파일과 비교해 자동 갱신. |
| `--force-champion`     | off               | 새 winner가 챔피언보다 낮아도 강제로 chevron 등록 (디버그용). |

## candidates.json 스키마

```json
[
  {
    "name": "act_baseline",
    "policy_type": "ACT",
    "policy_settings": {
      "chunk_size": 30, "kl_weight": 10.0, "vision_backbone": "resnet18",
      "hidden_dim": 256, "dim_feedforward": 2048,
      "enc_layers": 4, "dec_layers": 6, "nheads": 8,
      "position_embedding": "sine",
      "temporal_ensemble_coeff": 0.01
    },
    "train_settings": {
      "num_epochs": 1000, "batch_size": 8,
      "image_resolution": [128, 128],
      "optimizer_lr": 1e-5, "optimizer_lr_backbone": 1e-5,
      "optimizer_weight_decay": 1e-4, "dropout": 0.1,
      "use_amp": false, "has_succeed": true
    }
  },
  {
    "name": "act_chunk60",
    "policy_type": "ACT",
    "policy_settings": { "chunk_size": 60, "kl_weight": 20.0, ... },
    "train_settings": { "num_epochs": 1000, "batch_size": 8, ... }
  },
  {
    "name": "diffusion_baseline",
    "policy_type": "Diffusion",
    "policy_settings": { "timesteps": 100, "noise_schedule": "cosine" },
    "train_settings": { "num_epochs": 1000, "batch_size": 8, ... }
  }
]
```

`name` 은 unique. Policy DB row 가 자동으로 생성/갱신된다.

## 사전조건

- backend/ros2 컨테이너 Up. (`/smoke --quick`)
- Dataset (default 15) 가 `/opt/easytrainer/datasets/<id>/` 에 존재하고 LeRobot v2.1 포맷.
- 튜토리얼 sim + eval bridge 가 ros2 컨테이너에서 실행 중.
- 새 정책 type 이 후보에 있으면 `/policy-design` 으로 사전에 등록되어 있어야 함.

## 절차

### 0) Baseline smoke + 컨테이너 환경 확인
```bash
# 랩은 EasyTrainer 레포 안 → toplevel = EasyTrainer 루트 (호스트 무관).
EASYTRAINER_ROOT="${EASYTRAINER_ROOT:-$(git rev-parse --show-toplevel)}"
bash "$EASYTRAINER_ROOT/scripts/smoke.sh" --quick   # 또는 /lab-smoke (sim+bridge 포함)
```

### 1) sim + eval bridge 기동 (이미 떠 있으면 skip)
```bash
docker exec easytrainer_ros2 bash -lc "ros2 topic list 2>&1 | grep -q /tutorial/joint_states" \
  || docker exec -d easytrainer_ros2 bash -lc '
    source /opt/ros/humble/setup.bash &&
    source /root/ros2_ws/install/setup.bash &&
    export ROS_DOMAIN_ID=1 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp &&
    ros2 launch mujoco_world tutorial_pegtask.launch.py'

docker exec easytrainer_ros2 curl -s http://127.0.0.1:7799/health \
  | grep -q ok \
  || docker exec -d easytrainer_ros2 bash -lc '
    source /opt/ros/humble/setup.bash &&
    source /root/ros2_ws/install/setup.bash &&
    export ROS_DOMAIN_ID=1 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp &&
    export PYTHONPATH=/opt/easytrainer/project/ros2/ros2_ws/src/mujoco_world:$PYTHONPATH &&
    python3 -u -m mujoco_world.tutorial_eval_bridge --port 7799'
```

### 2) candidates 파일 빌드 (사용자가 직접 또는 Claude가)

Claude가 만드는 경우 `/tmp/candidates.json` 에 쓰면 충분.

### 3) 토너먼트 실행 — `--candidates` 가 곧 학습 큐

각 후보는 (학습 + 평가)를 순서대로 진행. 1000-epoch 학습은 GPU 시간 약 5h/모델
이므로 후보가 N개면 ~5N 시간을 본다.

```bash
docker exec -d easytrainer_backend bash -lc "
  cd /opt/easytrainer/project &&
  python3 -u -m backend.tools.model_tester.tournament run \
    --dataset-id $DATASET_ID \
    --candidates /tmp/candidates.json \
    --num-trials $NUM_TRIALS \
    --max-steps $MAX_STEPS \
    --bridge-url $BRIDGE_URL \
    --sensor-map '$SENSOR_MAP' \
    > /tmp/tournament.log 2>&1; echo \$? > /tmp/tournament.exit
"
```

진행상황은 Monitor 로 tail. 후보당 끝나면 `[tournament] candidate <name> ⇒ K/M (R%)`
한 줄이 찍힌다.

### 4) 결과 보기

```bash
docker exec easytrainer_backend bash -lc "
  tail -80 /tmp/tournament.log;
  ls /opt/easytrainer/training_data/tournaments/ | tail -3
"

TOURN_DIR=$(docker exec easytrainer_backend bash -lc \
  "ls -td /opt/easytrainer/training_data/tournaments/*/ | head -1")
docker exec easytrainer_backend bash -lc \
  "python3 -c 'import json;d=json.load(open(\"$TOURN_DIR/summary.json\"));print(json.dumps(d[\"candidates\"],indent=2));print(\"winner=\",d.get(\"winner\"))'"
```

### 5) 챔피언 선언 (`--declare-champion` 또는 수동)

```bash
docker exec easytrainer_backend bash -lc "
  cd /opt/easytrainer/project &&
  python3 -m backend.tools.model_tester.tournament declare-champion \
    --tournament-dir $TOURN_DIR
"
```

업데이트 규칙:
- 새 winner의 `success_rate` 가 **현재 챔피언 success_rate 보다 strictly 큼** 일 때만 갱신.
- 기존 챔피언은 `model_tester_champion.json.bak.<ts>` 로 백업.
- 동률이면 갱신하지 않음 (정상 동작; `--force` 로 무시 가능).
- 챔피언 파일 위치: `/opt/easytrainer/training_data/model_tester_champion.json`.

이 파일이 EasyTrainer 내에서 "현재 정밀조작 챔피언" 의 단일 정보원이 된다. UI Policy
페이지에서 winner의 `policy_id` 가 사용자에게 추천 행으로 노출되도록 별도 작업이
필요하다면 그건 frontend 측 follow-up.

## 최종 보고 (Claude → 사용자)

```
tournament:   <tourn_id>
dataset:      <id> (<name>, <N> episodes)
candidates:   <C>
results:
  - <name> (<type>): K/M (R%) train=<H:MM>
  - ...
winner:       <name> (<type>) — <K>/<M> (<R%>)
champion:     unchanged (<existing R%>) | UPDATED (was <X%> → now <R%>)
```

## 디버깅

- **모든 후보가 0% 였다.** 데이터셋이 부족하거나 max_steps 가 짧다. epoch을 더 올리거나
  데이터 수집을 추가. 새 후보에서도 0%면 평가 bridge → sim 의 publish rate를 의심.
- **non-ACT 후보 학습이 즉시 죽음.** 새 정책 type 이 train_worker dispatch 에 없거나
  policy_settings 키가 Config 가 기대하는 이름과 다름. `/policy-design --no-smoke`
  로 import만 검증해 디자인 문제와 학습 문제를 분리.
- **eval bridge 가 죽어있음.** evaluator 에러 `bridge unhealthy at <url>`.
  step 1을 다시 돌리고 토너먼트만 재시도 (이미 학습된 checkpoint는 재사용되지 않음 —
  토너먼트는 매번 새 Checkpoint row 를 만든다).
- **챔피언 갱신이 안 됨.** `declare-champion` 이 strictly-higher 만 받음. `--force` 로 회피.

## 자율 진화 모드

토너먼트 한 번이 아니라 **여러 라운드 자율 진화** (토너먼트→결과분석→다음 후보
설계→재토너먼트) 가 필요하면 `/policy-evolve` 를 쓴다. 이 스킬은 그 루프의
*한 라운드*에 해당.

## 관련

- 토너먼트 헬퍼: [tournament.py](../../../backend/tools/model_tester/tournament.py) (EasyTrainer 트리)
- 후보 정책 디자인: [.claude/commands/policy-design.md](policy-design.md)
- 자율 진화 루프: [.claude/commands/policy-evolve.md](policy-evolve.md)
- 정책 리서치 에이전트: [.claude/agents/policy-researcher.md](../agents/policy-researcher.md)
- 모델테스트(단일, EasyTrainer 측): [model-test.md](../../../.claude/commands/model-test.md)
