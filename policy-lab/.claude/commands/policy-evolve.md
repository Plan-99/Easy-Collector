---
description: 토너먼트→결과 분석 보고서→다음 후보 설계→재토너먼트 무한 진화 루프
argument-hint: "[--dataset-id 15] [--rounds 0] [--num-trials 5] [--max-steps 200] [--budget-epochs 300] [--target-rate 0.5] [--seed-candidates PATH]"
---

# /policy-evolve — 자율 토너먼트 진화 루프

**목표.** 무인 자율 모드(harness philosophy)에서 모델을 진화시킨다. 매 라운드마다
정책 토너먼트를 돌리고, 결과를 분석해 "왜 안 됐는가"의 진단 보고서를 쓰고, 그
보고서를 입력으로 `policy-researcher` 에이전트를 호출해 **다음 라운드의 후보를
구조적으로 새로 설계**한다. 그리고 즉시 다시 토너먼트를 시작한다.

**사용자가 멈추거나 — 또는 챔피언이 `--target-rate` 를 넘기거나 — `--rounds` 가
다 소진될 때까지 멈추지 않는다.**

이 스킬은 다음과 **다르다**:
- `/policy-design` — 새 정책 *하나* 디자인 (1-shot, 루프 없음).
- `/policy-tournament` — 후보 리스트가 *주어진 상태에서* 1번 토너먼트만 굴림.
- `/policy-evolve` — 위 둘을 묶어 자율적으로 끝까지 가는 메타 루프.

## 인자

| 인자 | 기본값 | 의미 |
|------|--------|------|
| `--dataset-id N`       | `15`     | LeRobot 데이터셋 DB id. |
| `--rounds N`           | `0`      | 0이면 무한 (target-rate 달성 또는 사용자 중단까지). |
| `--num-trials N`       | `5`      | 라운드마다 후보당 평가 trial 수. **빠른 라운드를 위해 작게 시작**, 챔피언 후보가 출현하면 사용자가 별도로 `/policy-tournament` 로 30-trial 재측정. |
| `--max-steps N`        | `200`    | trial 당 최대 정책 step. |
| `--budget-epochs N`    | `300`    | 후보당 학습 epoch. 너무 길면 한 라운드가 안 끝남. |
| `--target-rate R`      | `0.5`    | 챔피언 success_rate가 이 값 이상이면 루프 종료. |
| `--seed-candidates PATH` | (필수) | 첫 라운드 후보 JSON. 보통 baseline + 직전 토너먼트 winner. |
| `--bridge-url URL`     | `http://127.0.0.1:7799` | tutorial_eval_bridge. |
| `--sensor-map STR`     | `sensor_2=top,sensor_3=front` | evaluator sensor 매핑. |
| `--work-dir PATH`      | `/opt/easytrainer/training_data/evolve/<run_id>` | 라운드별 산출물 저장 루트. |

## 사전조건

- backend/ros2 컨테이너 Up + tutorial sim + eval bridge alive (`/smoke --quick` 또는
  `/policy-tournament` 의 step 1 참고).
- `--seed-candidates` 후보가 모두 train_worker.py + tutorial_evaluator.py dispatch에
  등록된 type. 신규 type은 `/policy-design`으로 먼저 등록.
- `policy-researcher` 에이전트 (`.claude/agents/policy-researcher.md`, 랩 안) 가 동작.
- 호스트 스크립트/코드는 EasyTrainer 트리 기준: `EASYTRAINER_ROOT="${EASYTRAINER_ROOT:-$(git rev-parse --show-toplevel)}"`
  (랩이 EasyTrainer 레포 안이라 toplevel = EasyTrainer 루트). smoke 는 `bash "$EASYTRAINER_ROOT/scripts/smoke.sh"` 또는 `/lab-smoke`.
- 라운드 분석 보고서의 **커밋본**은 `policy-lab/docs/evolve/<run>_round_<N>/analysis.md` 에 둔다
  (런타임 산출물은 `/opt/easytrainer/training_data/evolve/<run>/` — 기존과 동일).

## 루프 단일 라운드 절차

각 라운드는 다음 4 phase로 구성. **Claude(메인)가 진행상황을 관리하고, design phase만
sub-agent 호출.**

### Phase A. **Run tournament**
```bash
TOURN_ID="round_${ROUND}_$(date +%H%M)"
docker exec -d easytrainer_backend bash -lc "
  cd /opt/easytrainer/project &&
  python3 -u -m backend.tools.model_tester.tournament run \
    --dataset-id $DATASET_ID \
    --num-trials $NUM_TRIALS --max-steps $MAX_STEPS \
    --bridge-url $BRIDGE_URL --sensor-map '$SENSOR_MAP' \
    --candidates $CANDIDATES_PATH \
    --tournament-id $TOURN_ID > $WORK_DIR/round_${ROUND}/tournament.log 2>&1
"
```

토너먼트 완료까지 wait (백그라운드 task / `ScheduleWakeup`으로 폴링). 후보당 (학습
~1–2시간 + 평가 ~수분) × N 후보 만큼 걸린다.

### Phase B. **Write analysis report** (메인 Claude가 직접)

`$WORK_DIR/round_${ROUND}/analysis.md` 작성. **반드시 포함**:

1. **표** — 후보별 (success_rate, mean steps to done, train_time, train_loss 최종,
   val_loss 최종, early_done 비율, xy_err mean, tilt_cos mean).
2. **승자 분석** — winner가 무엇을 잘했나, baseline 대비 어디가 더 좋아졌나.
3. **실패 모드 분석** — 모든 trial의 last state를 봐서 어디서 막혔나:
   - peg가 hole에서 N cm 떨어져 있음 (vision 인식 약함?)
   - peg를 잡지 못함 (gripper timing?)
   - peg를 뽑아서 멀리 던짐 (action chunk 길이 부적절?)
   - 처음부터 안 움직임 (no-proprio 셋업에서 초기 visual cue 부족?)
4. **다음 라운드 가설** — 무엇을 구조적으로 바꾸면 이 실패 모드가 풀릴까. 1-2개 후보.

### Phase C. **Design next-round candidates** (sub-agent 호출)

`policy-researcher` 에이전트에게 다음을 위임. **메인 Claude는 분석 보고서를 그대로
입력으로 전달**:

```
Agent({
  description: "Design next-round candidates for evolve loop",
  subagent_type: "policy-researcher",
  prompt: """
    이전 라운드 결과 분석 보고서: $WORK_DIR/round_${ROUND}/analysis.md

    임무: 위 보고서의 "다음 라운드 가설" 섹션을 근거로 새 후보 2개를 설계해라.
    - 단순 hyperparameter 튜닝 금지. 구조 변형이어야 함.
    - 이전 라운드 winner의 정책 type을 **유지**하면서 더 깊은 변형 (e.g. flow head 위에
      cross-attn 추가), 또는 **완전히 다른 type** (e.g. DPDino → FlowMatchDino → ManiDiTMicro)
      각 1개씩 제출.
    - WebSearch로 2025–2026 arXiv 추가 reference 확인.
    - 신규 type이면 정책 구현 + train_worker dispatch + evaluator dispatch + DB/UI 등록
      모두 끝낸다 (agent 정의의 6단계 레시피).
    - 1-epoch smoke test 통과 후, candidates JSON 스니펫 2개를 stdout으로 출력.

    이전 라운드 winner와 새 후보 2개 = 다음 토너먼트 entrant 3명.
  """
})
```

Agent가 반환한 candidates JSON을 받아 다음 라운드 `$CANDIDATES_PATH` 로 저장:
```bash
# winner + agent가 만든 후보 2개 = 3 entrants
WINNER_JSON=$(jq '.winner' $WORK_DIR/round_${ROUND}/summary.json | jq 'del(.checkpoint_id) | del(.policy_id) | del(.report_path) | del(.train_exit) | del(.train_time_sec) | del(.evaluated) | del(.success_rate) | del(.num_success) | del(.num_trials)')
jq -s 'add' <(echo "[$WINNER_JSON]") <(echo "$AGENT_OUTPUT_CANDIDATES") \
  > $WORK_DIR/round_$((ROUND+1))/candidates.json
```

### Phase D. **Loop or stop**

종료 조건 체크:
- `winner.success_rate >= --target-rate` → loop 종료, champion 갱신.
- `--rounds > 0 && current_round >= --rounds` → loop 종료.
- 사용자가 명시적으로 중단 → loop 종료.

아니면 Phase A로 돌아간다 (`ROUND++`, 새 candidates.json).

## 챔피언 갱신 정책

매 라운드마다 자동으로:
```bash
docker exec easytrainer_backend bash -lc "
  cd /opt/easytrainer/project &&
  python3 -m backend.tools.model_tester.tournament declare-champion \
    --tournament-dir $WORK_DIR/round_${ROUND}
"
```
strictly-higher rule이 적용되므로 progress 안 된 라운드는 그냥 무시된다.

## 산출물 디렉토리 구조

```
/opt/easytrainer/training_data/evolve/<run_id>/
├── meta.json                ← 시작 인자, 시드 후보, target-rate
├── round_0/
│   ├── candidates.json
│   ├── tournament.log
│   ├── summary.json         ← tournament가 만들어줌
│   ├── analysis.md          ← Phase B 산출물
│   └── <00_name>/...        ← 각 후보 train.log/eval.log
├── round_1/
│   ├── candidates.json      ← winner + agent가 만든 신규 2명
│   ├── ...
└── final_report.md          ← 루프 종료 시 메인 Claude가 작성
```

## 최종 보고 (Claude → 사용자)

```
evolve_run:    <run_id>
rounds_done:   <R>
candidates_evaluated: <total>
champion_history:
  - round_0: <type/name> — K/M (R%)
  - round_1: <type/name> — K/M (R%)
  - ...
final_champion: <type/name> — K/M (R%)
champion_path: /opt/easytrainer/training_data/checkpoints/.../<id>/
arxiv_used:    <리스트>
```

## 디버깅

- **라운드가 너무 오래 걸려서 사용자가 깨워봐도 한 라운드도 못 끝냄.**
  `--budget-epochs` 를 줄이거나 (예 300→100), `--num-trials` 를 줄임 (5→3). 진화의
  목적은 trends 잡기 — single-shot 정확도가 아님.
- **챔피언이 라운드 N에서 1번 뽑힌 뒤 라운드 N+1, N+2가 모두 미달.**
  정상 — strictly-higher rule. 진화는 계속 시도하고, 첫 비기는 라운드는 그냥 winner만
  바뀜 (champion 파일은 안 바뀜).
- **agent가 같은 구조 변형을 반복 제안.**
  prompt에 "이미 시도한 type list = [...]" 추가해 회피. analysis.md 에 시도한 type을
  헤더로 명시.
- **새 type 등록이 실패해서 라운드 진행 불가.**
  Phase C agent 출력에서 smoke_test=failed 이면 그 후보를 빼고 winner+1만으로 진행.
  최악의 경우 winner 1명만으로 다음 라운드 = 그냥 재학습.
- **smoke로 평가 bridge 죽어있음.** `/policy-tournament` 의 step 1을 재실행.

## 관련

- 단일 토너먼트: [.claude/commands/policy-tournament.md](policy-tournament.md)
- 단일 정책 디자인: [.claude/commands/policy-design.md](policy-design.md)
- 정책 리서치 에이전트: [.claude/agents/policy-researcher.md](../agents/policy-researcher.md)
- 모델 테스트 (단일, EasyTrainer 측): [model-test.md](../../../.claude/commands/model-test.md)
- 야간 자율 진입점: [lab-overnight.md](lab-overnight.md)
- 랩 원칙/채점: [golden-principles.md](../../golden-principles.md), [rubric.md](../../rubric.md)
- harness 철학: `~/.claude/projects/-home-hjhj-EasyTrainer-v2-3-1/memory/harness_philosophy.md`
