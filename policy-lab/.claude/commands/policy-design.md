---
description: policy-researcher agent를 호출해 새 정책을 설계·구현·EasyTrainer 등록까지 수행
argument-hint: "<TypeName> [--goal STR] [--baseline-report PATH] [--budget-epochs N] [--no-smoke]"
---

# /policy-design — 새 정책 토너먼트 후보 만들기

**목적.** ACT/Diffusion 베이스라인 성능이 0%/낮음일 때, **새 정책 타입**을 디자인해
EasyTrainer 정책 목록에 추가하고, 그 다음 `/policy-tournament` 가 그것을 학습/평가
후보로 채택할 수 있게 한다. lerobot 코드는 절대 직접 수정하지 않는다.

## 인자

| 인자 | 의미 |
|------|------|
| `<TypeName>` *(필수)* | 정책 type 식별자. PascalCase. e.g. `ACTPlus`. |
| `--goal STR`           | 한 줄 문제 정의. 기본: "튜토리얼 peg-in-hole(Dataset 15, 53ep)에서 ACT 베이스라인을 능가". |
| `--baseline-report PATH` | 비교할 baseline 평가 JSON. 기본: `/opt/easytrainer/training_data/model_tester_reports/`에서 가장 최근 ACT 리포트 자동 선택. |
| `--budget-epochs N`     | UI 디폴트 epoch 수. 기본 `1000`. |
| `--no-smoke`            | 1-epoch smoke test 생략 (디자인만 검증할 때). |

## 사전조건

- backend/ros2 컨테이너가 Up. (`/smoke --quick`)
- Dataset 15 (또는 baseline 학습이 끝난 다른 dataset) 가 `/opt/easytrainer/datasets/`에 있다.
- `python3 -m backend.tools.model_tester.register_run --help` 가 동작한다.

## 절차 (Claude → policy-researcher agent → 결과 검증)

### 1) Agent 호출

```
Agent({
  description: "Design policy <TypeName>",
  subagent_type: "policy-researcher",
  prompt: """
    name=<TypeName>
    goal=<--goal 값 또는 기본>
    constraints=
      - VRAM ≤ 12GB (batch_size 8 기본)
      - 학습 1000 epochs, GPU 단일
      - 데이터 53 episodes (small-data 정밀조작)
      - lerobot 트리는 read-only, training_server/policies/{type_lower}/ 에 새로 짜기
    baseline_report=<--baseline-report 값>

    설계 후 다음을 반드시 수행:
      a. training_server/policies/{type_lower}/ 작성
      b. training_server/train_worker.py dispatch elif 추가
      c. backend/policies/utils.py make_easytrainer_processors 분기 추가 (필요시)
      d. backend/database/models/policy_model.py POLICY_CONFIGS 항목 추가
      e. frontend/src/configs/modelConfigs.js POLICY_CONFIGS + TRAIN_CONFIGS 항목 추가

    smoke-test: --no-smoke 가 아니면 1-epoch run 으로 _run_mini_train 통과 여부 확인.
    JSON 한 줄로 최종 보고. 형식은 agent 정의 참조.
  """
})
```

### 2) Agent 산출물 검증

Agent 의 JSON 응답에서 다음 4가지를 자동 검증:

```bash
# 모든 코드 경로는 EasyTrainer 트리 기준. 랩은 같은 git 레포 안이라 toplevel = EasyTrainer 루트.
EASYTRAINER_ROOT="${EASYTRAINER_ROOT:-$(git rev-parse --show-toplevel)}"
# 1) 정책 파일이 실제로 생겼는지
TYPE_LOWER=$(echo "<TypeName>" | tr A-Z a-z)
test -f "$EASYTRAINER_ROOT/training_server/policies/${TYPE_LOWER}/configuration_${TYPE_LOWER}.py"
test -f "$EASYTRAINER_ROOT/training_server/policies/${TYPE_LOWER}/modeling_${TYPE_LOWER}.py"

# 2) train_worker.py dispatch에 분기가 추가되었는지
grep -q "policy_obj\['type'\] == '<TypeName>'" "$EASYTRAINER_ROOT/training_server/train_worker.py"

# 3) DB POLICY_CONFIGS에 등록되었는지
grep -q "'<TypeName>'" "$EASYTRAINER_ROOT/backend/database/models/policy_model.py"

# 4) UI modelConfigs.js에 등록되었는지
grep -q "<TypeName>" "$EASYTRAINER_ROOT/frontend/src/configs/modelConfigs.js"
```

4개 중 하나라도 실패하면 사용자에게 보고하고 **중단**. agent를 재호출하거나 사람이 마저.

### 3) quick_apply (런타임 반영)

```bash
bash "$EASYTRAINER_ROOT/scripts/quick_apply.sh" "$EASYTRAINER_ROOT/" /opt/easytrainer/project
```

### 4) DB 행 prefab

UI에서 사용자가 직접 "이 정책으로 학습" 누를 수 있도록 Policy DB row 하나를 미리 만든다:

```bash
docker exec easytrainer_backend bash -lc "
  cd /opt/easytrainer/project &&
  python3 -c '
import json
from backend.database.config.database import db
from backend.database.models.policy_model import Policy, POLICY_CONFIGS
db.connect(reuse_if_open=True)
p = Policy.create(
    name=\"<TypeName>_default\",
    type=\"<TypeName>\",
    batch_size=\"8\",
    num_epochs=\"<--budget-epochs>\",
    settings=POLICY_CONFIGS[\"<TypeName>\"],
)
print(json.dumps({\"policy_id\": p.id, \"type\": p.type, \"name\": p.name}))
'
"
```

이 row id가 곧 `/policy-tournament` 의 `--candidate-policy-ids` 인자로 들어갈 값.

### 5) 최종 보고

```
type:            <TypeName>
policy_class:    training_server/policies/<lower>/modeling_<lower>.py:<Class>Policy
config_class:    training_server/policies/<lower>/configuration_<lower>.py:<Class>Config
db_policy_id:    <N>
smoke_test:      passed | skipped | failed:<reason>
ui_visible:      yes (frontend modelConfigs.js 갱신됨, 새로고침 후 TrainPage Step 2 type 셀렉터에서 보임)
expected_lift:   <agent가 추정한 값>
```

## 디버깅

- **smoke-test 실패 (missing key in batch).** 보통 새 정책의 forward()가 dataset에
  존재하지 않는 key를 참조. agent가 `observation.images.sensor_2/3` + `observation.qpos`
  + `action.joint` 만 보도록 다시 설계하게 함.
- **OOM.** chunk_size 또는 hidden_dim 을 절반으로. agent에게 `--budget-vram 8GB` 같은
  더 빡빡한 constraint를 주고 재호출.
- **frontend 변경이 UI 에서 안 보임.** Vite HMR이 뻑났을 가능성. 사용자가 브라우저
  Ctrl+F5. 또는 `docker exec easytrainer_frontend bash -lc 'kill -1 $(pidof node)'`.

## 관련

- agent: [.claude/agents/policy-researcher.md](../agents/policy-researcher.md)
- 토너먼트: [.claude/commands/policy-tournament.md](policy-tournament.md)
- 모델테스트(EasyTrainer 측): [model-test.md](../../../.claude/commands/model-test.md)
