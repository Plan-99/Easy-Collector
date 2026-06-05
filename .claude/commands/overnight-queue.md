---
description: docs/queue/ 의 spec들을 순서대로 처리. 자율 야간 운영 진입점.
argument-hint: "[--max-cycles N] [--time-budget Mh]  (예: --max-cycles 6 --time-budget 6h)"
---

야간 자율 운영의 **단일 진입점**. `docs/queue/*.md` (TEMPLATE/README 제외)를 순서대로
픽업해 `/feature-from-spec`을 spec마다 호출합니다. 사람이 자는 동안 6시간 굴리는 게 표준 시나리오.

인자: `$ARGUMENTS` (선택)
- `--max-cycles N`  최대 처리 spec 수 (기본 999)
- `--time-budget Xh` 총 시간 예산. 초과 직전 사이클 시작은 막음 (기본 6h)

---

## 0. 초기화

```bash
RUN_DIR=$(bash scripts/overnight_summary.sh start)
export OVERNIGHT_RUN_DIR="$RUN_DIR"
echo "[overnight] RUN_DIR=$RUN_DIR"
```

`--time-budget`을 초로 변환해 `BUDGET_SEC`에 저장. `START_TS=$(date +%s)`.

기본 가드:
- `git status --porcelain` 출력이 비어있어야 함 (uncommitted 변경 있으면 중단).
- 현재 브랜치가 `main`이 아니어도 OK — 그 브랜치를 base로 사용.

## 1. 큐 수집

```bash
specs=()
while IFS= read -r f; do specs+=("$f"); done < <(
  find docs/queue -maxdepth 1 -type f -name '[0-9]*_*.md' | sort
)
```

`docs/queue/_in-progress/`, `_done/`, `_failed/`, `_drafts/`는 스캔 대상 아님 (find의 `-maxdepth 1`이 처리). `_TEMPLATE.md`, `README.md`는 prefix 숫자가 없어 자동 제외.

큐가 비어있으면:
```bash
bash scripts/overnight_summary.sh cycle --spec "(empty)" --status skip --note "queue empty"
bash scripts/overnight_summary.sh end
```
하고 종료.

## 2. DB 스냅샷 (사이클 0)

```bash
bash scripts/db_snapshot.sh save overnight-${RUN_DIR##*/}-pre
```

각 사이클 시작 전엔 이 스냅샷으로 *되돌리지 않음* — DB는 그냥 상태가 누적되며, 누적이 acceptance에
영향을 주는 spec(있으면)에 한해 spec 본문이 명시적으로 복원을 요청해야 함. 사후 복구용 보존이 목적.

## 3. 메인 루프

```python  # 의사 코드 — 실제로는 Claude가 단계별 Bash 호출
consecutive_failures = 0
processed = 0
for spec in specs:
    if processed >= MAX_CYCLES: break
    if (time.time() - START_TS) > BUDGET_SEC - 5*60: break  # 마지막 5분은 cleanup용
    if consecutive_failures >= 2:
        log("HALT: 2 consecutive failures")
        break

    # /feature-from-spec 호출 (Agent tool 또는 직접 워크플로우 수행)
    result = run_feature_from_spec(spec)  # {status, stage, branch, pr, elapsed, note}

    bash overnight_summary.sh cycle \
        --spec "$spec" \
        --status "$result.status" \
        --stage "$result.stage" \
        --branch "$result.branch" \
        --pr "$result.pr" \
        --elapsed "$result.elapsed" \
        --note "$result.note"

    if result.status == "fail":
        consecutive_failures += 1
    elif result.status == "pass":
        consecutive_failures = 0
    # skip은 카운터 영향 없음

    processed += 1
```

### 실행 방식

각 사이클은 **Agent tool로 격리 실행**합니다 — 메인 컨텍스트가 부풀지 않게:

```
Agent({
  description: "Process spec NN",
  subagent_type: "general-purpose",
  isolation: "worktree",            # /feature-from-spec 안에서 이미 worktree 만들지만, 한 번 더 격리
  prompt: "/feature-from-spec docs/queue/01_xxx.md 를 실행하고 결과를 JSON으로 보고:
           {status, stage, branch, pr, elapsed, note}"
})
```

Agent의 result에서 JSON을 파싱해 `overnight_summary.sh cycle` 인자로 전달.

> 주의: spec 본문에 새 모듈/스키마 변경이 들어가는 경우 worktree 격리만으론 부족할 수 있음.
> 그런 spec은 `verify: e2e`로 명시해 무거운 검증을 받게 함.

## 4. 종료

```bash
bash scripts/db_snapshot.sh save overnight-${RUN_DIR##*/}-post
SUMMARY=$(bash scripts/overnight_summary.sh end)
echo "[overnight] DONE — $SUMMARY"
cat "$SUMMARY"
```

오래된 스냅샷 정리:
```bash
bash scripts/db_snapshot.sh clean --older-than 14
```

(선택) 일어나서 봐야 할 핵심 한 줄을 OS 알림으로 — 환경마다 다름 (notify-send / osascript /
Slack webhook). 본 명령은 알림을 띄우지 않음 — `~/easytrainer-overnight/latest/summary.md`를
아침에 열어 확인하는 게 표준.

---

## 중단 시나리오와 자동 대응

| 상황 | 자동 대응 |
|------|----------|
| 연속 2 실패 | HALT — summary에 "HALT after 2 consecutive failures" 행 추가 후 종료 |
| time budget 초과 | 다음 사이클 시작 막고 cleanup 단계로 |
| backend 컨테이너 다운 | 첫 spec에서 `baseline-smoke` 실패 → spec 1개 fail로 카운트 → 다시 시도 (다음 spec) → 같은 이유로 또 실패 시 연속 2회로 HALT |
| disk 부족 | worktree 만들기 실패 → spec fail → 다음 spec에서 또 실패 → HALT |
| 사용자 Ctrl-C | 진행 중인 사이클은 worktree에 보존, `summary.md` 마지막 행이 미완성으로 남을 수 있음 |

## 사후 정리 (일어난 후)

```bash
# 어떤 PR이 만들어졌는지
gh pr list --author @me --search "head:overnight/"

# 어떤 spec이 실패했는지
ls docs/queue/_failed/

# 야간 summary
less ~/easytrainer-overnight/latest/summary.md

# 모두 정리하고 다음 야간을 위해 워크트리 회수 (PR 머지 후)
for wt in /tmp/easytrainer-wt/*/; do
  if [ -d "$wt" ]; then
    git -C "$wt" status --porcelain | head -3
    # 머지된 게 확인되면:
    # git worktree remove "$wt"
  fi
done
```

---

## 처음 돌릴 때 (smoke test)

```
/overnight-queue --max-cycles 1 --time-budget 30m
```

처음엔 spec 1개만, 30분 budget으로 한 번 돌려서 인프라(권한 / git / gh / PR 생성)가
잘 굴러가는지 확인 후 본격 야간으로.
