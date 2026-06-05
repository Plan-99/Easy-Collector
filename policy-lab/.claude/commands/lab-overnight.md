---
description: policy-lab 야간 무인 진입점 — readiness 게이트 후 정책 진화 루프를 시간/라운드 예산 안에서 자율 반복
argument-hint: "[--rounds 0] [--time-budget 6h] [--target-rate 0.5] [--num-trials 5] [--budget-epochs 300] [--seed-candidates PATH] [--dataset-id 15]"
---

# /lab-overnight — 정책 랩 야간 자율 루프 (이 랩의 nightwatch)

EasyTrainer 의 `/overnight-queue`(기능 spec 처리)와 **별개**. 이건 **정책 진화 전용** 야간 진입점이다.
사람이 자는 동안 `/policy-evolve` 루프를 readiness 게이트·예산·요약·스냅샷·중단 가드와 함께 굴린다.

인자: `$ARGUMENTS` (모두 선택, `/policy-evolve` 인자를 그대로 전달)
- `--time-budget Xh`  총 시간 예산 (기본 6h). 초과 직전 라운드 시작 막음.
- `--rounds N`        최대 라운드 (기본 0 = target-rate/시간예산까지 무한).
- `--target-rate R`   챔피언 success_rate 가 이 값 이상이면 종료 (기본 0.5).
- 나머지(`--num-trials`,`--budget-epochs`,`--seed-candidates`,`--dataset-id`,`--sensor-map`)는 `/policy-evolve` 로 전달.

---

## 0. 초기화
```bash
EASYTRAINER_ROOT="${EASYTRAINER_ROOT:-$(git rev-parse --show-toplevel)}"   # 랩이 레포 안 → EasyTrainer 루트
export OVERNIGHT_BASE="${OVERNIGHT_BASE:-$HOME/policy-lab-overnight}"       # 기능 야간과 분리된 별도 루트
RUN_DIR=$(bash "$EASYTRAINER_ROOT/scripts/overnight_summary.sh" start)
export OVERNIGHT_RUN_DIR="$RUN_DIR"
echo "[lab-overnight] RUN_DIR=$RUN_DIR"
```
- `--time-budget` → `BUDGET_SEC`, `START_TS=$(date +%s)`.
- 가드: `git status --porcelain` 비어있어야 함(uncommitted 있으면 중단). 현재 브랜치를 base 로 사용.

## 1. readiness 게이트
`/lab-smoke` 수행. 실패면 한 번 컨테이너 기동 재시도 후에도 실패 시 **중단**(summary 에 `infra-down` 기록).

## 2. DB 스냅샷 (pre)
```bash
bash "$EASYTRAINER_ROOT/scripts/db_snapshot.sh" save lab-${RUN_DIR##*/}-pre
```
(라운드마다 되돌리지 않음 — 사후 복구용 보존.)

## 3. 진화 루프
`/policy-evolve` 를 인자 그대로 호출한다. evolve 는 **자체적으로** round 루프(토너먼트→분석→다음 후보
설계→재토너먼트)를 돈다. lab-overnight 는 그 위에 **예산·게이트·요약·중단 가드**를 씌운다:

각 라운드 경계에서:
1. `if (now - START_TS) > BUDGET_SEC - 10*60: break` (마지막 10분 cleanup 확보).
2. 라운드 시작 전 `/lab-smoke`(eval bridge 가 야간에 죽는 일이 잦음 — 재기동).
3. 라운드 결과를 요약에 기록:
   ```bash
   bash "$EASYTRAINER_ROOT/scripts/overnight_summary.sh" cycle \
     --spec "round_$ROUND" --status "$STATUS" \
     --note "winner=$WINNER K/M ($RATE%) champion=$CHAMP_STATE"
   ```
4. 연속 2라운드가 **진척 0**(챔피언 미갱신 + 모든 후보 학습/평가 실패) 이면 HALT (구조적 문제 의심).
5. 종료 조건(evolve 와 동일): `champion.success_rate ≥ --target-rate` / `--rounds` 소진 / 시간예산 / 사용자 중단.

> 라운드 분석 보고서 커밋본은 `policy-lab/docs/evolve/<run>_round_<N>/analysis.md` (golden-principles #9).

## 4. 종료
```bash
bash "$EASYTRAINER_ROOT/scripts/db_snapshot.sh" save lab-${RUN_DIR##*/}-post
SUMMARY=$(bash "$EASYTRAINER_ROOT/scripts/overnight_summary.sh" end)
echo "[lab-overnight] DONE — $SUMMARY"; cat "$SUMMARY"
bash "$EASYTRAINER_ROOT/scripts/db_snapshot.sh" clean --older-than 14
```
아침 확인: `$OVERNIGHT_BASE/latest/summary.md` + `policy-lab/docs/evolve/`.

## 중단 시나리오 자동 대응
| 상황 | 대응 |
|------|------|
| eval bridge 사망 | 라운드 전 `/lab-smoke` 가 재기동 |
| 연속 2라운드 진척 0 | HALT + summary 기록 |
| 시간예산 초과 | 다음 라운드 막고 cleanup |
| 후보 1개 등록 실패 | 그 후보 빼고 winner+나머지로 진행 (evolve Phase C 처리) |
| 컨테이너 다운 | step 1 게이트에서 `infra-down` 보고 후 중단 |

## 처음 돌릴 때 (smoke)
```
/lab-overnight --rounds 1 --time-budget 1h --num-trials 3 --budget-epochs 100 --seed-candidates <path>
```
1라운드 1시간으로 인프라(게이트/요약/스냅샷/evolve 호출)가 도는지 확인 후 본격 야간으로.

## 관련
- 진화 루프(라운드 1회분): [policy-evolve.md](policy-evolve.md)
- readiness: [lab-smoke.md](lab-smoke.md)
- 채점/원칙: [../../rubric.md](../../rubric.md), [../../golden-principles.md](../../golden-principles.md)
- EasyTrainer 공유 스크립트: `$EASYTRAINER_ROOT/scripts/{overnight_summary,db_snapshot,smoke}.sh`
