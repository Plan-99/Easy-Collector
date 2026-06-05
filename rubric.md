# Rubric — EasyTrainer 채점 기준

> (foundry-harness/templates/rubric.md.tmpl 인스턴스화.)
> Evaluator 가 사용한다. 생성 주체(Generator: `/feature-from-spec`, policy-researcher)와 **분리**(헌법 제6원칙).
> objective 항목은 자동 판정(스크립트/exit code), subjective 는 명시적 기준으로.
> 자율 운영의 "done" 협상 양식은 spec 자체(`docs/queue/_TEMPLATE.md` = sprint-contract 역할)에 있다.

## A. 자율 기능 사이클 (`/feature-from-spec` → PR)

| 항목 | 유형 | 가중치 | 합격 기준 |
|------|------|--------|-----------|
| baseline + 변경후 smoke | objective | 30 | `bash scripts/smoke.sh` exit 0 (두 번 모두) |
| spec verify hooks | objective | 25 | spec frontmatter `verify`(none/smoke/e2e) 에 해당하는 검증 명령 전부 exit 0 |
| 범위 준수 | objective | 20 | `git diff` 가 spec "영향 파일" 내. "Out of Scope" 폴더 무변경 |
| golden-principles 게이트 | objective | 15 | i18n·CSS·매뉴얼 게이트 위반 없음 ([golden-principles.md](golden-principles.md)) |
| 매뉴얼 동기화 | objective | 10 | `bash scripts/manual-check.sh` exit 0 (또는 `manual: skip/already`) |

**합격선:** 총점 = 100 이고 **objective 항목 실패 0개**. 하나라도 실패면 spec 은 `_failed/` 로,
1회 한정 재시도 후에도 실패면 fail 보고 + worktree 보존.

## B. 정책 토너먼트 / 진화 (`/policy-tournament`, `/policy-evolve`)

| 항목 | 유형 | 가중치 | 합격 기준 |
|------|------|--------|-----------|
| import + 1-epoch smoke | objective | (gate) | `_run_mini_train.py` 1 epoch 통과 (NaN/OOM/missing-key 없음) |
| 등록 완결성 | objective | (gate) | train_worker dispatch + evaluator dispatch + DB POLICY_CONFIGS + UI modelConfigs 모두 추가 |
| success_rate | objective | 100 | ground-truth 성공 trial 비율 (동일 dataset·epoch·trial 수) |

**챔피언 갱신 규칙:** 새 winner 의 `success_rate` 가 현 챔피언보다 **strictly 큼** 일 때만 갱신
(동률은 무시 = 정상). 구조 변형만 후보로 인정 — 단순 hyperparameter sweep 은 기각 (memory
[feedback_policy_evolve_creativity], policy-researcher 헌장).

## 메모

- 가중치는 우선순위를 드러낸다. 자율 사이클은 **회귀 방지(smoke+범위)** 에 절반(50)을 둔다 —
  "새 기능"보다 "기존을 안 깬다"가 우선.
- objective 게이트가 전부라 사실상 pass/fail. subjective 항목을 늘리면 Evaluator 가
  자기 결과를 후하게 주는 위험(제6원칙)이 커지므로 의도적으로 최소화했다.
