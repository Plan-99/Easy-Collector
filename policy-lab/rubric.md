# Rubric — policy-lab 채점 기준

> Evaluator 가 쓴다. 생성(policy-researcher)과 분리(헌법 제6원칙). 거의 전부 objective gate.

## A. 정책 후보 (게이트 — 토너먼트 진입 자격)

| 항목 | 유형 | 합격 기준 |
|------|------|-----------|
| import smoke | objective | `python3 -c 'from training_server.policies.<t>.modeling_<t> import *'` OK |
| 1-epoch 학습 | objective | `_run_mini_train --num-epochs 1` 통과 (NaN/OOM/missing-key 없음) |
| 등록 완결성 | objective | train_worker + evaluator dispatch + DB POLICY_CONFIGS + UI modelConfigs 모두 추가 |
| 구조 변형 | objective | architecture diff vs baseline 명시 + arXiv 근거 (sweep 후보 기각) |

게이트 불통과 후보는 토너먼트에서 제외 (winner+나머지로 진행).

## B. 토너먼트 / 진화 (성과)

| 항목 | 유형 | 가중치 | 합격 기준 |
|------|------|--------|-----------|
| success_rate | objective | 100 | ground-truth 성공 trial 비율 (동일 dataset·epoch·trial 수) |

- **챔피언 갱신:** 새 winner success_rate 가 현 챔피언보다 **strictly 큼** 일 때만 (동률 무시 = 정상).
- **진화 종료:** `winner.success_rate ≥ --target-rate` 또는 `--rounds` 소진 또는 사용자 중단.
- 보조 지표(분석용, 합격선 아님): mean steps-to-done, train/val loss, early_done 비율, xy_err mean, tilt_cos mean.

## 메모
- subjective 항목을 두지 않는다 — 정책 품질은 success_rate 라는 단일 객관 신호로 판정 가능하므로
  Evaluator 자기편향(제6원칙) 위험을 원천 차단. 설계서의 "왜 좋아질까"는 *가설*일 뿐 채점 대상 아님.
- 빠른 진화 라운드는 작은 `--num-trials`(예 5)로 트렌드를 잡고, 챔피언 후보 출현 시 30-trial 재측정.
