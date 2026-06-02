# learning.md — policy-lab 진화 로그

> 랩 운영 중 관찰된 실패·가지치기·진화 교훈을 누적. 사람/에이전트 기억이 아니라 여기에 기록.
> 하네스-레벨 교훈(경로·훅 등)은 EasyTrainer `../learning.md`. 이 파일은 **정책 진화** 신호 전용.
> 기록 관례: `[failure]` / `[fixed]` / `[pruned]` / `[skipped]` / `[round]`.

## 정책 진화 — 확정된 사실 (seed, memory 에서)

- `[anchor]` round_1+ 의 ACT 계열 자리는 **plain ACT-ResNet18** 로 고정 (ACTPlus 아님). 검증된 baseline anchor.
  근거: memory `project_round1_anchor`.
- `[failure] H-A 기각` — `act_resnet18_peg` 0% 의 원인은 image preprocessing 미스매치가 **아님**.
  모델은 학습 프레임에서 정확. 다음 가설 후보: H-D(closed-loop divergence) / H-B(state norm) / H-F(scene randomize).
  근거: memory `h_a_rejected_2026_05_29`.
- `[rule]` Loss 진단은 epoch 을 lr 보다 먼저 — loss 단조감소 중 0%면 lr 올리지 말고 num_epochs 2~4배.
  근거: memory `feedback_loss_diagnosis_epochs_over_lr`.
- `[rule]` 정책별 epoch budget — ACT(작은 모델)=10000, DPDino(큰 모델)=1000 (작을수록 길게).
  근거: memory `feedback_epoch_budget_by_policy`.

## 진화 라운드 (round)

- 과거 라운드 분석 보고서는 [docs/evolve/](docs/evolve/) (EasyTrainer 에서 이관됨):
  `act_resnet18_peg_20260528_2346` round 0/1/3/4/5, `dpdino_vs_actplus_20260526_1506` round 0.

## 랩 분리 (2026-06-01)

- `[moved]` `/policy-design`·`/policy-tournament`·`/policy-evolve` + `policy-researcher` 에이전트 +
  `docs/evolve/` 를 EasyTrainer `.claude/` 에서 이 랩으로 이관. EasyTrainer 하네스와 독립.
  호스트 경로는 `$EASYTRAINER_ROOT`(= git toplevel, 랩이 레포 안)로 참조하도록 수정.
- `[added]` `/lab-overnight`(야간 무인 진입점, evolve 루프 래퍼), `/lab-smoke`(readiness 게이트),
  AGENTS/golden-principles/rubric/README/easytrainer-integration.

## 다음 진화까지 지켜볼 지표
- H-D / H-B / H-F 중 어느 가설이 act_resnet18_peg 0% 를 푸는가.
- 챔피언 success_rate 추세 (strictly-higher 갱신 빈도).
