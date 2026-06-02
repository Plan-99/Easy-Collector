# Round_4 — 10k epoch carry vs anchor vs wide-FFN (peg-in-hole)

Tournament: `act_resnet18_peg_20260528_2346`
Dataset: 8 (`ui_peg_v2_1779972650`, 30 episodes)
Trials per candidate: 15 × 400 steps
Common settings: `policy_type=ACT`, `vision_backbone=resnet18`, `action_type=joint`, `obs_state_keys=[qpos]`, `chunk_size=15`, `use_vae=true`, `kl_weight=10`, `num_epochs=10000` (per-policy budget)
라운드 총 소요: 학습 234 min + 평가 ≈ 15 min = ≈ 4.2h

**메트릭 의미.** `xy_err` = trial 끝났을 때 peg 끝과 hole 중심 사이의 수평거리 (m). hole 직경 ≈ 2 cm 이라 ≤ 0.01 m 정도 가야 실제 삽입. `z_err` = peg 끝과 hole 상단 면의 수직 gap (음수 = 페그가 hole 위로 살짝 내려와 림과 접촉 가능). `tilt_cos` = gripper 의 -z 축과 world -z 축의 코사인 — 1 이면 완벽히 아래 향함.

## Results

| candidate                        | ckpt | best_loss | best_epoch | train_time | success | xy_err min/mean/max (m) | z_err mean (m) | tilt_cos > 0.99 |
|----------------------------------|------|----------:|-----------:|-----------:|--------:|--------------------------|---------------:|-----------------|
| act_resnet18_round0_carry_r4     | 12   |   0.084   |   9 844    |   77m      |  0 / 15 | 0.048 / 0.127 / 0.198    | −0.017         | 10 / 15         |
| act_resnet18_anchor_r4           | 13   |   0.093   |   9 687    |   77m      |  0 / 15 | 0.089 / 0.152 / 0.207    | −0.010         | 15 / 15         |
| act_resnet18_widefnn_r4          | 14   | **0.072** |   9 707    |   80m      |  0 / 15 | 0.077 / 0.141 / 0.248    | −0.011         | 14 / 15         |

**No strictly-higher winner.** champion 파일 unchanged (round_0 ckpt 2, 0% 그대로).

## 후보별 차이점

세 후보 모두 ACT-ResNet18 + joint action + chunk_size 15 + use_vae=True + 10k epoch 공통. 차이는 아래 표의 항목만.

| 후보                  | 역할     | anchor 대비 변경 항목                                              | 가설 |
|-----------------------|----------|---------------------------------------------------------------------|------|
| carry_r4 (ckpt 12)    | carry    | 동일 (champion 이 plain ACT-ResNet18 자체라 anchor 와 recipe 동일)  | reproducibility 검증 |
| anchor_r4 (ckpt 13)   | anchor   | — (baseline)                                                        | baseline |
| widefnn_r4 (ckpt 14)  | variant  | `dim_feedforward 3200 → 4800` (1.5x) + `n_decoder_layers 1 → 2` (2x) | H1 — transformer capacity 부족 가설 |

**carry == anchor 비효율 노트.** champion 이 round_0 의 plain ACT-ResNet18 그대로라 carry 와 anchor 가 같은 recipe 를 두 번 학습하는 셈. 라운드당 80분 GPU 낭비. Round_6 부터 design_round() 패치로 champion==anchor 시 carry 자리를 두 번째 variant 로 교체 (slate 가 `[anchor, variant_A, variant_B]` 로 바뀜).

## 결정적 발견 — Wide-FFN 의 loss 최저, 성공률은 여전 0%

이번 라운드의 핵심:

- **widefnn 의 best_loss 0.072 가 라운드 최저** — `dim_feedforward 3200 → 4800` + `n_decoder_layers 1 → 2` 로 ACT transformer capacity 를 1.5x 키운 효과가 train loss 에는 명확히 나타남 (carry 0.084, anchor 0.093 대비).
- **그런데 평가 0/15.** xy_err mean 0.141 m 로 carry (0.127) 와 anchor (0.152) 사이에 위치. capacity bump 가 task 성공에 기여 없음.
- carry_r4 의 xy_err min = **0.048 m (4.8 cm)** — 라운드 최단. 그래도 hole 직경(≈ 2 cm) 보다 2-3x 큼.

Round_3 의 input pipeline mismatch 가설을 강하게 보강. **모델 capacity 와 epoch 둘 다 dial-up 했지만 task perf 미동.** 학습은 충분히 수렴 (loss 0.07–0.09, best_epoch 가 9 687 – 9 844 으로 10k 직전), 그러나 학습된 정책이 추론 시간에 그대로 발휘되지 않음.

해석:

| Lever                    | round_0 → round_4 변화         | task 영향 |
|--------------------------|---------------------------------|-----------|
| num_epochs               | 200 → 10000 (50x)               | 없음      |
| transformer capacity     | dim_ff 3200 → 4800, dec 1 → 2   | 없음      |
| best_loss                | 2.99 → 0.072 (41x ↓)            | 없음      |
| xy_err mean (m)          | 0.148 → 0.127 (carry r4 최저)   | 미동      |

학습 측면의 lever 는 모두 결과적으로 task 성공률을 못 움직였다. **모델이 학습한 분포와 추론 분포 사이의 input mismatch** 가 보틀넥인 것이 거의 확정.

## 입력 파이프라인 미스매치 — 가설 우선순위

Round_3 의 H-A / H-B / H-C 를 그대로 carry. round_4 의 추가 증거를 반영해 우선순위 조정:

1. **H-A — Image preprocessing 차이 (강 우선)**. 학습은 LeRobot dataloader → `image_resolution=[224,224]` resize → ImageNet normalize. 추론은 `tutorial_evaluator` 가 bridge HTTP 응답에서 받은 prefer-RGB uint8 프레임을 그대로 모델에 넘김. resize 타이밍/순서, normalization (ImageNet mean/std vs 0-1 vs 0-255), 채널 순서 (BGR↔RGB), float/uint8 어디든 어긋날 가능성. capacity 와 epoch 둘 다 늘려도 풀리지 않는 점이 결정적 단서.
2. **H-B — State normalization (중 우선)**. dataset stats 기반 정규화 여부 미검증. 7-dim qpos 가 학습 시 zero-mean unit-var 처리 받았는데 추론 시 raw 값이 들어가면 attention 분포가 학습 영역 밖.
3. **H-C — Sensor 매핑 (저 우선)**. evaluator 의 `--sensor-map sensor_1=top,sensor_2=front,sensor_3=wrist` 가 데이터셋 어셈블러와 일치 확인됨. 단 bridge 가 같은 camera ID 로 같은 프레임을 보내는지는 미검증.

H-A 가 첫 번째 후보인 이유: ACT 가 visual feature 에 의존하는 정책이고, peg-in-hole 같은 정밀 XY task 는 image preprocessing 한 픽셀의 어긋남도 치명적. tilt/z 가 거의 완벽한데 xy 만 못 잡는 패턴은 vision feature 의 misalignment 와 잘 맞음.

## Round_5 entrant slate

토너먼트는 plan 대로 진행. 단 사용자가 round_5 종료 시점 또는 그 전에 멈춰서 `diagnose_inference.py` 로 입력 파이프라인 검증 단계로 전환 결정 가능성 높음.

1. **act_resnet18_round0_carry_r5** — champion (round_0 winner) 재학습 (carry-over 룰).
2. **act_resnet18_anchor_r5** — plain ACT-ResNet18, 10k ep (anchor 룰).
3. **act_resnet18_???_r5** — 다음 creative variant. `variant_lowdrop_kl1` fix 가 round_5 에 적용될지 또는 `act_resnet18_widefnn_r4` 의 변종으로 갈지 orchestrator 의 다음 라운드 design 에 따라 결정.

**중대한 주의.** Round_5 도 같은 0% 패턴을 반복할 확률이 매우 높다. 입력 파이프라인 미스매치라면 어떤 architectural / capacity 변종도 효과 없음.

## Diagnose 단계 권고

다음 라운드 후보를 모두 학습시키는 데 약 4 시간 + 30 분 평가 = 4.5 시간 GPU. 그 대신 **30 분 안에 H-A 를 검증** 가능:

1. ckpt 14 (round 최강 loss) 를 들고 학습 데이터셋의 1 episode 를 input 으로 다시 돌린다.
2. 모델 output action 과 dataset 의 GT action 의 per-joint L2 (norm err) 측정.
3. norm err < 0.3 이면 → 학습 추론 일치 → H-A 기각, 다른 가설 (H-B/H-C 또는 환경 차이) 로 이동.
4. norm err ≥ 0.3 이면 → 미스매치 확정. preprocessing 코드 (`policy_preprocessor` + evaluator 의 image conversion) 비교 후 fix.

`model_tester` 메모리 ([model_tester.md](../../../.claude/projects/-home-hjhj-EasyTrainer-v2-3-1/memory/model_tester.md)) 의 디버깅 노트가 이 시나리오를 정확히 지칭. 라운드 절약 ROI 가 크다.

## Infra fixes / 운영 노트

1. **variant_lowdrop_kl1 fix 효과 발효.** `/tmp/tournament/orchestrate.py` 의 `policy_settings.dropout` 라인 제거 patch 가 round_3 종료 직후 적용됨. 다음 low_reg 등장 라운드 (round_7 후보) 부터 효과. 본 라운드의 widefnn 후보는 별개 함수라 영향 무.
2. **per-policy epoch budget 일관성 확인.** ckpt 12/13/14 모두 num_epochs=10000 박혀있음. best_epoch 가 9 687 – 9 844 로 라운드 끝 무렵에 갱신 — 10k 가 ACT 의 적정 epoch 임을 또 한 번 확인. plateau 아님.
3. **GPU 점유 시간.** 10k epoch × 3 후보 ≈ 234 분 + 평가 ≈ 15 분 = 라운드당 4.2 h. 6 시간 unattended budget 으로 일일 1.3 라운드. round_3 (4.5h) 대비 round_4 (4.2h) 가 0.3h 짧은 이유: widefnn 의 transformer 가 약간 무겁지만 다른 두 후보의 train_time 안정화.
4. **Champion 정체.** Round_0 (ckpt 2, 0%) 이후 5 라운드 연속 champion 미갱신. strictly-higher 룰 (0% > 0% 불가) 때문에 champion 갈아탈 트리거 없음. 입력 파이프라인 수정 전까지는 이 정체가 풀릴 가능성 거의 없음.
