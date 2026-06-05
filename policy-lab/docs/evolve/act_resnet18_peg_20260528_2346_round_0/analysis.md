# Round_0 — ACT-ResNet18 baseline (peg-in-hole)

Tournament: `act_resnet18_peg_20260528_2346`
Dataset: 8 (`ui_peg_v2_1779972650`, 30 episodes, ground-truth success-filtered)
Trials per candidate: 20 × 400 steps
Common settings: `policy_type=ACT`, `vision_backbone=resnet18`, `action_type=joint`, `obs_state_keys=[qpos]`, `chunk_size=15`, `use_vae=true`, `kl_weight=10`

## Results

| candidate              | ckpt | num_epochs | best_loss | train_time | success | xy_err (m)    | tilt_cos > 0.99 | z_err (m) |
|------------------------|------|-----------:|----------:|-----------:|--------:|---------------|-----------------|----------:|
| act_resnet18_round0    | 2    |        200 |      2.99 |   1m 05s   |  0 / 20 | 0.067 – 0.258 | 19 / 20         | −0.011    |

**No baseline reached.** Champion 파일에 0% 로 일단 등재 (strictly-higher 룰 적용 시 다음 라운드 후보가 0% 초과만 하면 갈아탐).

## 후보별 차이점

라운드 0 은 baseline 한 개만 띄움 (anchor only). round_1 부터 [carry, anchor, variant] 슬레이트 도입.

| 후보               | 역할     | 설명                                                        |
|--------------------|----------|-------------------------------------------------------------|
| act_resnet18_round0 (ckpt 2) | baseline | plain ACT-ResNet18, num_epochs=200 (이 라운드만 짧음), chunk_size=15, use_vae=True, kl_weight=10, joint action, qpos input |

## Failure mode

DPDino/ACTPlus 라운드와 정확히 동일한 정성적 패턴:

- **Orientation 정확** — tilt_cos > 0.99 가 19/20. 그리퍼는 거의 항상 아래를 본다.
- **Z gap 작음** — z_err ≈ −0.011 m. 페그를 홀 림 바로 위에 들고 멈춤.
- **XY gap 크고 산만함** — xy_err 가 6.7 cm 부터 25.8 cm 까지 분산. 평균 14.8 cm 로 홀 직경(≈ 2 cm)보다 한 자릿수 큼.

요컨대 정책이 "내려보고 잡으러 가는" 자세는 학습했지만, 시각으로부터 마지막 6 – 25 cm xy 좁힘은 못 한다. *Visual XY-localization bottleneck* — 이전 토너먼트와 같은 병목.

## 학습 자체가 부족했는지 진단

ckpt 2 의 best_loss = 2.99 가 epoch 160 (200 중) 에서 나옴. 즉 학습은 epoch 끝까지 단조감소 중이었지 plateau 가 아니다. 200 epoch 은 30 demo × `chunk_size=15` 의 ACT 학습 신호를 fit 시키기에 절대량이 부족.

ACT 원논문 권장은 5,000 – 10,000 epoch (Pusht/transfer cube benchmark). 이 토너먼트의 초기 budget 200/500 은 ACT 학습 곡선 *시작부* 만 본 셈. 다음 라운드부터 num_epochs 를 **10,000** 으로 (정책 크기 ↔ epoch 룰: ACT 작은 모델 = 길게).

## Why no other candidate

Round_0 은 명시적으로 anchor 한 개로 시작 (dpdino_vs_actplus 라운드와 다른 토너먼트 라인 — 검증된 ACT-ResNet18 단일 baseline 으로 출발). Carry / creative variant 는 round_1 부터 도입.

## Round_1 entrant slate

`[carry, anchor, creative variant]` 룰에 따라:

1. **act_resnet18_round0_carry_r1** — round_0 winner 동일 재학습. (champion recipe 그대로, 단 num_epochs 는 round_1 budget 적용.)
2. **act_resnet18_anchor_r1** — plain ACT-ResNet18, 동일 설정. anchor 항상 유지.
3. **act_resnet18_chunk30_r1** — `chunk_size=15 → 30` 으로 두 배. 액션 청크를 길게 잡아 정책이 더 commit 한 trajectory 를 내도록 압박. (H2 line — action decoder 쪽 변종.)

## Infra notes

- evaluator: `tutorial_evaluator --sensor-map sensor_1=top,sensor_2=front,sensor_3=wrist --bridge-url http://127.0.0.1:7799` 로 호출 (model_tester skill 의 sensor 1/2/3 매핑).
- 학습은 사용자 요구사항대로 Playwright + UI 경로 (`/tmp/tournament/launch_train.py`) 로 enqueue.
- ROS_DOMAIN_ID=0, eval bridge cameras `top_cam,front_cam,wrist_cam`.
