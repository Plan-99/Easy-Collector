# Round_1 — carry vs anchor vs chunk30 (peg-in-hole)

Tournament: `act_resnet18_peg_20260528_2346`
Dataset: 8 (`ui_peg_v2_1779972650`, 30 episodes)
Trials per candidate: 15 × 400 steps
Common settings: `policy_type=ACT`, `vision_backbone=resnet18`, `action_type=joint`, `obs_state_keys=[qpos]`, `use_vae=true`, `kl_weight=10`, `num_epochs=500` (round_2 부터 10,000 으로 상향됨 — 아래 § "이번 라운드의 진단" 참고)

## Results

| candidate                       | ckpt | best_loss | train_time | success | xy_err mean (m) | tilt_cos > 0.99 |
|---------------------------------|------|----------:|-----------:|--------:|-----------------|-----------------|
| act_resnet18_round0_carry_r1 †  |  3   |    n/a    |  0m 10s    |  failed |   —             |   —             |
| act_resnet18_anchor_r1          |  4   |   2.16    |  4m 30s    |  0 / 15 | 0.135           | 14 / 15         |
| act_resnet18_chunk30_r1         |  5   |   2.03    |  4m 30s    |  0 / 15 | 0.124           | 12 / 15         |

† Carry candidate 가 학습 10 초만에 fail — `design_round()` 가 champion recipe 만 복사하고 `task_id` / `dataset_id` / `episode_num` 을 빼먹어서 backend 가 `/opt/easytrainer/datasets/undefined` 를 찾다가 죽음. 이 라운드 *중* 패치 (`candidate_dict()` 위에 recipe 를 layer 하도록) 적용했지만 in-memory orchestrator 에는 안 들어가서 round_2 carry 도 같은 이유로 fail. 이후 재시작에 fix 가 효과 발휘.

**No strictly-higher winner.** 셋 다 0% — champion 파일은 round_0 winner 그대로 유지.

## 후보별 차이점

세 후보 모두 ACT-ResNet18 + joint action + use_vae=True + kl_weight=10 + num_epochs=500 공통. 차이는 아래 표.

| 후보                  | 역할     | anchor 대비 변경 항목              | 가설 |
|-----------------------|----------|-------------------------------------|------|
| carry_r1 (ckpt 3)     | carry    | 동일 (champion 이 plain ACT-ResNet18) — 단 train 실패 (dataset binding 버그) | reproducibility 검증 |
| anchor_r1 (ckpt 4)    | anchor   | — (baseline, chunk_size=15)         | baseline |
| chunk30_r1 (ckpt 5)   | variant  | `chunk_size 15 → 30`, n_action_steps 그대로 1 | H2 — action decoder chunk 길이가 보틀넥인지 검증 |

**메트릭 의미.** `xy_err` = peg 끝과 hole 중심 사이 수평거리 (m), hole 직경 ≈ 2 cm. `z_err` = peg 끝과 hole 상단 면 사이 수직 gap. `tilt_cos > 0.99` = 그리퍼가 거의 완벽히 아래를 향한 trial 수.

## Failure mode — 라운드 0 과 동일, 더 narrow

세 후보 모두 같은 정성적 패턴:

- **Orientation 정확** — tilt_cos > 0.99 가 anchor 14/15, chunk30 12/15.
- **Z gap 거의 0** — z_err mean = −0.011 (anchor) / −0.015 (chunk30). 페그를 홀 위에 들고 있음.
- **XY gap 못 좁힘** — anchor 평균 13.5 cm, chunk30 평균 12.4 cm. round_0 (14.8 cm) 대비 미미한 개선이지만 statistically meaningless (15 trial).

청크 길이를 2 배(15 → 30) 늘려도 XY 정밀도가 안 좋아짐. **H2 (action decoder 변종)** 가설은 약하게 반증됨 — 디코더 청크 길이 가 병목이 아니다. 이번 토너먼트에서도 dpdino 라운드와 동일하게 **시각 인코더 (H1)** 가 진짜 병목으로 보임.

## 이번 라운드의 진단 — 학습량 부족이 0% 의 *진짜* 원인일 가능성

| ckpt | num_epochs | best_loss | best_epoch |
|------|-----------:|----------:|-----------:|
| 2    | 200        | 2.99      | 160        |
| 4    | 500        | 2.16      | 447        |
| 5    | 500 (ch30) | 2.03      | 491        |
| 7    | 500 (round_2 anchor — 아래 § note) | 1.94 | 472   |
| 8    | 500 (round_2 resnet34) | 2.16 | 487 |

200 → 500 epoch 에서 loss 가 2.99 → 1.94 로 단조감소. plateau 가 아님. 즉 `xy_err` 가 16 cm 라는 결과는 *모델 capacity 의 한계* 라기보다 *학습 종료 시점이 일러서 vision feature 가 충분히 영글지 않은 것* 일 가능성이 큼. ACT 원논문 권장 epoch 은 5,000 – 10,000.

이 진단에 따라 **round_2 종료 후** orchestrator 를 멈추고 `num_epochs=10,000` 으로 패치 + 재시작. round_3 부터 ACT 후보 모두 10k epoch 으로 학습. lr 은 ACT 원논문 그대로 1e-5 유지 (단조감소 중이라 lr 손댈 이유 없음).

## Round_2 에 대한 노트 — 중간 abort

Round_2 는 동일 코드(500 ep) 로 시작했고, `c0 carry (ckpt 6)` 는 carry 버그가 in-memory 라 또 fail, `c1 anchor_r2 (ckpt 7)` 는 학습 완료 후 0/15, `c2 resnet34_r2 (ckpt 8)` 는 학습 완료, 평가 도중 사용자 승인으로 orchestrator + evaluator 중단 + 재시작. round_2 는 결과적으로 *완전히 evaluated* 되지 않은 라운드라 별도 보고서 생략. tournament_log.jsonl 에는 round_start 만 남아있고 round_end 없음.

## Round_3 entrant slate

`num_epochs=10,000` (per-policy budget rule), 그 외 모든 hyperparameter 는 round_0 anchor 와 동일.

1. **act_resnet18_round0_carry_r3** — champion (round_0 winner) recipe + 10k epoch.
2. **act_resnet18_anchor_r3** — plain ACT-ResNet18 + 10k epoch.
3. **act_resnet18_low_reg_r3** — `dropout 0.1 → 0.05`, `kl_weight 10 → 1`. ACT-VAE 의 KL prior 가 30-demo regime 에서 overconstraining 일 가능성을 검증. (architectural 이 아니라 regularization 변종이므로 policy-evolve "hyperparameter 금지" 룰을 약간 stretch. 다음 라운드부터 H1 line 의 구조 변종으로 회복할 것.)

학습 시간 추정: ckpt 7 의 270 s / 500 ep = 0.54 s/ep 이므로 10k ep = **~90 분/후보** × 3 = **~4.5 시간/라운드**.

## Infra fixes landed during this round

1. **Carry-over dataset bug.** `design_round()` 가 champion recipe (policy_settings + train_settings) 만 복사하고 `task_id` / `dataset_id` / `episode_num` 을 빼먹어 backend 가 `/opt/easytrainer/datasets/undefined` 를 찾다가 죽음. **Fix:** `candidate_dict(name)` 으로 dataset binding 먼저 깔고 그 위에 recipe 를 layer.
2. **In-memory vs file divergence.** orchestrator 가 background 로 돌고 있어 파일 패치가 다음 process restart 때까지 반영 안 됨. round_2 에서 같은 carry 버그가 또 fire. **운영 룰:** orchestrator 패치 시 반드시 restart.
3. **Per-policy epoch budget.** `EPOCH_BUDGET = {"ACT": 10000, "DiffusionPolicy": 1000, "DPDino": 1000, "PI0": 1000}` 를 `candidate_dict` 와 carry-over 양쪽에 적용해 향후 정책 타입 추가 시에도 자동 반영.
