# Round_3 — 10k epoch carry vs anchor vs low_reg (peg-in-hole)

Tournament: `act_resnet18_peg_20260528_2346`
Dataset: 8 (`ui_peg_v2_1779972650`, 30 episodes)
Trials per candidate: 15 × 400 steps
Common settings: `policy_type=ACT`, `vision_backbone=resnet18`, `action_type=joint`, `obs_state_keys=[qpos]`, `chunk_size=15`, `use_vae=true`, `kl_weight=10`, **`num_epochs=10000`** (per-policy budget rule 첫 적용)
Per-candidate train_time ≈ 77 min, val_loss 가 0.08 수준으로 24x 감소.

## Results

| candidate                        | ckpt | best_loss | best_epoch | train_time | success | xy_err mean (m) | tilt_cos > 0.99 |
|----------------------------------|------|----------:|-----------:|-----------:|--------:|-----------------|-----------------|
| act_resnet18_round0_carry_r3     |  9   |   0.079   |   9 733    |   77m      |  0 / 15 | 0.171           | 14 / 15         |
| act_resnet18_anchor_r3           | 10   |   0.080   |   9 983    |   77m      |  0 / 15 | 0.162           | 14 / 15         |
| act_resnet18_low_reg_r3 †        | 11   |    n/a    |    n/a     |   33s      |  failed |   —             |   —             |

† `TypeError: ACTConfig() got multiple values for keyword argument 'dropout'`. `variant_lowdrop_kl1` 가 `policy_settings["dropout"]` 와 `train_settings["dropout"]` 양쪽에 dropout 을 박는데, `train_worker.py` 가 둘 다 ACTConfig 에 kwargs 로 넘겨 충돌. **Fix:** `policy_settings.dropout` 라인 제거 (`/tmp/tournament/orchestrate.py:variant_lowdrop_kl1`). 다음 orchestrator restart 부터 효과 발휘.

**No strictly-higher winner.** champion 파일 unchanged.

## 후보별 차이점

세 후보 모두 ACT-ResNet18 + joint action + chunk_size=15 + 10k epoch 공통. 차이는 아래 표.

| 후보                  | 역할     | anchor 대비 변경 항목                                | 가설 |
|-----------------------|----------|-------------------------------------------------------|------|
| carry_r3 (ckpt 9)     | carry    | 동일 (champion 이 plain ACT-ResNet18)                 | reproducibility 검증 |
| anchor_r3 (ckpt 10)   | anchor   | — (baseline, dropout=0.1, kl_weight=10, use_vae=True) | baseline |
| low_reg_r3 (ckpt 11)  | variant  | `dropout 0.1 → 0.05`, `kl_weight 10 → 1` (VAE KL prior 완화) | H3 — 30-demo regime 에서 over-regularization 가설. **단 ACTConfig dropout dup 버그로 train 33 s 만에 실패** |

**메트릭 의미.** `xy_err` = peg 끝과 hole 중심 사이 수평거리 (m), hole 직경 ≈ 2 cm. `tilt_cos > 0.99` = 그리퍼가 거의 완벽히 아래를 향한 trial 수.

## 결정적 발견 — Loss 24x ↓, 성공률 그대로 (input pipeline mismatch 의심)

이번 라운드의 진단적 의미:

- **학습은 제대로 됨.** carry 와 anchor 모두 val_loss ≈ 0.08 로 수렴 (round_1 의 2.0 대비 1/24). best_epoch 가 9 733 / 9 983 으로 라운드 budget 끝 무렵에 갱신 — 즉 10k epoch 가 적정량이지 plateau 가 아니다.
- **그런데 평가 결과는 round_1 (500 ep, loss 2.0) 과 정성·정량적으로 동일.** xy_err mean 0.162 – 0.171, tilt_cos > 0.99 비율 14/15. 페그를 들고 홀 위에 정렬은 하는데 마지막 16 cm 를 못 좁힘.

해석: **학습 단계와 추론 단계 사이에 input pipeline 미스매치가 있다.** model_tester 메모리의 디버깅 노트가 정확히 이 시나리오를 경고함:

> "policy 학습 로스는 낮지만 평가 success rate ≈ 0 일 때는 input pipeline 미스매치 가능성. diagnose_inference.py 로 학습 프레임에서의 per-joint norm err 가 0.3 미만인지 먼저 확인."

가설 후보:

1. **H-A — Image preprocessing 차이.** 학습은 `image_resolution=[224,224]` 로 resize + normalize, 추론은 `tutorial_evaluator` 가 `tutorial_eval_bridge` 에서 받은 `sensor_1/2/3` 프레임을 모델에 넘김. resize 타이밍/순서, normalization mean/std, 채널 순서(BGR↔RGB), uint8/float 등 어디든 어긋날 수 있음.
2. **H-B — State normalization.** `obs_state_keys=["qpos"]` 라 7-dim qpos 가 모델 input. dataset stats (`policy_preprocessor`) 가 학습 시 정규화에 쓰이는데, 추론 시 bridge 가 보내는 qpos 가 같은 normalizer 통과하는지 검증 안 됨.
3. **H-C — Sensor 매핑.** 학습 데이터는 `sensor_1=top, sensor_2=front, sensor_3=wrist`. evaluator 의 `--sensor-map sensor_1=top,sensor_2=front,sensor_3=wrist` 와 일치. 단, bridge 가 같은 카메라 ID 로 같은 프레임을 보내는지 confirm 필요.

이 가설들은 *epoch 가 더 늘어도 풀리지 않음* 이라는 round_3 의 negative result 가 직접 가리키는 방향. 더 epoch 늘리는 것은 의미 없고, 다음 단계는 **모델 inference 를 학습 프레임에 다시 돌려서 action 이 dataset action 과 일치하는지 검증** 하는 것.

## Round_4 entrant slate

토너먼트는 plan 대로 진행 (round_4 = carry + anchor + variant_wider_ffn).

1. **act_resnet18_round0_carry_r4** — champion (round_0 winner, lr=1e-5, 10k ep) 재학습.
2. **act_resnet18_anchor_r4** — plain ACT-ResNet18, 10k ep, 동일 설정.
3. **act_resnet18_widefnn_r4** — `dim_feedforward 3200 → 4800`, `n_decoder_layers 1 → 2`. ACT transformer capacity 를 늘려 visual feature → action mapping 의 표현력 부족을 압박.

단 **이 라운드도 같은 0% 패턴을 반복할 가능성이 매우 큼.** 입력 파이프라인이 미스매치면 모델 capacity 와 무관하다. 라운드 끝나면 diagnose_inference 단계로 넘어가야 한다.

## Infra fixes / 운영 노트

1. **Round 2 abort.** lr 패치 사용자 검토 중 잠시 멈추고 epoch 패치로 전환. round_2 의 c2 (ckpt 8, resnet34_r2, 500 ep) 는 학습 완료 / 평가 도중 abort. tournament_log.jsonl 의 round_2 는 `round_end` 가 없음. round_3 부터 `num_epochs=10000` 가 본격 적용.
2. **variant_lowdrop_kl1 fix.** `policy_settings.dropout` 라인 제거. orchestrator file patch 만 적용 — 효과는 round_8 (다음 low_reg 등장 round) 또는 그 전 restart 시.
3. **per-policy epoch budget 확인.** ckpt 9/10 train_settings.num_epochs 가 정확히 10000 로 박힘. carry-over 도 EPOCH_BUDGET 으로 override 되어 champion 옛 recipe (원래 200 ep) 의 영향 없이 새 epoch 적용.
4. **GPU 점유 시간.** 10k epoch × 3 후보 ≈ 4 시간 + 평가 ≈ 30 분 = 라운드당 4.5h. 6 시간 unattended budget 으로 일일 1.3 라운드.
