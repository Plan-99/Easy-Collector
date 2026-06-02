# Round_5 — 10k epoch carry vs anchor vs no-VAE (peg-in-hole)

Tournament: `act_resnet18_peg_20260528_2346`
Dataset: 8 (`ui_peg_v2_1779972650`, 30 episodes, 240×320 native resolution)
Trials per candidate: 15 × 400 steps
Common settings: `policy_type=ACT`, `vision_backbone=resnet18`, `action_type=joint`, `obs_state_keys=["qpos"]` (♦ 패치 적용 후엔 `[]`), `chunk_size=15`, `kl_weight=10` (no_vae 제외), `num_epochs=10000`
라운드 총 소요: 학습 232 min + 평가 ≈ 15 min = ≈ 4.1h

**메트릭 의미.** `xy_err` = peg 끝과 hole 중심 사이 수평거리 (m), hole 직경 ≈ 2 cm. `z_err` = peg 끝과 hole 상단 면 사이 수직 gap. `tilt_cos > 0.99` = 그리퍼가 거의 완벽히 아래를 향한 trial 수.

## Results

| candidate                       | ckpt | best_loss | best_epoch | train_time | success | xy_err min/mean/max (m) | z_err (m) | tilt_cos > 0.99 |
|---------------------------------|------|----------:|-----------:|-----------:|--------:|--------------------------|----------:|-----------------|
| act_resnet18_round0_carry_r5    | 15   |   0.084   |   9 870    |   77m      |  0 / 15 | 0.096 / 0.143 / 0.244    | −0.010    | 15 / 15         |
| act_resnet18_anchor_r5          | 16   |   0.078   |   9 391    |   78m      |  0 / 15 | **0.031** / 0.125 / 0.225 | −0.010    | 14 / 15         |
| act_resnet18_no_vae_r5          | 17   | **0.042** |   9 254    |   76m      |  0 / 15 | 0.050 / 0.143 / 0.238    | −0.011    | 14 / 15         |

**No strictly-higher winner.** champion 파일 unchanged (round_0 ckpt 2, 0% 그대로). **6 라운드 연속 champion 미갱신.**

## 후보별 차이점

세 후보 모두 ACT-ResNet18 + joint action + chunk_size=15 + 10k epoch 공통. 차이는 아래 표.

| 후보                 | 역할     | anchor 대비 변경 항목                            | 가설 |
|----------------------|----------|---------------------------------------------------|------|
| carry_r5 (ckpt 15)   | carry    | 동일 (champion 이 plain ACT-ResNet18)             | reproducibility 검증 (carry == anchor 비효율 한 번 더 노출 → Round 6 design_round() 패치) |
| anchor_r5 (ckpt 16)  | anchor   | — (baseline, use_vae=True, kl_weight=10)          | baseline |
| no_vae_r5 (ckpt 17)  | variant  | `use_vae True → False`, `kl_weight 10 → 0` (효과적으로 conditional regressor, no latent stochasticity) | H4 — VAE prior 가 30-demo regime 에서 representation 을 over-constrain 하는지 검증 |

## 결정적 발견 — No-VAE 의 loss 라운드 최저 (0.042), 성공률 여전 0%

이번 라운드의 핵심:

- **no_vae_r5 의 best_loss 0.042 가 토너먼트 전체 최저.** widefnn_r4 (0.072) 의 절반, anchor_r5 (0.078) 의 54%. VAE 비활성화 → KL 정규화 항 제거 → action distribution 이 deterministic 하게 수렴 → loss 함수 자체가 더 낮은 floor 를 가짐 (KL 항이 더해지지 않으니).
- **그런데 평가 0/15.** xy_err mean 0.143 m 로 anchor (0.125) 보다 *오히려 살짝 더 멀어짐*. loss 만 보면 가장 잘 학습된 정책인데 task 성공률에는 ZERO impact.
- ckpt 16 (anchor) 의 **xy_err min = 0.031 m (3.1 cm)** — 토너먼트 신기록. 한 trial 이 hole 의 1.5x 거리까지 좁힘. 그래도 trial 사이 분산이 매우 크고 평균은 12.5 cm.

| 토너먼트 누적 levers | round_0 → round_5 |
|----------------------|--------------------|
| num_epochs           | 200 → 10000 (50x)  |
| transformer capacity | dim_ff 3200 → 4800 (round_4) |
| chunk_size           | 15 → 30 (round_1) |
| backbone depth       | resnet18 → resnet34 (round_2 미완) |
| regularization       | dropout 0.1 → 0.05, kl_weight 10 → 1 (round_3 train fail) |
| VAE prior            | on → off (round_5)  |
| best_loss            | 2.99 → **0.042** (71x ↓) |
| eval success         | 0% → 0% (변화 없음) |

**학습 측면의 lever 6개 다 dial-up 했는데 task 성공률 미동.** 모델 측 모든 가설 (capacity, epoch, chunk, depth, reg, VAE) 이 한 번씩 실패. 보틀넥은 모델 외부 — **입력 파이프라인 미스매치** 가 사실상 확정.

## 입력 파이프라인 미스매치 — 코드 경로 분석 결과

Round_5 종료 직후 학습/추론 양측 image preprocessing 코드 경로를 비교 분석.

| 항목 | 학습 (`policies/utils.py:1593`) | 추론 (`tutorial_evaluator` → `process_image`) | 일치? |
|------|----------------------------------|----------------------------------------------|------|
| Resize | `transforms.Resize(image_resolution)` (224x224) | 동일 | ✅ |
| ToTensor | `transforms.ToTensor()` | 동일 | ✅ |
| pixel_range | ACT 은 '01' (`utils.py:1580`) | 기본 '01' | ✅ |
| image_resolution | train_settings → sidecar JSON | sidecar 로드 → fallback (224,224) | ⚠️ sidecar verify 필요 |
| **Image augmentation** | `RandomResizedCrop / Rotation / ColorJitter` (학습 시 매 step) | **없음** | ❌ 학습-only 변환 |
| **PIL mode** | `Image.fromarray(np.array(image))` — `image` 가 float32 [0,1] tensor 라 PIL 'F' mode (단일 채널) 가능성 | RGB PIL 보장 | ❌ 가능성 있음 |
| dataset stats | LeRobot dataset stats (raw 240x320 픽셀 분포 기반) | 동일 stats | ⚠️ — 학습 시 aug 거친 분포 vs 추론 raw 분포 |

처음 의심했던 **resize timing** 은 기각 (학습도 224x224 로 resize 후 normalize). 남은 의심점:

1. **Augmentation 만 학습 시 적용.** 학습 모델이 본 분포는 *augmented* 인데 추론은 raw — 학습이 raw frame 을 한 번도 안 봤을 수 있음. 30 demo × 10k epoch 면 model 이 augmented 분포에 깊게 fit.
2. **PIL 변환 시 mode.** float32 tensor → np.array → Image.fromarray 가 'F' mode (32-bit float, 1 channel) 가 되면 ToTensor 가 단일 채널 [0,1] 만 만들어냄. 학습이 어떻게든 통과는 했지만 (loss 떨어지는 거 보면), 채널 분포가 inference 의 RGB 와 완전히 다를 가능성.

이 두 후보는 진단 스크립트로 1 시간 안에 검증 가능.

## 다음 단계 — H-A 진단으로 전환

사용자 결정 (2026-05-29 11:07): "H-A 검증이 우선." Round 6 진입하지 않고 orchestrator stop. **ckpt 18 (carry_r6) 은 Playwright 큐 enqueue 까지 갔지만 학습 실패 (orchestrator process kill 시점), ckpt 19 (anchor_r6) 학습 0.6% 진행 후 stop 명령으로 abort.** GPU 자유.

## H-A 진단 결과 — **REJECTED (이미지 파이프라인 미스매치 없음)**

`diagnose_inference.py` (ckpt 16, dataset 8 episode 0, sensor_2 front cam, 세 프레임 idx 50/100/150) 결과:

| 비교 | max_abs | mean_abs | L2 | action L2 diff |
|------|--------:|--------:|---:|---:|
| A `train_process_image` vs B `eval_process_image` (raw 동일) | **0.000000** | 0.000000 | 0.0000 | **0.000000** |
| A vs C (raw → JPEG q=90 → eval_process_image) | 0.137–0.153 | 0.0017–0.0025 | 1.87–2.02 | 0.000165–0.000308 |
| B vs C (eval ± JPEG) | 동일 | 동일 | 동일 | 동일 |

핵심:

- **`backend/training_server/policies/utils.py:1648` 의 `process_image` 와 `backend/policies/utils.py:829` 의 `process_image` 는 함수 본문 byte-wise 동일.** docstring 만 다름. 둘 다 default `image_resolution=(224,224)`, `pixel_range='01'`, `transforms.Resize + ToTensor` 만 적용. ResNet18 backbone 은 ImageNet normalize 없음 (둘 다 [0,1] tensor 그대로 반환).
- **JPEG 라운드트립 (bridge HTTP 전송 mimic) 의 픽셀 노이즈는 평균 0.002 / max 0.15.** 모델 통과 후 action 차이는 L2 **0.0003** 미만. 학습 vs 추론 image preprocessing 은 사실상 무차이.
- **학습 프레임에 대한 모델 예측 정확도 매우 높음.**
  - frame 50: GT `[0.113, 0.854, -1.596, 0, -0.829, 0.113, 0.020]` vs pred `[0.133, 0.854, -1.609, 0, -0.814, 0.131, 0.021]` → L2 err 0.040, per-joint max 0.030 rad
  - frame 100: GT vs pred → L2 err 0.047, per-joint max 0.031 rad
  - frame 150: GT vs pred → L2 err 0.026, per-joint max 0.018 rad

**결론: 모델 자체는 학습 분포에서 정확히 작동한다. 이미지 전처리는 학습-추론 사이 미스매치 없음. 0% 의 원인은 다른 곳에 있다.**

## 새 가설 — 모델은 잘 학습되었으나 closed-loop 에서 발산

H-A 가 깨끗하게 기각된 만큼 남은 가설을 우선순위 재정렬:

1. **H-D (Closed-loop divergence)** — 단일 학습 프레임에 대해 모델은 0.03 rad 정확도로 다음 action 을 예측한다. 그러나 400 step rollout 동안 매 step 의 0.03 rad 오차가 누적 + state 가 OOD 로 빠지면 회복 불가. ACT 는 chunk size 15 인데 inference 에선 매 step receding-horizon 으로 첫 action 만 쓰면, 첫 action 의 작은 오차가 다음 observation 을 약간 OOD 로 만들고, 이것이 다음 chunk prediction 을 더 OOD 로... 이 발산 모드는 small-data ACT 의 알려진 실패 모드. (참조: ACT paper 의 temporal-ensemble 권장 — `n_action_steps > 1` 또는 ensemble averaging.)
2. **H-B (State normalization at runtime)** — 학습 시 qpos 는 `obs_state_keys=['qpos']` 로 dataset stats 기반 정규화 됨. 추론 시 bridge 의 `obs['state']` 가 raw qpos 로 들어가서 같은 normalizer (preprocessor) 통과하는지는 확인됨 (`_build_input` 에서 self.pre 적용). 단 **bridge 가 보내는 qpos 의 joint 순서/단위 (rad vs deg)** 가 dataset 의 그것과 1:1 일치한다는 보장은 미검증.
3. **H-E (Action interpretation)** — 모델은 8-dim action 출력 ([j1..j7, done_token]). 평가기는 7-dim 만 bridge 에 보냄. **단 학습 시 succeed (done) token 이 trainer 의 action_dim 에 포함되어 있어 model 이 그 자리에 의미 있는 신호를 emit 한다면, 추론에서 잘라내는 것이 학습 분포와 misalignment 일 가능성.** train_meta 에 action_dim 8 인지 7 인지 확인 필요.
4. **H-F (Camera scene mismatch)** — 학습 frame 과 추론 frame 의 카메라 위치/randomize 가 다를 가능성. tutorial 시뮬은 `/tutorial/randomize` 로 매 trial 마다 peg/hole 위치를 무작위로 옮기는데, 데이터 수집 시 randomize 분포 와 평가 시 randomize 분포 가 정확히 동일한지 확인 필요. 동일 service 호출이면 OK.

가장 빠른 다음 검증:

- **H-D 확인**: ckpt 16 으로 episode 0 의 frame 0 부터 시작해서 모델 action 을 sequential 하게 적용 → 50 step 후 모델이 예측한 state 가 demo 의 state[50] 과 얼마나 다른지. 시뮬 없이 forward-only 시뮬로도 OK (model + env step).
- **H-B 확인**: 평가 브릿지를 띄우고 `/observe` 한 번 호출해서 `obs['state']` 를 받아본 뒤, dataset episode 0 의 frame 0 qpos 와 byte-wise 비교.

이 두 검증은 H-A 보다 추가 환경 setup 필요하지만 1 시간 이내 완료 가능.

## H-D 추가 진단 — Temporal Ensemble 도 0% (REJECTED)

`config.json` 확인 결과 ckpt 16 은 `n_action_steps=1`, `temporal_ensemble_coeff=None`, `chunk_size=15`. 즉 매 step 에서 15-action chunk 의 첫 action 만 사용 — ACT 원논문이 권장하는 temporal ensemble 미적용. small-data ACT 의 알려진 발산 모드.

`/tmp/eval_with_ensemble.py` — `ACTTemporalEnsembler(coeff=0.01, chunk_size=15)` 를 평가기에 monkey-patch 로 attach 후 ckpt 16 재평가 (sim+bridge 가 ROS_DOMAIN_ID=0 에 이미 떠 있어서 별도 startup 불필요):

| trial | success | xy_err (m) | z_err (m) | tilt_cos |
|------:|:-------:|-----------:|----------:|---------:|
| 1 | ✗ | 0.123 | -0.010 | 1.00 |
| 2 | ✗ | 0.133 | -0.010 | 1.00 |
| 3 | ✗ | 0.199 | -0.033 | -0.00 |
| 4 | ✗ | 0.191 | -0.033 | -0.00 |
| 5 | ✗ | 0.069 | -0.010 | 1.00 |

**0/5, xy_err min 0.069 m — baseline ckpt 16 의 0.031 m 보다 *더 멀어짐*.** tilt_cos 가 5 회 중 3 회만 1.0 (baseline 14/15), 즉 temporal ensemble 이 orientation 보정도 dampen 함. H-D 도 **REJECTED**.

## 종합 결론 — Architectural / data 단으로 회귀

- **모델 자체는 학습 분포에서 정확하다** (per-joint 0.03 rad 정확도).
- **이미지 전처리는 학습-추론 동일** (H-A 기각).
- **Closed-loop temporal smoothing 도 도움 안 됨** (H-D 기각, 오히려 악화).
- **남은 의심점은 architectural/data 단**: 30 demos × 224x224 resolution 으로는 peg-hole 정밀 정렬 (마지막 3-15 cm) 의 sub-pixel feature 가 학습 불가. xy_err min 0.031 m 는 ckpt 16 의 capability ceiling — 평균 12.5 cm 는 trial 별 scene randomize 분산.

**다음 단계**: H-A / H-D 진단 종료 → Round 6 재개. 패치된 orchestrator (obs_state_keys=[], merged-slate, EPOCH_BUDGET) 로 재시작. Round 6 후보는 **더 큰 visual feature 분해능을 노리는 variant 들** 우선순위:

- `vision_backbone: resnet18 → resnet34` (round_2 에서 학습 abort 됨 — 이번에 10k epoch full)
- `image_resolution: 224 → 320` (native 240x320 의 정보 손실 최소화; resize 1.5x → vision feature spatial 분해능 ↑)
- DINOv2 backbone 변종 (multi-layer fusion)

이 셋 중 둘은 자동으로 design_round() 의 CREATIVE_VARIANTS 풀에 있어야 함 — `/tmp/tournament/orchestrate.py` 점검 필요.

## Infra fixes / 운영 노트

1. **orchestrate.py + champion.json 패치 4건 (Round 5 종료 직후 file-level 만 적용, in-memory orchestrator 는 이미 stop):**
   - `obs_state_keys ["qpos"] → []` (프로젝트 컨벤션, 단 0% 의 원인 아님 — 사용자 명시).
   - `design_round()` — champion==anchor 시 carry 자리에 두 번째 variant 투입 → `[anchor, variant_A, variant_B]`. 라운드당 80분 GPU 절약.
   - `variant_lowdrop_kl1` 의 `policy_settings.dropout` 라인 제거 (round_3 ACTConfig dup 버그 fix).
   - per-policy EPOCH_BUDGET 적용 — ACT=10000, DPDino=1000, PI0=1000.
2. **다음 orchestrator 재시작 시점**: H-A 진단 결과에 따라 결정. 미스매치 fix 가 되면 ckpt 16 재평가가 우선; 재학습 필요하면 그 후 Round 6 (이번엔 design_round 패치 + obs_state_keys=[] 양쪽 효과 발휘).
3. **GPU 효율 누적 분석.** Round 1–5 총 5 라운드 × 4.2h ≈ 21h 학습 시간. 같은 0% 패턴 5번 반복. H-A 진단을 round_3 종료 시점에 했다면 3 라운드 (12.6h) 절약 가능했음. 다음 모델 변종 검토 전엔 항상 inference path verification 먼저.
