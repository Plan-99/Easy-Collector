# openpi vs EasyTrainer PI0.5 — 차이점 및 수정 기록

EasyTrainer에서 PI0.5를 학습/추론하면서 openpi 대비 발견한 모든 차이점과 수정 내역을 정리한 문서.

## 1. 프레임워크 차이 (구조적)

| 항목 | openpi | EasyTrainer |
|------|--------|-------------|
| 백엔드 | JAX/Flax | PyTorch |
| 분산 학습 | FSDP / multi-GPU | 단일 GPU |
| 체크포인트 포맷 | Orbax | safetensors |
| 학습 방식 (full FT preset) | Full fine-tune (4B params) | — |
| 학습 방식 (LoRA preset) | **하이브리드: LLM body LoRA + vision/projector/heads full FT** | **하이브리드 동일 (2026-04-28 변경)** |
| Epoch 정의 | **프레임 단위** (모든 시작점 통과) | **에피소드 단위** (에피소드당 랜덤 1개) |

→ 같은 "200 epoch"이라도 EasyTrainer는 openpi의 ~2-4 epoch 분량의 실제 학습량.

> **중요 (2026-04-28 발견)**: openpi의 "LoRA fine-tuning"은 사실 100% LoRA가 아님. `gemma_2b_lora` freeze filter는 `nnx.All('*llm*', Not('*lora*'))` — LLM body만 LoRA가 걸리고 **vision_tower, multi_modal_projector, action heads는 full FT**됨. 이전 EasyTrainer는 "다 freeze + LoRA만 학습"이라 본질적으로 다른 학습 방식이었고, 이게 vision-blind / 행동 망가짐의 근본 원인이었음. §10에서 자세히.

## 2. 발견하고 수정한 버그들

| # | 버그 | 영향 | 수정 위치 |
|---|------|------|----------|
| 1 | `process_image`가 [0,1] 출력 — pi05_base는 [-1,1] 기대 | 🔥🔥 visual feature 망가짐, "움직이지만 못 잡음" | `policies/utils.py` `process_image(pixel_range='-11')` |
| 2 | Language embedding 이중 scaling (×sqrt(2048) × 2) | 🔥🔥 text magnitude 46배 → pi05_base 입력 분포 OOD | `modeling_pi05.py` `embed_prefix` |
| 3 | Image embedding 유사 이중 scaling | 🔥 image magnitude 45배 OOD | `modeling_pi05.py` `embed_image` |
| 4 | pi05_base vocab=257152 vs 우리 257216 | 🔥 weights 로드 실패 | `modeling_pi05.py` 257216 → 257152 통일 |
| 5 | pi05_base의 `vision_tower.vision_model.*` key prefix | 🔥 437/438 key mismatch | `train.py` key translator |
| 6 | embed_tokens ↔ lm_head tied weights 미복원 | 🔥 1 missing | `train.py` lm_head → embed_tokens 복사 |
| 7 | `Pi05PrepareStateTokenizerProcessorStep` registry 미등록 | 🔥 inference 시 정규화 skip | `policies/utils.py` 사전 import |
| 8 | `task` key 미존재 | 🔥 inference crash | `forward_pass` + `checkpoint_test.py` 매핑 |
| 9 | `AbsoluteActionsProcessorStep.relative_step` disk 로드 후 None | 🔥 delta 모드 inference crash | `policies/utils.py` 로드 후 rewire |
| 10 | gripper도 delta로 변환 (openpi는 absolute) | 🔥 gripper 제어 망가짐 | `relative_action_processor.py` `absolute_action_dims` |
| 11 | use_relative_actions=True 시 stats가 raw에서 계산 | 🔥 정규화 깨져 학습 신호 0 | `policies/utils.py` `get_norm_stats` delta 계산 |
| 12 | `relative_action_processor`에 dual-arm 지원 없음 | 양팔 로봇 학습 불가 | `relative_action_processor.py` `relative_action_mask` |
| 13 | done token 컬럼 q01=q99=0 → normalize 폭발 | 학습 신호 왜곡 | `policies/utils.py` q99-q01 최소 1e-2 clamp |
| 14 | scheduler/grad_clip이 코드에 없음 (config는 있는데) | warmup/clip 무시 | `scripts/train.py` LambdaLR + clip |
| 15 | pi05_base 사전학습 weights 로드 자체가 안 됨 | 🔥 사실상 from-scratch 학습 | `scripts/train.py` lerobot/pi05_base 자동 로드 |
| 16 | meta-device 후 weights 재할당 안 됨 (CPU OOM) | 학습 시작도 못 함 | `pi_gemma.py` meta device 후 materialize |
| 17 | Adam → AdamW 누락, betas 기본값 잘못 | gradient 갱신 살짝 다름 | `scripts/train.py` AdamW + betas |
| 18 | `EC_BACKEND_AUTORELOAD=1`이 학습 중 코드 수정 → kill | 학습 중단 | 학습 중 파일 수정 안 하기 |
| 19 | draccus가 `list[X] \| None` encoding 못함 | save_pretrained 마지막에 crash | `configuration_pi05.py` default_factory=list |
| 20 | **LoRA recipe가 openpi와 카테고리 차이** — vision/projector/heads도 LoRA로 wrap | 🔥🔥 vision capacity 부족 → real≈zeros, robot이 학습 데이터에 없는 행동 | §10 5-step 재구성 (2026-04-28) |
| 21 | `best_state_dict_cpu` 캡처가 `'lora' in k`로만 필터 | 🔥 full-FT 모듈은 best epoch 아닌 last epoch 가중치로 저장 (frankenstein checkpoint) | `train.py` `requires_grad` 기반 필터로 수정 |
| 22 | LoRA scale α/r = 2.0 (alpha=128, r=64) | 🔥 noise amplification 2배 → ckpt 94 erratic 행동 | UI 기본값 alpha=64, openpi는 α=r=1.0 |
| 23 | Prompt에 32-token state string (24개 padding noise) | 🔥 pi05_base 사전학습 분포 밖, prefix attention 낭비 | `Pi05PrepareStateTokenizerProcessorStep.state_dim` 추가 |
| 24 | wrist 카메라에도 spatial aug (crop/rotate) | gripper-observation geometry 깨짐 | `EpisodicDataset(wrist_sensor_ids=[...])` 인프라 추가 |
| 25 | Best epoch이 raw val_loss argmin = noise outlier 잡음 | 🔥 ckpt 96은 epoch 98(outlier dip) 저장, 실제 best 영역(140부근) 손실 | `train.py` smoothed val_loss (5-epoch window) |
| 26 | EMA weights 미구현 (openpi는 ema_decay=0.99) | 🔥 매 epoch weight noise 그대로 저장됨, smooth 안 됨 | `train.py` EMA shadow weights, validation/save 모두 EMA 사용 |
| 27 | Validation 1-pass per epoch = 작은 val set 변동 큼 | val_loss noise std 0.02~0.04 (epoch마다) | `train.py` `val_n_passes=3` (multi-pass averaging) |
| **28** | 🔥🔥🔥 **이미지 pixel range가 두 번 scaling됨** — `process_image`(`pixel_range='-11'`)이 [0,1]→[-1,1] 변환 후 `modeling_pi05._preprocess_images`이 또 `img*2-1` 적용 → **SigLIP이 [-3, 1]을 받음** (pi05_base는 [-1,1] 기대) | 🔥🔥🔥 ckpt 93~99 모든 LoRA 학습이 OOD 이미지에 적응하느라 saturation. ckpt 99 loss 0.05 plateau의 **진짜 원인** | `modeling_pi05.py:1231` 중복 `*2-1` 제거 |
| 29 | EMA가 LoRA에서도 활성화됨 (openpi LoRA preset은 `ema_decay=None`) | 🔥 LoRA-B가 0 init이라 EMA(0.99)가 trained delta를 0 쪽으로 끌어당김 → 학습 신호 억제 | `train.py` `use_peft=True`면 EMA 기본 0 |
| **30** | 🔥🔥🔥 **Inference 시 이미지가 BGR로 모델에 들어감** — `ros_image_to_numpy()`가 항상 BGR 반환 (cv2 convention), 학습 데이터는 RGB로 저장 (`lerobot_io.py:441` `cv2.cvtColor(BGR2RGB)` 적용 후 mp4/png write). 그러나 `checkpoint_test.py`는 BGR→RGB 변환 없이 `process_image` → PIL.fromarray(RGB로 해석) → R/B 채널 swap된 채 SigLIP 입력 | 🔥🔥🔥 빨간 큐브가 모델에 파란 큐브로 보임. **ckpt 100이 transition diagnostic은 통과 (진단은 dataset RGB 읽음) 하지만 실제 robot은 cube 못 찾는 진짜 원인.** Export template (`ros_inference.py:269-272`)에는 fix 있는데 live path에 빠짐 | `checkpoint_test.py` 두 위치에 `image[:, :, ::-1]` BGR→RGB flip 추가 |
| **31** | 🔥🔥🔥 **wrist 카메라 augmentation infrastructure가 wire-through 안 됨** — audit-doc #24에서 `EpisodicDataset(wrist_sensor_ids=...)` infrastructure는 추가됐지만 `load_data()` 시그니처에 누락, 호출 사이트도 누락 → 항상 default 빈 set → wrist cam도 full crop+rotate aug 받음 | 🔥🔥🔥 wrist cam은 end-effector geometry 인코딩 — random crop이 gripper↔scene spatial prior 파괴 → fine cube approach 학습 실패. **openpi 100% 성공 vs EasyTrainer 10% 미만 격차의 핵심 원인** | `load_data()` + `train.py` + UI에 `wrist_sensor_ids` wire-through 완료 |
| 32 | `prepare_pi05_language_tokens` dead code call이 `forward_pass`/`checkpoint_test.py`에 있음 — preprocessor가 overwrite하지만 fix #23 우회 위험 | ⚠️ Foot-gun (현재는 dead). 누군가 preprocessor short-circuit하면 silent로 min-max+padded prompt로 회귀 | `forward_pass`/`checkpoint_test.py`의 호출 제거 + preprocessor=None fallback만 유지 |

## 3. 의도적 차이 (버그 아님)

| 항목 | openpi | EasyTrainer | 비고 |
|------|--------|-------------|------|
| Pre-training stage | FAST 토큰 280k step | 생략, pi05_base 로드만 | 사용자 reproduce 불가능 |
| Image augmentation | 학습 시 RandomCrop+Rotate+ColorJitter | 동일 | 일치 |
| Optimizer | optax.adamw(b2=0.95, wd=1e-10) | torch.AdamW(b2=0.95, wd=1e-10) | 일치 |
| LR scheduler | warmup + cosine decay | 수동 LambdaLR (동일 로직) | 일치 |
| Flow matching τ | Beta(1.5, 1) × 0.999 + 0.001 | 동일 | 일치 |
| Action mask | `make_bool_mask(6, -1)` | `absolute_action_dims=[6, 7]` 등 | 표현만 다름, 효과 동일 |
| Pixel range | [-1, 1] | [-1, 1] | 일치 |
| Validation | 없음 | 매 epoch (best 추적) | EasyTrainer 장점 |

## 4. 완전히 같지 않은 것 (수정 어려움 / 의도적)

| 항목 | 영향 |
|------|------|
| Trainable params (LoRA 126M vs full 4B) | LoRA는 distribution shift 적응 한계 있음 (이전 17M에서 7.4× 확대됨) |
| Pre-training 안 됨 (pi05_base 그대로) | 큰 generalization 차이 못 메움 |
| 데이터셋 로더 구조 | EasyTrainer가 episode-based, openpi는 frame-based |
| state token format | 둘 다 256-bin discretize, openpi는 control_mode token 추가 |
| Total optimizer steps | EasyTrainer는 같은 "epoch 수"로 훨씬 적은 step 학습 |

## 5. 권장 설정 (openpi-equivalent에 가장 가까움, 2026-04-29 업데이트)

### Policy
```yaml
use_relative_actions: true
absolute_action_dims: [6, 7]  # 6-DOF arm + gripper(6) + done(7)
freeze_vision_encoder: true   # train.py가 LoRA target에 vision_tower attn+MLP 자동 추가
train_expert_only: true       # paligemma 백본 freeze. PEFT가 LoRA target만 학습하게 함
gradient_checkpointing: true
chunk_size: 50                # paper-equivalent (이전 20에서 변경)
n_action_steps: 50
paligemma_variant: gemma_2b
action_expert_variant: gemma_300m
max_state_dim: 32
max_action_dim: 32
compile_model: false          # PEFT + 학습 안정성 위해 false 유지
```

### Train
```yaml
batch_size: 8
grad_accum_steps: 1           # 4로 올렸다가 ckpt 94 망가짐 — 1 유지 권장
use_amp: true
use_peft: true
peft_r: 64
peft_alpha: 64                # ★ 128 → 64 (scale α/r=1.0, openpi 매칭)
optimizer_lr: 2.5e-5
optimizer_weight_decay: 1e-10
optimizer_betas: [0.9, 0.95]
optimizer_eps: 1e-8
optimizer_grad_clip_norm: 1.0
scheduler_warmup_steps: 100~200
scheduler_decay_steps: 5000~10000   # num_epochs와 dataset 크기에 맞춰 조정
scheduler_decay_lr: 2.5e-6
num_epochs: 300~500           # episode-based이므로 충분한 step 수 확보 (ckpt 103: 500)
num_workers: 1~4
# EMA + smoothed validation (2026-04-28 추가)
ema_decay: 0                  # ★ LoRA에서는 0 (openpi LoRA preset 매칭)
val_smooth_window: 5          # 5-epoch rolling avg로 best epoch 선택 (outlier 무시)
val_n_passes: 3               # multi-pass val로 noise std 1/√3 감소
# Wrist camera 차별 augmentation (2026-04-28 추가)
wrist_sensor_ids: "2"         # ★ 사용자의 wrist-mounted sensor ID (콤마 sep, 예: "2,3")
```

### 양팔 로봇이라면
```yaml
absolute_action_dims: [6, 13, 14]  # 2 grippers + done
# 또는 더 명시적으로
relative_action_mask: [true, true, true, true, true, true, false,
                       true, true, true, true, true, true, false,
                       false]
```

## 6. 핵심 결론 (2026-04-28 업데이트)

**남은 진짜 큰 차이는 두 가지**:

1. **Total training step 수** — openpi는 수천~수만 step, EasyTrainer는 episode-based라 같은 epoch 수에서 훨씬 적은 step 학습. 더 많이 돌려야 함.
2. **Trainable capacity** — LoRA 126M vs full FT 4B. distribution shift가 크면 LoRA로 따라가기 한계 (이전 17M에서 7.4× 확대됐음).

§10의 5-step 재구성으로 "EasyTrainer가 다 freeze + LoRA만 학습"이라는 카테고리 차이는 해결됨. 이제 openpi와 같은 하이브리드 recipe (LLM body LoRA + vision/projector/heads full FT) 사용.

## 7. 디버깅 시 체크리스트 (2026-04-29 업데이트)

학습 시작 시 다음 로그가 모두 떠야 정상:

1. **`[CONFIG] wrist_sensor_ids: [N] (skip spatial aug)`** — wrist 카메라 ID가 정상 wire-through됐는지 (안 뜨면 모든 cam에 random crop+rotate 적용 = bug #31 재발)
2. **`[TRAIN] pi05_base loaded. missing=0 unexpected=0`** — 사전학습 weights 정상 로드
3. **`[TRAIN] LoRA targets: 414 modules`** — gemma_expert(126) + LM attn+MLP(126) + vision_tower attn(108) + **vision_tower MLP(54, 2026-04-28 추가)** 합계
4. **`[TRAIN] Full-FT params (no LoRA): 4.53M`** — multi_modal_projector + action_in/out_proj + time_mlp_*가 LoRA가 아닌 full FT
5. **`trainable params: ~145M`** — 3.5% 수준. ckpt 93 baseline 17M (0.4%) → ckpt 96 126M → ckpt 103 145M
6. **EMA 비활성화 (LoRA의 경우)**: `[TRAIN] EMA enabled` 로그가 **안 보여야** 정상. 떠 있으면 ema_decay가 0이 아닌 것
7. **첫 epoch loss < 1.5**: pi05_base가 정상 작동 (high이면 입력 OOD — image 채널 swap 또는 pixel range 확인)

추론 시 확인:
8. **`[INFER] Loaded preprocessor/postprocessor`** — processor 로드 성공 (None이면 raw state로 prompt 만들어 silent OOD)
9. **이미지 BGR→RGB flip 적용됐는지** — `checkpoint_test.py`의 `image[:, :, ::-1]` 코드 존재 (bug #30)

추가 검증 (진단 스크립트):
10. **diagnose_vision_grounding.py**: `real_B vs zeros` diff가 `real_B vs A`보다 명확히 커야 (vision이 information-bearing). 그 반대면 vision이 noise amplify 중
11. **diagnose_pi05_transition.py**: norm_err mean < 0.10이 목표. 0.15+ 면 transition 학습 부실
12. **diagnose_pi05_replay.py**: 학습 데이터 재현 norm_err < 0.10. 0.20+ 면 학습 자체 실패 또는 inference pipeline bug
13. **stats 확인**: q01/q99이 sane한 범위인지, done token clamp 적용됐는지

## 8. 관련 파일 (변경된 것들, 2026-04-29 갱신)

### Backend
- `src/backend/policies/utils.py` — process_image(pixel_range), get_norm_stats(chunk-wise delta), forward_pass(prepare_pi05_language_tokens fallback only), **load_data(wrist_sensor_ids 추가, 2026-04-28)**, EpisodicDataset(_image_augment_full vs _image_augment_color_only 분기, wrist_sensor_ids 인자), make_easytrainer_processors
- `src/backend/scripts/train.py` — pi05_base 로드, **LoRA target 414 modules (2026-04-28: vision_tower attn+MLP, paligemma LM attn+MLP, gemma_expert attn+MLP)**, **full-FT 모듈 manual unfreeze (multi_modal_projector + action/time heads)**, **EMA shadow weights (use_peft=True면 default 0)**, **smoothed val_loss + multi-pass val**, **wrist_sensor_ids pop + load_data 전달**, scheduler/clip, AdamW
- `src/backend/api/process/checkpoint_test.py` — pixel_range 적용, **BGR→RGB flip 두 위치 (2026-04-28, bug #30)**, task key 매핑, language_instruction, **prepare_pi05_language_tokens dead call 제거 (bug #32)**, pi05_chunk_queue (relative actions chunk-anchored)
- `src/backend/api/routes/checkpoint.py` — language_instruction payload
- `src/backend/scripts/diagnose_vision_grounding.py` — **신규 진단 스크립트** (vision conditioning 정량 측정)
- `src/backend/scripts/diagnose_pi05_transition.py` — **신규** (transition frame norm_err + endpoint diversity)
- `src/backend/scripts/diagnose_pi05_replay.py` — **신규** (학습 데이터 재현 정확도)

### Lerobot 내부
- `src/backend/lerobot/src/lerobot/policies/pi05/configuration_pi05.py` — vocab(257152), dtype(bfloat16), action mask config (absolute_action_dims, relative_action_mask)
- `src/backend/lerobot/src/lerobot/policies/pi05/modeling_pi05.py` — embed scaling 제거, vocab 257152, **`_preprocess_images`의 중복 `*2-1` 제거 (bug #28, 2026-04-28)**
- `src/backend/lerobot/src/lerobot/policies/pi05/processor_pi05.py` — RelativeActionsProcessorStep config plumb, **`Pi05PrepareStateTokenizerProcessorStep.state_dim` 추가 (native dim truncate, bug #23)**
- `src/backend/lerobot/src/lerobot/policies/pi_gemma.py` — meta device materialize
- `src/backend/lerobot/src/lerobot/processor/relative_action_processor.py` — absolute_action_dims, relative_action_mask, dual-arm 지원
- `src/backend/lerobot/src/lerobot/processor/normalize_processor.py` — quantile clamp

### UI
- `src/ui/src/configs/modelConfigs.js` — PI05 정책 설정 (absolute_action_dims, **peft_alpha=peft_r 기본값**, **ema_decay/val_smooth_window/val_n_passes**, **wrist_sensor_ids 텍스트 필드**)
- `src/ui/src/pages/v2/TrainPage.vue` — array type, showIf 지원
- `src/ui/src/components/v2/MonitoringWindow.vue` — language prompt input
- `src/ui/src/components/v2/FormDialog.vue` — placeholder 지원

## 9. openpi 경로 (vendored)

`src/backend/lerobot/src/lerobot/policies/openpi_train/` 아래에 전체 openpi 코드 vendored됨.

핵심 비교 대상:

| 파일 | 역할 |
|------|------|
| `src/openpi/models/pi0.py` | flow matching loss, embed_prefix/suffix |
| `src/openpi/models/model.py` | image preprocessing ([-1,1]), augmentation |
| `src/openpi/training/config.py` | DataConfig — `use_delta_joint_actions`, `make_bool_mask` 등 |
| `src/openpi/training/data_loader.py` | dataset transform pipeline (DeltaActions before Normalize) |
| `src/openpi/training/optimizer.py` | AdamW + warmup_cosine_decay (optax) |
| `src/openpi/transforms.py` | Normalize / DeltaActions / AbsoluteActions / RepackTransform |
| `src/openpi/policies/piper_policy.py` | piper 데이터 PiperInputs/Outputs (adapt_to_pi 변환) |
| `scripts/train.py` | train_step (loss, grad, jit) |
| `scripts/compute_norm_stats.py` | norm stats 사전 계산 (chunk 단위 transform 적용 후) |

## 10. LoRA Recipe 5-Step 재구성 (2026-04-28)

ckpt 93/94까지의 LoRA recipe는 "다 freeze + LoRA만"으로 openpi와 카테고리 자체가 달랐음. ckpt 94에서 multi_modal_projector LoRA만 추가하니 vision이 noise를 amplify하면서 robot이 학습 데이터에 없는 행동하기 시작 → 코드 레벨 deep audit으로 차이점 발견 → 5-step 재구성.

### 발견한 카테고리 차이

openpi `gemma_2b_lora` freeze filter는 `nnx.All('*llm*', Not('*lora*'))` — **이름에 `llm`이 있는 모듈만 freeze + LoRA 적용**. SigLIP은 `PaliGemma/img/...` 경로라 freeze filter에 안 걸려 **full FT**됨. 결과:

| 모듈 | openpi gemma_2b_lora | EasyTrainer (~ckpt 94) | 새 recipe (2026-04-28~) |
|------|---------------------|----------------------|------------------------|
| `vision_tower` (SigLIP) | **Full FT** | ❌ Frozen | LoRA r=64 (attn) |
| `multi_modal_projector` | **Full FT** | LoRA r=64 | **Full FT** |
| `language_model.attn` | LoRA r=16 | LoRA r=64 | LoRA r=64 |
| `language_model.mlp` | **LoRA r=16** | ❌ 없음 | **LoRA r=64** |
| `gemma_expert.attn+mlp` | LoRA r=32 | LoRA r=64 | LoRA r=64 |
| `action_in/out_proj`, `time_mlp_*` | **Full FT** | LoRA r=64 | **Full FT** |
| LoRA scale (α/r) | **1.0** | **2.0** | **1.0** |
| LoRA dropout | 0.0 | 0.05 | 0.0 |
| Wrist cam aug | ColorJitter only | full aug | 인프라만 (사용자 설정 필요) |
| Prompt state dim | native (~7) | padded (32) | native (~7) |

### 5-Step 변경 내역

| Step | 우선순위 | 파일 | 변경 |
|------|---------|------|------|
| 1 | 🔥🔥 | `scripts/train.py:285-373` | LoRA target 재구성 — vision_tower attn LoRA + LM MLP LoRA 추가, 작은 모듈은 LoRA 제거 후 full-FT, alpha=r 기본, dropout=0.0 |
| 2 | 🔥 | `scripts/train.py:481-485` | `best_state_dict_cpu`를 `requires_grad` 기반으로 — full-FT 모듈도 best epoch 가중치 보존 |
| 3 | ⚠️ | `modelConfigs.js:198-199` | `peft_alpha` 기본값 128 → 64 |
| 4 | ⚠️ | `processor_pi05.py:49-90, 173-180` | `Pi05PrepareStateTokenizerProcessorStep.state_dim` 필드 추가, native dim으로 truncate |
| 5 | ℹ️ | `policies/utils.py:89, 107-125, 305-321` | `wrist_sensor_ids` 인프라 추가 (default 빈 set, backward compat) |

### Trainable param 분포 변화

| | ckpt 93/94 | 새 recipe |
|---|-----------|----------|
| LoRA gemma_expert | ~12M | 27.7M |
| LoRA paligemma LM attn | ~5M | 14.8M |
| LoRA paligemma LM MLP | 0 | **63.7M** (NEW) |
| LoRA vision_tower attn | 0 | **15.9M** (NEW) |
| Full FT projector + heads | 0 | **4.5M** (NEW) |
| **Total** | **~17M** | **126.6M** (7.4×) |
| Optimizer state 추가 | ~270MB | ~2GB (+1.7GB) |

### 검증 / 진단

새 학습 후 `diagnose_vision_grounding.py` 실행해서 다음 신호 확인:

| 척도 | 실패 (ckpt 94) | 성공 (목표) |
|------|---------------|------------|
| step-0 vision/state ratio | 0.031 | > 0.2 |
| `real_B vs zeros` diff | 0.31 (≥ real_B) | < real_B × 0.5 |
| per-dim diff vs q01/q99 범위 | ~100% (OOD) | < 30% |
| gripper/done dim 차이 | 0 (mode collapse) | non-zero |

### 왜 ckpt 94에서 vision noise가 amplify됐나 (재해석)

1. multi_modal_projector LoRA만 추가됐는데 vision_tower는 여전히 frozen → projector가 (frozen vision feature를) (frozen LM space로) 매핑하는 작은 LoRA delta 학습
2. LoRA scale=2.0 → 그 작은 delta가 ×2로 증폭
3. Prompt에 padding noise + LM MLP에 LoRA 없음 → prefix가 task-adapted representation 만들 capacity 부족
4. action heads도 LoRA로 묶여 있어서 새 vision signal에 동적 적응 못 함
5. step 수까지 부족 (grad_accum=4 + epoch 120) → noise basin에 빠진 LoRA가 거대한 magnitude로 vision을 amplify

→ "vision-blind 일관됨"보다 "vision-noisy chaotic"이 inference 행동상 더 나쁨. ckpt 93보다 ckpt 94가 나쁜 이유.

## 11. 후속 변경 (ckpt 96~103, 2026-04-28~29)

§10의 5-step 재구성 이후에도 transition error가 충분히 안 떨어지고 robot이 cube 못 찾는 문제 지속 → 추가 audit 2회 → 4개 큰 fix 발견.

### 후속 fix 목록

| Step | 우선순위 | 발견 시점 | 파일 | 변경 |
|------|---------|----------|------|------|
| 6 | 🔥🔥 | ckpt 96 진단 후 | `scripts/train.py` LoRA target | **vision_tower MLP까지 LoRA 추가** (54 modules, +18.86M trainable). 이전엔 attn만 |
| 7 | 🔥 | ckpt 99 분석 | `scripts/train.py` | **EMA shadow weights** + **smoothed val_loss (5-window)** + **multi-pass val (n=3)**. ckpt 96 best epoch이 outlier dip(epoch 98 val=0.014 vs 주변 0.07)에서 잡힌 것 발견 |
| 8 | 🔥🔥🔥 | ckpt 99 deep audit | `modeling_pi05.py:1231` | **이미지 pixel range 중복 *2-1 제거 (bug #28)** — process_image이 [-1,1] 변환 후 modeling_pi05이 또 *2-1 → SigLIP에 [-3,1]. ckpt 93~99 saturation의 진짜 원인 |
| 9 | 🔥 | ckpt 99 deep audit | `train.py:105-110` | **EMA를 LoRA 학습에서 default 0** (bug #29). openpi LoRA preset이 모두 ema_decay=None |
| 10 | 🔥🔥🔥 | ckpt 100 inference audit | `checkpoint_test.py:343-364, 555-575` | **BGR→RGB flip 추가 (bug #30)**. ros_image_to_numpy는 BGR 반환, 학습 데이터는 RGB로 저장. 빨간 큐브가 모델에 파란 큐브로 보임 |
| 11 | 🔥🔥🔥 | ckpt 103 deep audit | `policies/utils.py`, `train.py`, `modelConfigs.js` | **wrist_sensor_ids wire-through (bug #31)**. infrastructure는 fix #5에서 추가됐지만 load_data까지 연결 안 됨 → 모든 cam에 random crop+rotate. wrist cam은 end-effector geometry 인코딩이라 crop이 spatial prior 파괴 |
| 12 | ⚠️ | ckpt 103 audit | `policies/utils.py:1017`, `checkpoint_test.py:331` | **prepare_pi05_language_tokens dead call 제거 (bug #32)**. preprocessor가 overwrite하지만 fix #23 우회 foot-gun |

### 최종 trainable param 분포 (ckpt 103)

| | ckpt 93/94 | ckpt 96~99 | **ckpt 103+** |
|---|-----------|-----------|--------------|
| LoRA gemma_expert (attn+mlp) | ~12M | 27.7M | 27.7M |
| LoRA paligemma LM attn | ~5M | 14.8M | 14.8M |
| LoRA paligemma LM MLP | 0 | 63.7M | 63.7M |
| LoRA vision_tower attn | 0 | 15.9M | 15.9M |
| **LoRA vision_tower MLP** | 0 | 0 | **18.9M (NEW)** |
| Full FT projector + heads | 0 | 4.5M | 4.5M |
| **Total trainable** | ~17M | 126.6M | **145.5M (8.6×)** |

### ckpt 진화: 진단 결과 추이

| ckpt | 주요 변경 | Transition norm_err | endpoint dist | 실제 robot |
|------|----------|--------------------|--------------:|-----------|
| 93 | 5-step 재구성 전 baseline | 0.122 | 0.21 | cube blind, mean trajectory |
| 94 | + multi_modal_projector LoRA만 | 0.159 | 0.365 | erratic, vision noise amplify |
| 96 | + 5-step 재구성 (vision LoRA) | 0.122 | 0.21 | 회복, 여전히 cube 못 찾음 |
| 99 | + EMA, smoothed val | 0.159 | 0.365 | 후퇴 (EMA가 LoRA 억제) |
| 100 | + image *2-1 fix (#28), EMA off | 0.107 | 0.21 | 약간 개선, 여전히 cube blind |
| 103 | + BGR/RGB fix (#30), wrist wire (#31), vision_tower MLP, 500 epoch | 진단 보류 | — | **진행 중** |

### 4개 결정타 (impact 큰 순)

1. **bug #30 BGR/RGB swap** — 빨간 큐브가 파란 큐브로 보임. **모든 ckpt가 inference에서 wrong color**.
2. **bug #31 wrist aug wire-through** — wrist cam이 random crop으로 gripper geometry 파괴. End-effector localization 학습 실패. **openpi 100% vs 우리 10% 격차 핵심**.
3. **bug #28 image double *2-1** — SigLIP이 [-3, 1] 받음. pi05_base [-1, 1] 사전학습과 OOD. 모든 ckpt 93~99 영향.
4. **bug #29 EMA on for LoRA** — LoRA-B가 0 init이라 EMA(0.99)가 trained delta를 0으로 끌어당김.

이 네 개 + §10 5-step 재구성 = openpi-equivalent recipe.

## 12. 운영 메모

- **GPU OOM 학습 실패 시**: backend가 inference 모델을 GPU에서 안 내릴 수 있음 (~9GB 점유). `docker exec easy_collector_service kill 116 3271774`로 backend 재시작 (autoreload).
- **새 학습 전 EMA 확인**: DB 또는 UI에서 `ema_decay`를 명시적으로 0으로 설정 (default가 0이지만 이전 값이 남아있을 수 있음).
- **Wrist sensor ID 결정**: 데이터셋의 sensor 영상에서 gripper가 항상 비슷한 위치에 보이는 카메라.
- **체크리스트 우선순위**: §7 1번(wrist_sensor_ids 로그) → 3번(LoRA targets 414) → 5번(trainable 145M) → 9번(BGR→RGB flip 코드 존재).

## 13. 참고 자료

- [PI0.5 Paper](https://arxiv.org/abs/2504.16054)
- [openpi GitHub](https://github.com/Physical-Intelligence/openpi)
- [lerobot/pi05_base on HuggingFace](https://huggingface.co/lerobot/pi05_base)

진단 스크립트 (모두 `src/backend/scripts/`):
- `diagnose_vision_grounding.py` — vision conditioning 정량 측정 (real vs zeros vs random)
- `diagnose_pi05_transition.py` — episode 가로질러 transition frame norm_err + endpoint diversity
- `diagnose_pi05_replay.py` — 학습 데이터 자체 재현 정확도
