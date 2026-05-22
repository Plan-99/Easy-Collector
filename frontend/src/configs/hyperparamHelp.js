// Hyperparameter help text per policy / parameter, in Korean and English.
//
// Lookup: HYPERPARAM_HELP[policyType + '.' + key]  OR  HYPERPARAM_HELP['Common.' + key]
// Each entry has { ko: {title, easy, expert, note?}, en: {title, easy, expert, note?} }.
// - easy   : 비전공자도 이해할 수 있는 한 줄~짧은 단락 설명
// - expert : 정확한 기술적 설명 (모델/학습 동작 기준)
// - note   : 선택, 주의사항 또는 "추후 구현" 표시
// 본문은 plain text — 빈 줄(\n\n)이 단락 구분으로 렌더링됩니다.

export const HYPERPARAM_HELP = {
  // ============================================================
  // ACT — model config
  // ============================================================
  'ACT.n_obs_steps': {
    ko: {
      title: '관찰 스텝 수 (n_obs_steps)',
      easy: '로봇이 한 번 행동을 결정할 때, "지금 이 순간"의 영상 한 장만 볼지, 직전 몇 장도 같이 볼지 정합니다. ACT는 한 장만 보도록 만들어진 모델이라 그대로 1로 두시면 됩니다.',
      expert: '정책이 한 번 forward할 때 입력하는 과거 관찰 프레임 길이입니다. ACT 논문 구조상 단일 프레임 전제로 설계되어 있어 코드에서도 1만 허용합니다 (config __post_init__에서 검증). 늘려도 transformer 입력 토큰만 늘고 의미 있는 성능 이득은 없습니다.',
      note: '현재 ACT는 n_obs_steps=1만 유효 — 다른 값을 넣으면 학습 시작 시 에러로 막힙니다. 1 그대로 두세요.',
    },
    en: {
      title: 'Observation Steps (n_obs_steps)',
      easy: 'How many recent video frames the robot looks at to decide its next move. ACT is designed to use only the current frame, so leave this at 1.',
      expert: 'Number of past observation frames fed into the policy per forward pass. ACT is built around a single-frame transformer encoder, and the codebase enforces n_obs_steps=1 in config __post_init__. Increasing it only inflates token count without measurable gains.',
      note: 'Only n_obs_steps=1 is currently valid — other values raise an error at training start. Keep at 1.',
    },
  },

  'ACT.chunk_size': {
    ko: {
      title: '청크 크기 (chunk_size)',
      easy: '"앞으로 몇 번의 동작을 한꺼번에 머릿속에 그려놓고 움직일지" 정합니다. 값이 크면 부드럽고 시연을 잘 따라가지만, 새로운 상황에 빠르게 반응하기 어렵습니다.\n\n빠른 작업이면 15~30, 느리고 정밀한 작업이면 50~100 정도가 보통입니다.',
      expert: 'ACT의 핵심 아이디어인 action chunking의 길이 — 한 번의 추론으로 미래 N개 액션을 통째로 예측합니다. decoder 위치 임베딩(config.chunk_size)에 직접 사용됩니다.\n\n값이 크면 시연과의 fidelity는 좋아지지만 temporal ensembling이 강해져 환경 변화에 reactive하지 않습니다. 제어 주기(Hz)와 작업의 시간 스케일에 맞춰 조정하세요.',
    },
    en: {
      title: 'Chunk Size',
      easy: 'How many future actions the robot plans in one shot. Larger values give smoother, demo-faithful motion but slower reactions to surprises.\n\nFor fast tasks try 15-30; for slow, precise tasks 50-100 works well.',
      expert: 'Length of the action chunk that ACT predicts in one forward pass — the core idea of action chunking. Directly drives the decoder positional embedding (config.chunk_size).\n\nLarger chunks improve demo fidelity but, combined with temporal ensembling, make the policy less reactive to scene changes. Match this to your control frequency and task time-scale.',
    },
  },

  'ACT.n_action_steps': {
    ko: {
      title: '실행 액션 수 (n_action_steps)',
      easy: '한 번 머릿속에 그린 동작 중에서 "실제로 몇 개나 따라 할지" 정합니다. 보통 1로 두고 매번 새로 생각하게 합니다.',
      expert: '예측된 chunk에서 실제로 환경에 보낼 액션 개수. action_queue 크기로 사용되며, temporal ensembling이 켜져 있으면 1로 강제됩니다.\n\n늘리면 추론 호출 빈도가 줄어 throughput은 좋아지지만 reactivity가 떨어집니다. chunk_size 이하 값만 허용됩니다.',
    },
    en: {
      title: 'Action Steps (n_action_steps)',
      easy: 'Of the actions the robot plans ahead, how many it actually executes before re-planning. Usually 1 (always re-plan).',
      expert: 'Number of actions consumed from the predicted chunk before the next inference call. Used as the action_queue length; forced to 1 when temporal ensembling is on.\n\nIncreasing it reduces inference frequency at the cost of reactivity. Must be ≤ chunk_size.',
    },
  },

  'ACT.temporal_ensemble_coeff': {
    ko: {
      title: '시간 앙상블 계수 (temporal_ensemble_coeff)',
      easy: '서로 다른 시점에 계획된 동작들을 부드럽게 섞어주는 정도입니다. 비워두면 섞지 않고 그대로 실행해요.',
      expert: '서로 다른 timestep에 예측된 액션들을 exp(-coeff × i) 가중치로 부드럽게 평균냅니다. 값이 작을수록(0.01) 천천히 변하는 부드러운 동작, 클수록 최신 추론을 따릅니다.\n\n비워두면 ACTTemporalEnsembler를 생성하지 않고 action_queue 방식으로 chunk를 그대로 실행. ACT 논문 기본값 0.01.',
      note: '이 값을 설정하면 n_action_steps는 자동으로 1로 강제됩니다.',
    },
    en: {
      title: 'Temporal Ensemble Coefficient',
      easy: 'How strongly to blend together actions planned at different moments. Leave empty to skip blending and execute the chunk raw.',
      expert: 'Exponentially weighted average of actions predicted at different timesteps, with weights exp(-coeff × i). Lower (0.01) = slower-changing, smoother motion; higher = follow latest inference.\n\nLeaving this empty skips ACTTemporalEnsembler entirely and uses the action_queue path. ACT paper default 0.01.',
      note: 'Setting this value forces n_action_steps to 1.',
    },
  },

  'ACT.vision_backbone': {
    ko: {
      title: '비전 백본 (vision_backbone)',
      easy: '카메라 영상을 "이해해서 숫자로 바꿔주는" 신경망입니다. 작은 데이터셋이면 resnet18(가벼움)이 안전하고, 데이터가 많고 일반화가 중요하면 DINO 계열을 시도해볼 수 있어요. DINO 중에서는 dinov2-small(작고 빠름)이 일반적인 기본 추천 — dinov2(base) 대비 3~4배 빠른데 정확도 차이는 미미합니다.',
      expert: '이미지를 임베딩 벡터로 변환하는 CNN/ViT.\n\n- resnet18/34/50: torchvision 표준 CNN (`getattr(torchvision.models, backbone_name)`)\n- dinov2-small: 자기지도 ViT, hidden_size=384, ~22M params. DINO 중 가장 가벼움, 학습 속도/정확도 균형이 좋아 DINO 기본 추천.\n- dinov2: hidden_size=768, ~86M params. dinov2-small 보다 표현력은 강하지만 학습이 3~4배 느리고 VRAM 큼.\n- dinov3: HF gated repo (token 필요), 현재 실험적.\n\nDINO 계열은 HuggingFace `AutoModel` 로 직접 로드하며 `pretrained_backbone_weights` 는 무시됩니다 (UI 자동 숨김). 대신 `n_unfrozen_blocks` 로 partial unfreeze 가능.',
    },
    en: {
      title: 'Vision Backbone',
      easy: 'The neural network that turns camera frames into numbers the policy can understand. Use resnet18 for small datasets; try a DINO backbone when generalization matters. Within DINO, dinov2-small is the recommended default — 3-4x faster than dinov2(base) with negligible accuracy loss on most tasks.',
      expert: 'CNN/ViT that produces image embeddings.\n\n- resnet18/34/50: standard torchvision CNN (`getattr(torchvision.models, backbone_name)`)\n- dinov2-small: self-supervised ViT, hidden_size=384, ~22M params. Lightest DINO variant, best speed/accuracy trade-off, recommended default for DINO.\n- dinov2: hidden_size=768, ~86M params. Stronger features but ~3-4x slower forward/backward and higher VRAM.\n- dinov3: gated HF repo (token required), experimental.\n\nDINO variants load via HuggingFace `AutoModel` and ignore `pretrained_backbone_weights` (auto-hidden in UI). Use `n_unfrozen_blocks` for partial unfreeze instead.',
    },
  },

  'ACT.pretrained_backbone_weights': {
    ko: {
      title: '백본 사전학습 가중치 (ResNet 전용)',
      easy: 'ResNet 백본을 ImageNet 사전학습 가중치로 시작할지 정합니다. 거의 항상 켜두는 게 좋습니다 (None 으로 두면 random 초기화).',
      expert: 'torchvision 의 `weights=` 인자로 그대로 전달됩니다. e.g. `ResNet18_Weights.IMAGENET1K_V1` 이면 ImageNet1K 분류로 학습된 conv 가중치를 받아 backbone 을 초기화. `None` 으로 두면 random init.',
      note: 'DINOv2/v3 백본에서는 무시됩니다 — DinoVisionBackbone 은 HuggingFace AutoModel 로 자체적으로 사전학습 weight 를 받아 옵니다. 그래서 dinov* 선택 시 이 필드는 자동 숨김.',
    },
    en: {
      title: 'Pretrained Backbone Weights (ResNet only)',
      easy: 'Whether to initialize the ResNet backbone from ImageNet weights. Almost always yes — set to `None` only if you specifically want random init.',
      expert: 'Passed verbatim as torchvision `weights=`. e.g. `ResNet18_Weights.IMAGENET1K_V1` loads ImageNet1K-pretrained conv weights into the backbone. `None` → random init.',
      note: 'Ignored on DINOv2/v3 backbones — DinoVisionBackbone loads its own pretrained weights via HuggingFace AutoModel, so this field is hidden when a dinov* backbone is selected.',
    },
  },

  'ACT.n_unfrozen_blocks': {
    ko: {
      title: 'DINO 학습 가능 블록 수 (n_unfrozen_blocks)',
      easy: 'DINO 비전 백본의 뒤쪽 N개 transformer 블록만 추가로 학습합니다. 0이면 백본을 그대로 두고 사용(빠름·메모리 적음). 케이블 삽입처럼 위치 정밀도가 중요한 작업에서 0으로 성능이 부족하면 2~4부터 조금씩 풀어보세요.',
      expert: 'DINOv2/v3 trunk의 마지막 N개 transformer 블록과 final LayerNorm만 학습 가능 상태로 전환합니다. 그 외 블록은 frozen + eval()-pinned. 0이면 fully frozen (기본). vision_backbone이 resnet*일 때는 무시됩니다.\n\n전형적인 트레이드오프:\n- 0: representation 완전 보존, 학습 비용 최소. self-sup invariance가 정밀 위치 단서를 깎을 수 있음.\n- 2~4: 마지막 블록만 task 데이터로 미세조정. ResNet 전체 학습보다 훨씬 적은 파라미터로 fine-grained 위치 적응 가능.\n- 12 (전체): 사실상 full fine-tune. 큰 데이터셋(>100 episodes) 권장.\n\n학습 가능한 파라미터는 ACT의 backbone optimizer group(`optimizer_lr_backbone`)으로 자동 흡수됩니다.',
      note: 'ResNet 백본일 때는 화면에 보이지 않습니다.',
    },
    en: {
      title: 'DINO Unfrozen Blocks (n_unfrozen_blocks)',
      easy: 'How many transformer blocks at the END of the DINO trunk to fine-tune. 0 keeps the backbone fully frozen (fast, low VRAM). For precision tasks like cable insertion where 0 underperforms, try 2-4 to gently adapt the backbone to your camera view.',
      expert: 'Unfreezes the last N transformer blocks plus the final LayerNorm of the DINOv2/v3 trunk; everything else stays frozen and eval()-pinned. 0 = fully frozen (default). Ignored when vision_backbone starts with "resnet".\n\nTypical trade-off:\n- 0: representation fully preserved, minimal training cost. Self-supervised invariance may smear fine spatial cues.\n- 2-4: late blocks adapt to task data. Far fewer trainable params than a full ResNet, while regaining fine-grained spatial precision.\n- 12 (all): essentially full fine-tune. Recommended only for large datasets (>100 episodes).\n\nTrainable params are automatically picked up by ACT\'s backbone optimizer group (`optimizer_lr_backbone`).',
      note: 'Hidden in the UI when a ResNet backbone is selected.',
    },
  },

  'ACT.replace_final_stride_with_dilation': {
    ko: {
      title: 'Final Stride → Dilation 치환',
      easy: '비전 신경망이 작은 물체 위치도 정확히 잡도록 해주지만, 메모리와 학습 시간이 4배 가까이 늘어납니다. 보통 끄세요.',
      expert: 'ResNet 마지막 stage의 stride-2 conv를 dilation-2 conv로 치환해 feature map 해상도를 두 배 유지합니다. 작은 물체 위치 단서가 중요한 작업에 도움이 되지만, 메모리/연산이 약 4배 증가.\n\n코드에서는 backbone 생성 시 `replace_stride_with_dilation=[F, F, bool]`로 전달됩니다. 기본 false.',
    },
    en: {
      title: 'Replace Final Stride with Dilation',
      easy: "Helps the vision network spot small-object positions, but uses ~4× more memory and time. Usually leave off.",
      expert: "Replaces the last ResNet stage's stride-2 conv with a dilated conv, keeping the feature map at higher resolution. Useful for tasks where small-object localization matters; ~4× memory/compute cost.\n\nPassed to the backbone as `replace_stride_with_dilation=[F, F, bool]`. Default off.",
    },
  },

  'ACT.pre_norm': {
    ko: {
      title: 'Pre-Normalization',
      easy: 'Transformer 내부 정규화를 입력 쪽에 둘지 출력 쪽에 둘지 정하는 옵션입니다. ACT 크기에서는 차이가 거의 없어요. 기본값(false) 그대로 두세요.',
      expert: 'Transformer 블록 안 LayerNorm을 attention/FFN의 입력(true) 또는 출력(false) 쪽에 둡니다. ACTEncoderLayer/ACTDecoderLayer에서 조건부로 적용.\n\npre-norm은 깊은 네트워크 학습 안정성에 유리하지만, ACT는 인코더 4 + 디코더 1로 얕아 영향이 미미합니다.',
    },
    en: {
      title: 'Pre-Normalization',
      easy: "Where the transformer's internal normalization is placed. Makes almost no difference at ACT's size — keep default (false).",
      expert: 'Whether LayerNorm sits before (true) or after (false) the attention/FFN sublayer in each block. Conditionally applied in ACTEncoderLayer/ACTDecoderLayer.\n\nPre-norm helps training stability for deep networks, but ACT is shallow (4 encoder + 1 decoder) so the effect is negligible.',
    },
  },

  'ACT.dim_model': {
    ko: {
      title: '모델 차원 (dim_model)',
      easy: 'Transformer의 "두뇌 크기"입니다. 크면 똑똑하지만 학습 시간과 GPU 메모리가 늘어납니다. 기본 512가 대부분의 작업에 적당해요.',
      expert: 'Transformer hidden state의 차원. 모든 attention/embedding/linear layer의 핵심 크기로 사용됩니다.\n\n512가 표준. 데이터셋이 충분(>200 episodes)할 때만 768/1024를 시도하세요. n_heads로 나누어떨어져야 합니다.',
    },
    en: {
      title: 'Model Dimension',
      easy: "Transformer's 'brain size'. Larger = smarter but slower and more VRAM. Default 512 works for most tasks.",
      expert: 'Hidden dimension of the transformer. Drives the size of every attention/embedding/linear layer.\n\n512 is standard. Only try 768/1024 when your dataset is large (> 200 episodes). Must be divisible by n_heads.',
    },
  },

  'ACT.n_heads': {
    ko: {
      title: 'Attention 헤드 수',
      easy: 'Transformer가 한 번에 몇 가지 관점으로 동시에 정보를 볼지 정합니다. 기본 8이 거의 모든 경우에 잘 동작해요.',
      expert: 'Multi-head attention의 헤드 개수. dim_model이 이 값으로 나누어떨어져야 합니다 (예: 512 / 8 = 64). 8이 거의 모든 경우에 적합.',
    },
    en: {
      title: 'Number of Heads',
      easy: 'How many parallel "viewpoints" the transformer uses to attend. Default 8 covers almost all cases.',
      expert: 'Number of attention heads. dim_model must be divisible by this (e.g. 512 / 8 = 64). 8 works in nearly all cases.',
    },
  },

  'ACT.dim_feedforward': {
    ko: {
      title: 'FFN 차원 (dim_feedforward)',
      easy: 'Transformer 내부의 작은 신경망 너비입니다. 거의 바꿀 일이 없는 값이에요.',
      expert: 'Transformer 안 feed-forward 서브레이어의 hidden 차원. 보통 dim_model × 4~8. ACT 기본값 3200(=512×6.25). 거의 튜닝할 필요 없습니다.',
    },
    en: {
      title: 'Feedforward Dimension',
      easy: "Width of the internal sub-network inside the transformer. Rarely needs tuning.",
      expert: 'Hidden size of the feed-forward sublayer inside each transformer block. Typically dim_model × 4-8. ACT default 3200 (~512 × 6.25). Rarely needs tuning.',
    },
  },

  'ACT.feedforward_activation': {
    ko: {
      title: 'FFN 활성화 함수',
      easy: 'Transformer 안 작은 신경망이 쓰는 "비선형 함수"입니다. relu가 ACT 기본이고 가장 안정적이에요.',
      expert: 'FFN 안의 non-linearity. `get_activation_fn(config_value)`로 동적 선택 — relu/gelu/glu 등.\n\nrelu가 ACT 기본이자 가장 안정. gelu/silu는 표현력이 조금 좋지만 학습이 미세하게 느려질 수 있습니다.',
    },
    en: {
      title: 'Feedforward Activation',
      easy: 'Which "bend" the transformer uses inside its FFN. relu is the safest, default choice.',
      expert: 'Non-linearity inside the FFN, selected dynamically via `get_activation_fn(config_value)`. relu is the ACT default and most stable; gelu/silu can be slightly more expressive but marginally slower.',
    },
  },

  'ACT.n_encoder_layers': {
    ko: {
      title: '인코더 레이어 수',
      easy: '카메라/관절 정보를 "이해하는" Transformer의 두께입니다. ACT는 4가 기본이고 늘려도 큰 이득이 없어요.',
      expert: '비전+상태를 받는 transformer encoder의 깊이. ModuleList 길이로 직접 제어됩니다. ACT 권장값 4 — 8 이상이면 작은 데이터셋에서 과적합 위험이 커집니다.',
    },
    en: {
      title: 'Number of Encoder Layers',
      easy: 'Depth of the transformer that "understands" inputs. ACT uses 4 — going higher rarely helps.',
      expert: 'Depth of the transformer encoder consuming vision + state. Directly drives the ModuleList length. ACT recommends 4 — beyond 8 overfits small datasets.',
    },
  },

  'ACT.n_decoder_layers': {
    ko: {
      title: '디코더 레이어 수',
      easy: '최종 동작을 만들어내는 부분의 두께입니다. ACT는 원래 구현 버그를 우회해서 1만 사용해요.',
      expert: '액션 chunk를 생성하는 transformer decoder의 깊이. 코드 주석에 따르면 원본 구현이 첫 layer만 사용하는 버그가 있어 ACT는 1로 두는 게 정석입니다. 늘려도 측정 가능한 성능 향상이 없습니다.',
    },
    en: {
      title: 'Number of Decoder Layers',
      easy: 'Depth of the part that produces the final actions. ACT uses 1 to work around an original-implementation quirk.',
      expert: 'Depth of the transformer decoder emitting the action chunk. Per code comments, the original implementation uses only the first layer due to a bug — ACT keeps this at 1. Increasing it yields no measurable improvement.',
    },
  },

  'ACT.use_vae': {
    ko: {
      title: 'VAE 사용 (CVAE 인코더)',
      easy: '사람 시연이 "같은 상황에서 다른 행동"으로 갈리는 경우(다양성)를 모델이 학습하게 해주는 옵션입니다. 보통 켜두는 게 좋아요.',
      expert: 'ACT의 conditional VAE 가지를 활성화합니다. 시연의 multi-modality(같은 상태에서 분기되는 액션)를 명시적으로 모델링.\n\nFalse면 평범한 회귀 학습이 되어 학습은 빠르지만 분기 표현이 약해집니다. True일 때만 kl_weight가 손실에 들어가고 KL term이 추가됩니다.',
    },
    en: {
      title: 'Use VAE',
      easy: 'Lets the model handle "different valid actions for the same situation" naturally. Generally on.',
      expert: "Enables ACT's conditional VAE branch, explicitly modeling demo multi-modality (different actions valid in the same state).\n\nFalse reduces ACT to plain regression — faster but worse at branching behavior. kl_weight only affects the loss when this is True.",
    },
  },

  'ACT.latent_dim': {
    ko: {
      title: '잠재 변수 차원 (latent_dim)',
      easy: 'VAE가 동작 다양성을 표현하는 공간의 크기입니다. 32가 표준.',
      expert: 'CVAE latent z의 차원. encoder가 latent_dim×2 projection으로 평균/분산을 출력합니다. 32가 표준 — 늘리면 표현력↑이지만 학습이 어렵고 작은 데이터셋에서는 노이즈만 증가.',
    },
    en: {
      title: 'Latent Dimension',
      easy: 'Size of the space the VAE uses to capture action variety. 32 is standard.',
      expert: 'Dimensionality of the CVAE latent z. The encoder projects to latent_dim×2 for μ/σ. 32 is standard — larger values capture more variation but are harder to train and just add noise on small datasets.',
    },
  },

  'ACT.n_vae_encoder_layers': {
    ko: {
      title: 'VAE 인코더 레이어 수',
      easy: 'VAE 인코더(시연 → 잠재 변수)의 두께. 4가 기본.',
      expert: 'CVAE 인코더(action sequence → z)의 transformer 깊이. 일반 encoder와 별도 ModuleList로 구성됩니다. ACT 권장 4.',
    },
    en: {
      title: 'Number of VAE Encoder Layers',
      easy: 'Depth of the VAE encoder that maps demos to latent space. Default 4.',
      expert: 'Transformer depth of the CVAE encoder mapping action sequences to z. Built as its own ModuleList separate from the main encoder. ACT recommends 4.',
    },
  },

  // ============================================================
  // Diffusion — model config
  // ============================================================
  'Diffusion.n_obs_steps': {
    ko: {
      title: '관찰 스텝 수 (n_obs_steps)',
      easy: '로봇이 행동을 결정할 때 최근 몇 장의 영상을 같이 볼지 정합니다. Diffusion Policy 기본은 2장(직전 + 현재) — 움직임 단서를 잡기 좋아요.',
      expert: 'Diffusion Policy가 1회 추론 시 입력하는 최근 관찰 프레임 수. n_obs_steps-1을 추론 시작 인덱스로 사용해 global_cond를 계산합니다.\n\n논문 기본 2 — 직전+현재로 속도 단서를 잡습니다. 빠른 작업이면 1로, 느리고 정적이면 3~4로 조정 가능.',
    },
    en: {
      title: 'Observation Steps',
      easy: "How many recent frames the policy looks at per inference. Diffusion's default is 2 (previous + current) — good for capturing motion cues.",
      expert: 'Number of recent observation frames Diffusion Policy consumes per inference. The starting index for action generation is n_obs_steps-1, also used in global_cond computation.\n\nPaper default 2 — captures velocity cues from prev + current. Drop to 1 for reactive tasks, raise to 3-4 for slow/static.',
    },
  },

  'Diffusion.horizon': {
    ko: {
      title: '예측 호라이즌 (horizon)',
      easy: '한 번의 디퓨전 과정에서 미래 동작을 몇 개나 만들어낼지 정합니다. 보통 1~2초 분량 — 제어 속도에 맞춰서 정하세요.',
      expert: '디퓨전 sampling이 한 번에 생성하는 액션 시퀀스의 전체 길이. sample shape이 (batch, horizon, action_dim)입니다.\n\nU-Net 다운샘플링 인자(down_dims 길이)로 나누어떨어져야 한다는 제약이 있어요. 1~2초가 일반적(10Hz면 16, 30Hz면 32). 너무 크면 학습이 어렵고 짧으면 장기 일관성↓.',
    },
    en: {
      title: 'Horizon',
      easy: 'How many future actions the model generates per diffusion pass. About 1-2 seconds is typical — pick to match your control rate.',
      expert: 'Full length of the action sequence generated per diffusion sampling pass. Sample shape is (batch, horizon, action_dim).\n\nMust be divisible by the U-Net downsampling factor (length of down_dims). Roughly 1-2 seconds (horizon 16 at 10Hz, 32 at 30Hz). Too long → hard to learn; too short → poor long-horizon coherence.',
    },
  },

  'Diffusion.n_action_steps': {
    ko: {
      title: '실행 액션 수 (n_action_steps)',
      easy: '한 번에 만든 호라이즌 동작 중 실제 로봇에게 보낼 개수입니다. 보통 horizon의 절반.',
      expert: '예측된 horizon 액션 중 실제로 실행할 슬라이스 길이. `action[start : start + n_action_steps]`로 추출됩니다.\n\n기본 8(horizon=16의 절반). 늘리면 추론 호출 빈도↓·속도↑이지만 환경 변화에 늦게 반응합니다.',
    },
    en: {
      title: 'Action Steps',
      easy: 'How many of the planned horizon actions actually get sent to the robot. Usually half of horizon.',
      expert: 'How many of the predicted horizon actions are executed, sliced as `action[start : start + n_action_steps]`.\n\nDefault 8 (half of horizon=16). Increasing it reduces inference frequency but slows reaction to scene changes.',
    },
  },

  'Diffusion.drop_n_last_frames': {
    ko: {
      title: '마지막 N 프레임 제거 (drop_n_last_frames)',
      easy: '에피소드 끝부분 N개 프레임을 학습에서 빼는 옵션입니다.',
      expert: '에피소드 끝에서 drop_n_last_frames 프레임을 학습에서 제외 — 보통 horizon - n_action_steps - 1. 에피소드 종료 직전의 무의미한 정지 액션이 노이즈가 되는 걸 막아줍니다.',
      note: '추후 구현: 현재 코드에는 config로만 정의돼 있고 실제 학습 루프에서 사용되지 않습니다 (원본 Diffusion Policy 참고용). 값을 바꿔도 효과가 없습니다.',
    },
    en: {
      title: 'Drop Last N Frames',
      easy: "Excludes the last N frames of each episode from training.",
      expert: 'Excludes drop_n_last_frames frames at the end of each episode — typically horizon - n_action_steps - 1. Avoids learning from degenerate end-of-episode actions.',
      note: 'Not yet wired up: defined in config but unused by the training loop (carried for reference parity with the original Diffusion Policy). Changing this has no effect.',
    },
  },

  'Diffusion.vision_backbone': {
    ko: {
      title: '비전 백본',
      easy: '카메라 이미지를 숫자 표현으로 바꾸는 CNN. 작고 빠른 resnet18 추천.',
      expert: '이미지를 임베딩으로 변환하는 CNN. `getattr(torchvision.models, name)`으로 동적 로드, "resnet"으로 시작하는 모델만 허용.\n\nresnet18이 표준 추천. 데이터셋이 200 에피소드 이상이면 resnet34/50 시도해볼 만합니다.',
    },
    en: {
      title: 'Vision Backbone',
      easy: 'CNN that converts camera frames into numeric features. resnet18 is the recommended small + fast choice.',
      expert: 'CNN that produces image embeddings. Loaded dynamically via `getattr(torchvision.models, name)`; only names starting with "resnet" are allowed.\n\nresnet18 is the recommended baseline. resnet34/50 worth trying for > 200 episodes.',
    },
  },

  'Diffusion.crop_shape': {
    ko: {
      title: '이미지 크롭 크기 (H, W)',
      easy: '카메라 이미지를 잘라낼 크기 (높이, 너비). 학습 중에는 랜덤 위치로 잘라 일반화를 돕고, 실제 사용할 때는 중앙을 잘라냅니다.',
      expert: 'Vision backbone 입력 전 적용되는 크롭 크기. 학습 시 RandomCrop(crop_shape), 평가/추론 시 CenterCrop으로 자동 전환됩니다 (DiffusionRgbEncoder.forward).\n\n원본 해상도보다 작아야 하고, 작업 중요 영역이 잘리지 않는지 데이터셋 미리보기로 확인하세요.',
    },
    en: {
      title: 'Crop Shape (H, W)',
      easy: 'Crop size for camera frames. Training uses random crops (regularization); inference uses a center crop.',
      expert: 'Crop applied before the vision backbone. Automatically switches between RandomCrop(crop_shape) during training and CenterCrop during eval/inference (in DiffusionRgbEncoder.forward).\n\nMust be smaller than source resolution; verify the crop does not chop off task-relevant regions via the dataset preview.',
    },
  },

  'Diffusion.use_group_norm': {
    ko: {
      title: 'Group Norm 사용',
      easy: 'ResNet 내부 정규화 방식을 작은 배치에서도 안정적인 GroupNorm으로 바꾸는 옵션. 로봇 학습엔 일반적으로 켜두는 게 좋아요.',
      expert: 'ResNet의 BatchNorm을 GroupNorm으로 치환합니다. 작은 배치(≤ 8)에서도 안정적 — 로봇 학습 환경에 적합. 기본 true.',
      note: '⚠️ 사전학습 가중치(pretrained_backbone_weights)와 함께 쓰면 에러가 납니다 — GroupNorm 치환이 사전학습 BatchNorm 통계를 파괴하기 때문. 둘 중 하나만 켜세요.',
    },
    en: {
      title: 'Use Group Norm',
      easy: "Replaces BatchNorm with GroupNorm, which stays stable at the small batch sizes typical in robotics. Usually leave on.",
      expert: "Swaps the ResNet BatchNorm for GroupNorm — far more stable at small batch sizes (≤ 8) common in robotics. Default on.",
      note: '⚠️ Cannot be combined with pretrained_backbone_weights — the swap destroys the pretrained BatchNorm stats and the code raises an error. Use one or the other.',
    },
  },

  'Diffusion.spatial_softmax_num_keypoints': {
    ko: {
      title: 'Spatial Softmax 키포인트 수',
      easy: '카메라 영상에서 "주목할 N개의 위치 단서"를 뽑아내는 개수. 32가 표준이고 더 늘리면 과적합 위험이 있어요.',
      expert: '비전 feature map에서 추출하는 미분가능한 2D 키포인트 수 — (B, num_kp × 2) feature로 액션 예측 conditioning에 들어갑니다. 32가 표준.\n\n많이 늘리면 더 세밀한 위치 단서를 얻지만 작은 데이터셋에서 과적합 위험이 커집니다.',
    },
    en: {
      title: 'Spatial Softmax Keypoints',
      easy: 'Number of "where to look" cues extracted from the feature map. 32 is standard; raising it risks overfitting.',
      expert: 'Number of differentiable 2D keypoints pulled from the backbone feature map, producing a (B, num_kp × 2) feature that conditions action prediction. 32 is standard.\n\nMore keypoints give finer spatial cues but overfit small datasets more easily.',
    },
  },

  'Diffusion.use_separate_rgb_encoder_per_camera': {
    ko: {
      title: '카메라별 별도 RGB 인코더',
      easy: '카메라마다 따로따로 영상 처리기를 만들지(true), 모든 카메라에 같은 처리기를 공유할지(false). 보통 공유(false)가 무난해요.',
      expert: 'True면 카메라마다 독립 ResNet 인스턴스, False면 한 encoder를 공유. global_cond_dim 계산에도 영향을 줍니다.\n\n카메라 시점/렌즈가 매우 다르면 true가 유리하지만, 파라미터·학습 시간이 카메라 수에 비례해 증가. 기본 false 권장.',
    },
    en: {
      title: 'Use Separate RGB Encoder per Camera',
      easy: 'Whether each camera gets its own vision network (true) or all cameras share one (false). Sharing (false) is the safe default.',
      expert: 'True instantiates one ResNet per camera; False shares a single encoder. Also affects global_cond_dim.\n\nSeparate encoders help when cameras differ widely in viewpoint/lens, at the cost of parameter count and training time scaling with the number of cameras. Default false.',
    },
  },

  'Diffusion.down_dims': {
    ko: {
      title: 'U-Net Down Dims',
      easy: '내부 U-Net의 각 단계별 채널 수를 정합니다. GPU 메모리가 작으면 작은 숫자로, 데이터가 매우 크면 큰 숫자로.',
      expert: '1D U-Net의 각 다운샘플링 stage 채널 폭. in_out zip 형태로 사용되며, 배열 길이가 곧 downsampling factor를 결정합니다 (horizon이 이 인자의 배수여야 함).\n\n기본 [256, 512, 1024]. 8GB GPU면 [128, 256, 512], 대형 데이터셋이면 [512, 1024, 2048].',
    },
    en: {
      title: 'Down Dims',
      easy: 'Channel widths of each U-Net stage. Lower for small GPUs, higher for very large datasets.',
      expert: 'Channel widths of each downsampling stage of the 1D U-Net, consumed as in_out zip pairs. The length controls the downsampling factor — horizon must be divisible by it.\n\nDefault [256, 512, 1024]. For an 8GB GPU try [128, 256, 512]; for very large datasets [512, 1024, 2048].',
    },
  },

  'Diffusion.kernel_size': {
    ko: {
      title: 'U-Net 커널 크기',
      easy: 'U-Net이 시간 축에서 한 번에 보는 폭. 5가 표준입니다.',
      expert: '1D convolution의 kernel size. 모든 conv 블록에 동일하게 적용. 5가 표준이며, 늘리면 더 긴 시간 패턴을 잡지만 연산이 증가합니다.',
    },
    en: {
      title: 'Kernel Size',
      easy: "How wide a window the U-Net looks at over time. 5 is standard.",
      expert: 'Kernel size of the 1D convolutions, applied uniformly across all conv blocks. 5 is standard. Larger kernels capture longer temporal patterns at the cost of compute.',
    },
  },

  'Diffusion.n_groups': {
    ko: {
      title: 'Norm Group 수',
      easy: '내부 정규화 그룹 개수. 8이 기본.',
      expert: 'U-Net 내부 GroupNorm의 그룹 수. 채널 수가 이 값으로 나누어떨어져야 합니다. 기본 8 권장.',
    },
    en: {
      title: 'Number of Groups for Norm',
      easy: 'Number of groups for internal normalization. Default 8.',
      expert: 'Group count for GroupNorm layers inside the U-Net. Channel counts must be divisible by this. Default 8 is recommended.',
    },
  },

  'Diffusion.diffusion_step_embed_dim': {
    ko: {
      title: '디퓨전 스텝 임베딩 차원',
      easy: '"지금 디노이즈 몇 단계째인지"를 모델에 알려주는 벡터의 크기. 128이 표준.',
      expert: '디노이즈 timestep t를 임베딩으로 바꿔 U-Net을 conditioning할 때 쓰는 차원. global feature의 일부로 합쳐집니다. 128 권장.',
    },
    en: {
      title: 'Diffusion Step Embedding Dimension',
      easy: 'Size of the vector that tells the model "which denoising step we are on". 128 is standard.',
      expert: 'Embedding dim for the diffusion timestep t, which conditions the U-Net (merged into the global feature). Default 128.',
    },
  },

  'Diffusion.use_film_scale_modulation': {
    ko: {
      title: 'FiLM Scale 변조',
      easy: '컨디셔닝을 더 풍부하게 주입하는 기법. 보통 켜두는 게 좋아요.',
      expert: 'FiLM conditioning이 shift만(false) 줄지, scale + shift(true) 둘 다 줄지 결정. true면 cond_channels = out_channels × 2.\n\nScale + shift가 표현력↑ — 일반적으로 true 권장.',
    },
    en: {
      title: 'Use FiLM Scale Modulation',
      easy: 'Slightly richer conditioning. Usually on.',
      expert: 'Whether FiLM conditioning applies both scale and shift (true) or only shift (false). When true, cond_channels = out_channels × 2.\n\nScale + shift is more expressive — generally recommended on.',
    },
  },

  'Diffusion.noise_scheduler_type': {
    ko: {
      title: '노이즈 스케줄러',
      easy: '디퓨전 노이즈를 어떻게 더하고 빼낼지 정하는 알고리즘. DDPM이 가장 안정적이고 표준.',
      expert: 'DDPMScheduler 또는 DDIMScheduler 선택. _make_noise_scheduler()에서 분기.\n\n- DDPM: 학습 안정성 좋고 표준 (보통 100 step)\n- DDIM: 결정론적 sampling, 추론 step을 5~10까지 줄일 수 있음',
      note: '학습은 DDPM, 추론만 DDIM으로 가속하는 패턴이 일반적이지만, EasyTrainer는 학습 시 선택한 스케줄러를 추론에도 그대로 사용합니다.',
    },
    en: {
      title: 'Noise Scheduler Type',
      easy: 'Algorithm for adding/removing diffusion noise. DDPM is the safest, standard choice.',
      expert: 'Picks DDPMScheduler or DDIMScheduler via _make_noise_scheduler().\n\n- DDPM: more stable training, slower inference (~100 steps)\n- DDIM: deterministic sampling, inference can drop to 5-10 steps',
      note: 'A common pattern is to train DDPM + infer DDIM, but EasyTrainer uses the same scheduler at training and inference.',
    },
  },

  'Diffusion.num_train_timesteps': {
    ko: {
      title: '학습 타임스텝 수',
      easy: '학습할 때 노이즈를 단계적으로 더하는 횟수. 100이 표준.',
      expert: '학습 중 디퓨전 timestep 총 개수. 늘리면 더 부드러운 노이즈 스케줄이지만 추론/학습이 비례해서 느려집니다. 기본 100.',
    },
    en: {
      title: 'Number of Train Timesteps',
      easy: 'How many noise steps the model is trained over. 100 is standard.',
      expert: 'Total diffusion timesteps during training. More timesteps = smoother noise schedule but proportionally slower training/inference. Default 100.',
    },
  },

  'Diffusion.beta_schedule': {
    ko: {
      title: 'Beta 스케줄',
      easy: '노이즈 강도가 시간에 따라 변하는 방식. squaredcos_cap_v2가 로봇 학습에 더 안정적이에요.',
      expert: '노이즈 강도 β가 timestep에 따라 어떻게 변하는지.\n\n- linear: 단순 선형 증가\n- squaredcos_cap_v2: cosine 기반, robotics에서 더 흔하고 안정적',
    },
    en: {
      title: 'Beta Schedule',
      easy: 'How noise strength evolves over time. squaredcos_cap_v2 is more stable for robotics.',
      expert: 'How the noise strength β evolves across timesteps.\n\n- linear: simple linear ramp\n- squaredcos_cap_v2: cosine-based, more common and stable in robotics',
    },
  },

  'Diffusion.beta_start': {
    ko: {
      title: 'Beta Start',
      easy: '노이즈 강도의 시작값. 0.0001이 표준이고 거의 바꿀 일이 없어요.',
      expert: '노이즈 스케줄의 초기 β. 0.0001 권장. 거의 변경하지 않습니다.',
    },
    en: {
      title: 'Beta Start',
      easy: 'Initial noise strength. 0.0001 is standard and rarely changed.',
      expert: 'Starting β of the noise schedule. 0.0001 recommended; almost never tuned.',
    },
  },

  'Diffusion.beta_end': {
    ko: {
      title: 'Beta End',
      easy: '노이즈 강도의 끝값. 0.02가 표준이고 거의 바꿀 일이 없어요.',
      expert: '노이즈 스케줄의 최종 β. 0.02 권장. 거의 변경하지 않습니다.',
    },
    en: {
      title: 'Beta End',
      easy: 'Final noise strength. 0.02 is standard and rarely changed.',
      expert: 'Final β of the noise schedule. 0.02 recommended; almost never tuned.',
    },
  },

  'Diffusion.prediction_type': {
    ko: {
      title: '예측 타입',
      easy: '모델이 "노이즈 자체"를 예측할지 "깨끗한 동작"을 예측할지 정합니다. epsilon(노이즈 예측)이 더 안정적.',
      expert: 'compute_loss의 target을 결정합니다.\n\n- epsilon: 노이즈를 예측 (논문 기본, 더 안정)\n- sample: 깨끗한 액션 시퀀스 자체를 예측 (수렴 빠를 수 있음)',
    },
    en: {
      title: 'Prediction Type',
      easy: 'Whether the model predicts the noise itself or the clean action. epsilon (noise) is the more stable default.',
      expert: 'Determines the target in compute_loss.\n\n- epsilon: predict the noise (paper default, more stable)\n- sample: predict the clean action sequence (can converge faster)',
    },
  },

  'Diffusion.clip_sample': {
    ko: {
      title: '샘플 클리핑',
      easy: '디노이즈 중간값이 너무 커지지 않도록 잘라주는 안전장치. 켜두는 게 안전해요.',
      expert: '디노이즈 중간 결과를 [-clip_sample_range, +clip_sample_range]로 잘라낼지 결정. 액션이 [-1, 1]로 정규화돼 있을 때 발산 방지. 기본 on.',
    },
    en: {
      title: 'Clip Sample',
      easy: 'Safety clamp on intermediate denoising values. Leave on.',
      expert: 'Whether intermediate denoising outputs are clipped to [-clip_sample_range, +clip_sample_range]. Keeps things in range when actions are normalized to [-1, 1]. Default on.',
    },
  },

  'Diffusion.clip_sample_range': {
    ko: {
      title: '샘플 클리핑 범위',
      easy: '클리핑 켰을 때 얼마까지 허용할지의 절대값. 액션이 [-1, 1] 정규화돼 있다면 1.0 그대로.',
      expert: 'clip_sample=true일 때 적용되는 절대값 한도. 액션이 [-1, 1]에 정규화된 표준 가정에서 1.0이 표준.',
    },
    en: {
      title: 'Clip Sample Range',
      easy: 'The absolute bound when clipping is on. 1.0 matches the standard [-1, 1] action normalization.',
      expert: 'Absolute clipping bound used when clip_sample is on. 1.0 is standard, assuming actions are normalized to [-1, 1].',
    },
  },

  'Diffusion.do_mask_loss_for_padding': {
    ko: {
      title: '패딩 손실 마스킹',
      easy: '에피소드 길이가 짧을 때 빈자리(패딩)를 학습에서 제외할지 결정합니다. 에피소드 길이 편차가 클 때 켜면 좋아요.',
      expert: '시퀀스가 horizon보다 짧을 때 채워넣은 padding 위치를 loss에서 제외할지(true) 포함할지(false). compute_loss에서 action_is_pad 마스크로 적용. 에피소드 길이 편차가 크면 true 권장.',
    },
    en: {
      title: 'Mask Loss for Padding',
      easy: 'When episodes are shorter than horizon, whether to exclude the padding from the loss. Turn on if episode lengths vary a lot.',
      expert: 'Whether to exclude padded positions from the loss (true) or include them (false) when sequences are shorter than horizon. Applied via the action_is_pad mask in compute_loss. Recommended on for highly varying episode lengths.',
    },
  },

  // ============================================================
  // PI05 — model config
  // ============================================================
  'PI05.hf_token': {
    ko: {
      title: 'HuggingFace 토큰',
      easy: 'PaliGemma 모델을 인터넷에서 다운로드할 때 필요한 비밀번호입니다. https://huggingface.co/settings/tokens 에서 발급해 붙여넣으세요.',
      expert: 'PaliGemma/Gemma는 HuggingFace에서 사용 동의(gated)가 필요한 모델로, 학습 서버가 처음 다운로드할 때 read 권한 토큰이 필요합니다.\n\ntrain_worker.py가 HF_TOKEN / HUGGING_FACE_HUB_TOKEN 환경변수로 설정해서 transformers의 다운로드 흐름에 전달합니다. 이미 캐시돼 있다면 비워둬도 됩니다.',
    },
    en: {
      title: 'HuggingFace Token',
      easy: 'Password used to download the PaliGemma model from HuggingFace. Create a read-token at https://huggingface.co/settings/tokens.',
      expert: 'PaliGemma/Gemma are gated on HuggingFace and require accepting a license. train_worker.py exports it as HF_TOKEN / HUGGING_FACE_HUB_TOKEN so transformers can authenticate downloads.\n\nLeave empty if the server already has the weights cached.',
    },
  },

  'PI05.n_obs_steps': {
    ko: {
      title: '관찰 스텝 수',
      easy: 'PI0.5는 한 장의 영상만 보고 동작을 결정합니다. 1 그대로 두세요.',
      expert: 'PI0.5는 단일 frame 관찰 전제. processor가 1프레임만 처리합니다. 기본 1, 바꿀 일 없습니다.',
    },
    en: {
      title: 'Observation Steps',
      easy: 'PI0.5 uses just one frame to decide. Keep at 1.',
      expert: 'PI0.5 assumes single-frame observations; the processor only handles one frame. Default 1, rarely changed.',
    },
  },

  'PI05.chunk_size': {
    ko: {
      title: '청크 크기 (chunk_size)',
      easy: 'PI0.5가 한 번에 미리 만들어두는 동작 개수. 기본 50으로 그대로 두는 게 일반적이에요.',
      expert: 'PI0.5가 1회 forward에 생성하는 액션 시퀀스 길이. PI0.5는 긴 chunk에서 강하도록 설계 — 기본 50.\n\n추론 Hz가 매우 높은 경우에만 줄여보세요.',
    },
    en: {
      title: 'Chunk Size',
      easy: 'How many actions PI0.5 plans in one shot. Usually leave at 50.',
      expert: "Length of the action sequence PI0.5 generates per forward pass. PI0.5 is designed for long chunks — default 50.\n\nOnly reduce if inference frequency is unusually high.",
    },
  },

  'PI05.n_action_steps': {
    ko: {
      title: '실행 액션 수',
      easy: 'PI0.5는 미리 만든 동작 전체(50개)를 다 쓰고 그다음에 다시 생각하는 방식이에요.',
      expert: 'chunk에서 실제 실행할 액션 수. 기본은 chunk_size와 동일(50)로 chunk 전체를 실행 후 재추론. chunk_size 이하만 허용.',
    },
    en: {
      title: 'Action Steps',
      easy: 'PI0.5 plays the entire planned chunk (50 actions) before re-planning.',
      expert: 'How many actions from the chunk get executed. Default equals chunk_size (50) — execute the full chunk, then re-infer. Must be ≤ chunk_size.',
    },
  },

  'PI05.max_state_dim': {
    ko: {
      title: '최대 상태 차원',
      easy: 'PI0.5는 다양한 로봇을 지원하려고 상태를 32개 길이로 통일합니다. 그냥 기본값 두세요.',
      expert: 'PI0.5는 morphology-agnostic하게 학습되도록 state를 고정 길이(32) 벡터로 zero-pad합니다. configuration.py가 state feature shape으로 강제. 거의 변경할 일 없습니다.',
    },
    en: {
      title: 'Max State Dimension',
      easy: 'PI0.5 pads state to a fixed length (32) so it can support different robots. Leave as-is.',
      expert: 'PI0.5 zero-pads state to a fixed-length vector (32) for morphology-agnostic training. Enforced as the state feature shape in configuration.py. Rarely changed.',
    },
  },

  'PI05.max_action_dim': {
    ko: {
      title: '최대 액션 차원',
      easy: '액션도 32개 길이로 통일됩니다. 단일 팔+그리퍼 정도면 그대로 두세요.',
      expert: '액션을 길이 32로 zero-pad — state와 같은 morphology-agnostic 이유. modeling.py의 pad_vector()가 실제 패딩을 수행. 일반적 단일 팔 + 그리퍼면 기본값 그대로.',
    },
    en: {
      title: 'Max Action Dimension',
      easy: 'Actions are also padded to length 32. Leave as-is for a typical single-arm + gripper.',
      expert: 'Actions are zero-padded to length 32 — same morphology-agnostic reason as state. Actual padding done by pad_vector() in modeling.py. Leave default for typical single-arm setups.',
    },
  },

  'PI05.image_resolution': {
    ko: {
      title: '이미지 해상도 (H, W)',
      easy: 'PaliGemma가 받아들이는 영상 크기. (224, 224)가 표준이고 바꾸면 학습된 가중치가 망가질 수 있어요.',
      expert: 'PaliGemma 비전 입력 크기. modeling.py에서 resize_with_pad_torch()로 적용됩니다. (224, 224)가 표준이며 변경 시 PaliGemma 사전학습 가중치 효력 상실.',
    },
    en: {
      title: 'Image Resolution',
      easy: 'Image size that goes into PaliGemma. (224, 224) is standard — changing it can invalidate the pretrained weights.',
      expert: 'PaliGemma vision input size, applied via resize_with_pad_torch() in modeling.py. (224, 224) is standard; changing it invalidates the pretrained weights.',
    },
  },

  'PI05.num_inference_steps': {
    ko: {
      title: '추론 스텝 수 (flow matching)',
      easy: '추론할 때 동작을 만들어내는 반복 횟수. 많으면 정확하고 적으면 빠릅니다. 기본 10이 좋은 균형.',
      expert: 'PI0.5는 flow matching으로 액션을 생성 — 추론 시 ODE 적분 step 수. 늘리면 정확도↑·속도↓.\n\n10이 좋은 trade-off. 가속이 필요하면 5까지 줄일 수 있습니다.',
    },
    en: {
      title: 'Inference Steps (Flow Matching)',
      easy: 'How many integration steps PI0.5 takes to generate each chunk. More = accurate; fewer = fast. Default 10 is a good balance.',
      expert: 'PI0.5 generates actions via flow matching — this is the ODE integration step count at inference. More = more accurate but slower.\n\n10 is a good trade-off. Drop to 5 for extra speed.',
    },
  },

  'PI05.tokenizer_max_length': {
    ko: {
      title: '토크나이저 최대 길이',
      easy: '자연어 명령("빨간 컵을 집어")이 잘릴 수 있는 최대 토큰 수. 짧은 명령엔 200이면 충분.',
      expert: 'PaliGemma tokenizer max_length 인자로 전달됩니다 (processor.py). 짧은 manipulation instruction에는 200이면 충분.',
    },
    en: {
      title: 'Tokenizer Max Length',
      easy: 'Max token length for natural-language instructions like "pick up the red cube". 200 is plenty for short commands.',
      expert: 'Passed to the PaliGemma tokenizer as max_length (in processor.py). 200 is enough for short manipulation instructions.',
    },
  },

  'PI05.paligemma_variant': {
    ko: {
      title: 'PaliGemma 변종',
      easy: '큰 vision-language 모델의 크기 선택. 현재 gemma_2b 한 가지만 지원돼요.',
      expert: 'Vision-language backbone 크기. __post_init__에서 허용 옵션(gemma_2b 등) 검증. 현재 코드는 gemma_2b만 지원.',
    },
    en: {
      title: 'PaliGemma Variant',
      easy: "Size of the big vision-language model. Only gemma_2b is currently supported.",
      expert: 'Vision-language backbone size. Allowed values validated in __post_init__. The codebase currently supports only gemma_2b.',
    },
  },

  'PI05.action_expert_variant': {
    ko: {
      title: '액션 전문가 변종',
      easy: '동작을 만들어내는 작은 전문가 모델의 크기. 현재 gemma_300m 한 가지만 지원돼요.',
      expert: '액션 생성 전용 소형 transformer 크기. 현재 코드는 gemma_300m만 지원합니다.',
    },
    en: {
      title: 'Action Expert Variant',
      easy: 'Size of the small "action specialist" model. Only gemma_300m is supported.',
      expert: 'Size of the small transformer dedicated to action generation. Currently only gemma_300m is supported.',
    },
  },

  'PI05.freeze_vision_encoder': {
    ko: {
      title: '비전 인코더 동결',
      easy: 'PaliGemma의 영상 처리 부분을 학습 동안 안 건드리고 그대로 둘지 결정합니다. 보통 켜두는 게 안전해요.',
      expert: '비전(PaliGemma) 부분을 학습에서 제외 — 작은 데이터셋에서 PaliGemma 사전학습 표현을 보존. true 권장.\n\nfalse로 풀려면 24GB 이상 GPU + 충분한 데이터(논문 기준 400h)가 필요합니다.',
      note: '추후 구현: config에는 정의돼 있지만 실제 freeze 로직이 train_worker.py에서 활성화돼 있지 않습니다 — 현재로서는 어느 값을 골라도 같은 동작이 나올 수 있습니다.',
    },
    en: {
      title: 'Freeze Vision Encoder',
      easy: "Whether the vision part of PaliGemma stays untouched during training. Usually safest to leave on.",
      expert: 'Excludes the vision (PaliGemma) part from training — preserves the pretrained representation on small datasets. Recommended on.\n\nUnfreezing requires ≥ 24GB GPU and substantial data (~400h in the paper).',
      note: 'Not yet wired up: the flag is defined in config but the actual freeze logic in train_worker.py is commented out — both values may currently behave the same.',
    },
  },

  'PI05.train_expert_only': {
    ko: {
      title: '액션 전문가만 학습',
      easy: '큰 PaliGemma는 그대로 두고, 작은 액션 전문가 부분만 학습합니다. 단일 GPU + LoRA로 학습할 때 켜두세요.',
      expert: '액션 expert만 학습하고 PaliGemma LLM 전체는 frozen 상태. 단일 GPU + LoRA 파인튜닝 환경에 권장.',
      note: '추후 구현: config에는 정의돼 있지만 실제 freeze 로직이 활성화돼 있지 않습니다. LoRA 사용 시에는 LoRA 자체가 frozen base + adapter만 학습하므로 결과적으론 의도된 동작에 가까워집니다.',
    },
    en: {
      title: 'Train Expert Only',
      easy: 'Keeps the big PaliGemma frozen and only trains the small action specialist. Turn on for single-GPU + LoRA fine-tuning.',
      expert: 'Trains only the action expert while keeping the PaliGemma LLM frozen. Recommended for single-GPU + LoRA fine-tuning.',
      note: 'Not yet wired up: defined in config but the freeze logic in the training code is currently commented out. With LoRA enabled, the LoRA mechanism itself effectively achieves the intended behavior (frozen base + adapter training).',
    },
  },

  'PI05.gradient_checkpointing': {
    ko: {
      title: 'Gradient Checkpointing',
      easy: 'GPU 메모리를 아끼는 대신 학습이 약 20% 느려지는 옵션. 24GB GPU에서 PI0.5 학습할 때 보통 켜둡니다.',
      expert: '중간 activation을 저장하지 않고 backward 시 재계산해 VRAM을 절약. 속도 ~20% 손해, 24GB GPU에서 PI0.5 학습을 가능하게 합니다.',
    },
    en: {
      title: 'Gradient Checkpointing',
      easy: "Saves GPU memory at the cost of ~20% slower training. Usually on when training PI0.5 on a 24GB GPU.",
      expert: 'Recomputes intermediate activations in the backward pass instead of caching them — saves VRAM at ~20% speed cost. Makes PI0.5 trainable on 24GB GPUs.',
    },
  },

  'PI05.use_relative_actions': {
    ko: {
      title: '상대 액션 사용',
      easy: '"어디로 가" 대신 "지금 위치에서 얼마나 움직여"로 동작을 표현합니다. 데이터가 적을 때 학습이 잘 돼서 보통 켜둬요.',
      expert: '액션을 absolute target 대신 현재 state 대비 delta로 표현. processor.py의 RelativeActionsProcessorStep이 변환을 담당.\n\nDelta는 0 근처에 집중돼 작은 데이터셋에서 학습이 쉬워집니다. False면 절대 좌표를 그대로 사용 — 대형 사전학습 모델이나 정밀 추적 작업에 적합.',
    },
    en: {
      title: 'Use Relative Actions',
      easy: 'Expresses actions as "how far to move from here" instead of "go to this exact spot". Helps learning on small datasets.',
      expert: 'Express actions as deltas from the current state rather than absolute targets. Conversion handled by RelativeActionsProcessorStep in processor.py.\n\nDeltas concentrate around zero and are easier to learn on small datasets. False uses absolute coordinates — fits large-pretrain models or precise tracking.',
    },
  },

  'PI05.absolute_action_dims': {
    ko: {
      title: '절대 액션 차원 인덱스',
      easy: '상대 모드라도 "그리퍼 열림/닫힘" 같은 일부 차원은 절대값으로 두는 게 자연스러워요. 그 차원 번호들을 적어주세요. 기본 [6, 7].',
      expert: 'use_relative_actions=true일 때, 일부 차원만 절대값으로 유지할 인덱스. 나머지는 delta로 자동 변환.\n\n[6, 7]: 6-DOF 단일 팔의 그리퍼(6)와 done flag(7) 유지\n듀얼팔 (2×6 joints + 2 gripper + done): [6, 13, 14]\n빈 배열 []: 모든 차원을 delta로 처리',
    },
    en: {
      title: 'Absolute Action Dims',
      easy: 'Even in relative mode, some dims (like gripper open/close) make more sense as absolute. List those dim indices. Default [6, 7].',
      expert: 'When use_relative_actions=true, the dim indices that stay absolute. All other dims become deltas.\n\n[6, 7]: 6-DOF single arm — gripper (6) and done flag (7) absolute\nDual-arm (2×6 joints + 2 grippers + done): [6, 13, 14]\nEmpty []: all dims as delta',
    },
  },

  // ============================================================
  // Shared model config (assigned below for all three policies)
  // ============================================================
  'ACT.action_type': null,
  'Diffusion.action_type': null,
  'PI05.action_type': null,
  'ACT.obs_state_keys': null,
  'Diffusion.obs_state_keys': null,
  'PI05.obs_state_keys': null,

  // ============================================================
  // ACT — training config
  // ============================================================
  'ACT.dropout': {
    ko: {
      title: 'Dropout 비율',
      easy: '학습 중에 일부 연결을 무작위로 잠시 끊어서 과적합을 줄여줍니다. 0.1이 표준.',
      expert: 'Transformer attention/FFN 내부에 적용되는 dropout 확률. encoder/decoder 모든 layer에 동일 적용.\n\n작은 데이터셋(<50 episode)에서 과적합 보이면 0.2~0.3까지 시도해보세요.',
    },
    en: {
      title: 'Dropout Rate',
      easy: 'Randomly drops some connections during training to reduce overfitting. 0.1 is standard.',
      expert: 'Dropout probability inside the transformer attention/FFN, applied uniformly to all encoder/decoder layers.\n\nRaise to 0.2-0.3 if you see overfitting on small datasets (< 50 episodes).',
    },
  },

  'ACT.kl_weight': {
    ko: {
      title: 'KL Divergence 가중치',
      easy: 'VAE가 "다양한 동작"을 학습하도록 강제하는 정도. 시연이 매우 일관적이면 줄여서 정확도를 우선시할 수 있어요.',
      expert: 'CVAE KL term이 전체 loss에 기여하는 비중. loss = l1_loss + mean_kld × kl_weight.\n\nACT 기본 10. use_vae=true일 때만 효과 있음. 시연 일관성이 매우 높으면 1~5로 줄여 reconstruction 정확도 우선.',
    },
    en: {
      title: 'KL Divergence Weight',
      easy: 'How strongly the VAE is pushed to learn varied actions. Lower it for very consistent demos to prioritize accuracy.',
      expert: "How much the CVAE KL term contributes to total loss: loss = l1_loss + mean_kld × kl_weight.\n\nACT default 10. Only active when use_vae=true. Lower to 1-5 for very consistent demos to favor reconstruction accuracy.",
    },
  },

  'ACT.optimizer_lr': {
    ko: {
      title: '학습률 (learning rate)',
      easy: 'Transformer 부분을 한 걸음에 얼마나 크게 업데이트할지. 너무 크면 발산, 너무 작으면 학습이 느려요. ACT 기본 1e-5는 작은 편이라 안정적이에요.',
      expert: 'Transformer 부분의 AdamW learning rate. ACT 기본 1e-5는 작은 편으로 적은 데이터로도 안정적.\n\n학습 loss가 plateau면 3e-5 정도까지 올려보세요.',
    },
    en: {
      title: 'Optimizer Learning Rate',
      easy: "How big each training step is for the transformer. Too big diverges; too small is slow. ACT's small default (1e-5) is stable.",
      expert: 'AdamW learning rate for the transformer. ACT default 1e-5 is conservative — stable even with little data.\n\nIf train loss plateaus, try up to ~3e-5.',
    },
  },

  'ACT.optimizer_weight_decay': {
    ko: {
      title: 'Weight Decay',
      easy: '가중치가 너무 커지지 않도록 잡아주는 정규화 강도. 1e-4가 ACT 기본.',
      expert: 'AdamW L2 정규화 강도. 1e-4가 ACT 기본. 심한 과적합엔 1e-3까지 늘릴 수 있지만 너무 크면 학습 자체를 방해.',
    },
    en: {
      title: 'Optimizer Weight Decay',
      easy: 'Regularization that prevents weights from growing too large. 1e-4 is the ACT default.',
      expert: 'AdamW L2 regularization strength. ACT default 1e-4. Heavy overfitting → up to 1e-3, but too much will block learning entirely.',
    },
  },

  'ACT.optimizer_lr_backbone': {
    ko: {
      title: '백본 학습률',
      easy: '비전 신경망(ResNet/Dino)에만 따로 적용되는 학습률. 사전학습 가중치를 보존하려면 더 작게 두세요.',
      expert: '비전 백본 파라미터 그룹에만 적용되는 별도 lr. get_optim_params()에서 backbone params를 분리해서 이 lr을 할당.\n\n사전학습 weights 보존을 원하면 Transformer lr과 같거나 더 작게(1e-6) 두세요.',
    },
    en: {
      title: 'Backbone Learning Rate',
      easy: "Separate learning rate just for the vision network (ResNet/Dino). Keep it smaller to preserve pretrained weights.",
      expert: 'Separate lr applied only to vision backbone parameters. get_optim_params() splits backbone params and assigns this lr.\n\nKeep it equal to or smaller (e.g. 1e-6) than the transformer lr to preserve pretrained weights.',
    },
  },

  // ============================================================
  // Diffusion — training config
  // ============================================================
  'Diffusion.optimizer_lr': {
    ko: {
      title: '학습률',
      easy: 'Diffusion Policy의 학습 보폭. 기본 1e-4는 ACT보다 큼 — U-Net이 더 안정적이에요.',
      expert: 'AdamW learning rate. Diffusion Policy 기본 1e-4 — U-Net이 transformer보다 안정적이라 ACT 기본보다 큽니다.\n\n발산하면 5e-5로 줄이세요.',
    },
    en: {
      title: 'Optimizer Learning Rate',
      easy: "Diffusion's step size. Default 1e-4 is larger than ACT's because the U-Net is more stable.",
      expert: 'AdamW learning rate. Diffusion Policy default 1e-4 — larger than ACT because the U-Net is more stable than a transformer.\n\nDrop to 5e-5 if training diverges.',
    },
  },

  'Diffusion.optimizer_betas': {
    ko: {
      title: 'AdamW Betas',
      easy: 'AdamW 옵티마이저의 "관성" 계수. 거의 바꿀 일 없어요.',
      expert: 'AdamW의 1차/2차 모멘텀 감쇠 계수. Diffusion Policy 논문은 [0.95, 0.999] 사용 — Transformer 기본 [0.9, 0.999]보다 약간 보수적.',
    },
    en: {
      title: 'Optimizer Betas',
      easy: 'AdamW "momentum" coefficients. Rarely changed.',
      expert: 'AdamW first/second moment decay. Diffusion Policy uses [0.95, 0.999] — slightly more conservative than the Transformer default [0.9, 0.999].',
    },
  },

  'Diffusion.optimizer_eps': {
    ko: {
      title: 'AdamW Epsilon',
      easy: '수치 안정용 작은 상수. 1e-8 그대로 두면 됩니다.',
      expert: 'AdamW 수치 안정용 epsilon. 1e-8 표준. 거의 바꿀 일 없습니다.',
    },
    en: {
      title: 'Optimizer Epsilon',
      easy: 'Small constant for numerical stability. Leave at 1e-8.',
      expert: 'AdamW numerical stability epsilon. Standard 1e-8, rarely tuned.',
    },
  },

  'Diffusion.optimizer_weight_decay': {
    ko: {
      title: 'Weight Decay',
      easy: 'Diffusion Policy의 가벼운 정규화. 1e-6이 기본 — 과적합 심하면 살짝 올려보세요.',
      expert: 'AdamW L2 정규화. Diffusion Policy 기본 1e-6 — 약한 정규화. 과적합 심하면 1e-5까지.',
    },
    en: {
      title: 'Optimizer Weight Decay',
      easy: "Light regularization for Diffusion. 1e-6 default; bump up slightly for heavy overfitting.",
      expert: 'AdamW L2 regularization. Diffusion Policy default 1e-6 — light. Heavy overfitting → up to 1e-5.',
    },
  },

  'Diffusion.scheduler_name': {
    ko: {
      title: 'LR 스케줄러',
      easy: '학습이 진행될수록 학습률을 어떻게 줄일지. cosine이 가장 무난해요.',
      expert: '학습률 decay 방식.\n- cosine: 초반 천천히, 후반 빠르게 감소 (권장)\n- linear: 일정 선형 감소\n- constant: 감소 없음',
    },
    en: {
      title: 'LR Scheduler',
      easy: 'How the learning rate decays over training. cosine is the safe default.',
      expert: 'How LR decays during training.\n- cosine: slow at first, then fast (recommended)\n- linear: steady linear decay\n- constant: no decay',
    },
  },

  'Diffusion.scheduler_warmup_steps': {
    ko: {
      title: 'Warmup 스텝',
      easy: '학습 초반에 학습률을 천천히 올려서 시작하는 단계 수. 500이 보통 충분해요.',
      expert: '학습 초반 N step 동안 lr을 0에서 목표값까지 선형 증가. DiffuserSchedulerConfig(num_warmup_steps)로 전달.\n\n500이 일반적으로 충분. 매우 큰 데이터셋 + 작은 batch면 1000~2000.',
    },
    en: {
      title: 'LR Scheduler Warmup Steps',
      easy: "Steps spent ramping the learning rate up from zero at the start. 500 is usually enough.",
      expert: 'Linearly ramps LR from 0 to target over N steps. Passed to DiffuserSchedulerConfig(num_warmup_steps).\n\n500 is typically enough. Use 1000-2000 for very large datasets with small batch sizes.',
    },
  },

  // ============================================================
  // PI05 — training config
  // ============================================================
  'PI05.optimizer_lr': {
    ko: {
      title: '학습률',
      easy: 'PI0.5는 워낙 큰 모델이라 학습률이 작아야 안정적. 2.5e-5가 기본.',
      expert: 'AdamW lr. PI0.5는 큰 vision-language 모델이라 2.5e-5처럼 작은 lr로 안정적 학습. LoRA 파인튜닝에선 5e-5까지 시도 가능.',
    },
    en: {
      title: 'Optimizer Learning Rate',
      easy: "PI0.5 is huge, so it needs a small learning rate. 2.5e-5 default.",
      expert: 'AdamW lr. PI0.5 is a large vision-language model that trains stably at small lrs like 2.5e-5. Up to ~5e-5 for LoRA fine-tuning.',
    },
  },

  'PI05.optimizer_betas': {
    ko: {
      title: 'AdamW Betas',
      easy: 'PI0.5는 LLM 학습에서 흔한 [0.9, 0.95]를 사용합니다.',
      expert: 'PI0.5는 [0.9, 0.95] 사용 — LLM 학습에서 흔한 설정.',
    },
    en: {
      title: 'Optimizer Betas',
      easy: 'PI0.5 uses [0.9, 0.95], common in LLM training.',
      expert: 'PI0.5 uses [0.9, 0.95] — common LLM-style setting.',
    },
  },

  'PI05.optimizer_eps': {
    ko: {
      title: 'AdamW Epsilon',
      easy: '수치 안정용 상수. 1e-8 그대로.',
      expert: 'AdamW 수치 안정 epsilon. 1e-8 표준.',
    },
    en: {
      title: 'Optimizer Epsilon',
      easy: 'Numerical-stability constant. Leave at 1e-8.',
      expert: 'AdamW numerical-stability epsilon. Standard 1e-8.',
    },
  },

  'PI05.optimizer_weight_decay': {
    ko: {
      title: 'Weight Decay',
      easy: 'PI0.5는 사실상 weight decay를 끄고 학습합니다 (1e-10). 절대 크게 올리지 마세요.',
      expert: 'openpi 기본 1e-10 — 사실상 비활성. weight decay가 PaliGemma 사전학습 가중치를 과정규화해 학습 초기에 학습된 feature를 망가뜨릴 수 있어 의도적으로 매우 작게 둡니다.',
      note: '⚠️ 백엔드 transformer AdamW 기본은 0.01이지만 EasyTrainer는 의도적으로 1e-10으로 override합니다. 직접 큰 값으로 바꾸면 학습이 망가질 수 있습니다.',
    },
    en: {
      title: 'Optimizer Weight Decay',
      easy: 'PI0.5 effectively disables weight decay (1e-10). Do not raise this.',
      expert: 'openpi default 1e-10 — effectively off. Weight decay would over-regularize PaliGemma\'s pretrained weights and damage early-training features, so it is intentionally tiny.',
      note: "⚠️ The HF AdamW default is 0.01 but EasyTrainer intentionally overrides it to 1e-10. Raising it can break training.",
    },
  },

  'PI05.optimizer_grad_clip_norm': {
    ko: {
      title: 'Gradient Clip Norm',
      easy: '한 번의 학습 step에서 너무 큰 기울기를 잘라주는 안전장치. 1.0이 표준.',
      expert: '매 step의 gradient L2 norm이 이 값을 넘으면 잘라냅니다 (torch.nn.utils.clip_grad_norm_). 1.0 표준. 학습 안정에 매우 중요.',
    },
    en: {
      title: 'Gradient Clip Norm',
      easy: 'Safety clamp on per-step gradients. 1.0 standard.',
      expert: 'Caps gradient L2 norm per step via torch.nn.utils.clip_grad_norm_. 1.0 standard — important for stability.',
    },
  },

  'PI05.scheduler_warmup_steps': {
    ko: {
      title: 'Warmup 스텝',
      easy: '학습 초반에 학습률을 천천히 올려가는 step 수. PI0.5는 1000이 권장.',
      expert: 'lr을 0에서 목표값까지 선형 증가시키는 step 수. PI0.5는 1000 권장.',
    },
    en: {
      title: 'Warmup Steps',
      easy: 'Steps to ramp LR up at the start. PI0.5 recommends 1000.',
      expert: 'Steps to linearly ramp LR from 0 to target. 1000 recommended for PI0.5.',
    },
  },

  'PI05.scheduler_decay_steps': {
    ko: {
      title: 'Decay 스텝',
      easy: 'Warmup 이후 학습률을 천천히 내리는 데 걸리는 step 수. 30000이 표준.',
      expert: 'Warmup 이후 lr이 scheduler_decay_lr까지 감소하는 step 수. 30000 표준 — 데이터셋 크기에 맞춰 조정.',
    },
    en: {
      title: 'Scheduler Decay Steps',
      easy: 'How many steps to decay LR over after warmup. 30000 standard.',
      expert: 'Number of steps over which LR decays from peak to scheduler_decay_lr after warmup. 30000 standard — adjust to dataset size.',
    },
  },

  'PI05.scheduler_decay_lr': {
    ko: {
      title: 'Decay 후 학습률',
      easy: 'Decay가 끝난 뒤 유지되는 최저 학습률. 보통 처음 학습률의 1/10.',
      expert: 'Decay 완료 후 유지되는 최소 lr. 보통 optimizer_lr의 1/10. PI0.5 기본 2.5e-6.',
    },
    en: {
      title: 'Scheduler Decay LR',
      easy: 'Minimum LR after decay finishes. Usually ~1/10 of the starting LR.',
      expert: 'Minimum LR maintained after decay ends. Usually ~1/10 of optimizer_lr. PI0.5 default 2.5e-6.',
    },
  },

  // ============================================================
  // Common training config — shared across policies
  // ============================================================
  'Common.num_epochs': {
    ko: {
      title: '에폭 수 (num_epochs)',
      easy: '전체 데이터를 몇 번 반복해서 학습할지. 작은 데이터셋이면 많이(500~1500), 큰 데이터셋이면 적게(200 이하) 돌리는 게 보통이에요.',
      expert: '전체 데이터셋 반복 횟수. 작은 데이터셋이면 500~1500, 큰 데이터셋이면 200 이하부터 시작.\n\nValidation loss가 plateau면 일찍 중단해도 됩니다.',
    },
    en: {
      title: 'Number of Epochs',
      easy: 'How many times to loop over the dataset. Small dataset → many epochs (500-1500); large dataset → fewer (≤ 200).',
      expert: 'Number of passes over the full dataset. 500-1500 for small datasets; start at ≤ 200 for large ones.\n\nStopping early once validation plateaus is fine.',
    },
  },

  'Common.batch_size': {
    ko: {
      title: '배치 크기',
      easy: '한 번에 모델에게 보여주는 샘플 수. 크면 안정적이지만 GPU 메모리를 많이 씁니다.',
      expert: '1회 forward/backward에 처리하는 샘플 수. 클수록 안정적이지만 VRAM↑.\n\n24GB GPU 기준: ACT/Diffusion은 32~64, PI0.5 LoRA는 16~32부터.\n\nVRAM이 빠듯하면 grad_accum_steps로 effective batch를 늘릴 수 있습니다.',
    },
    en: {
      title: 'Batch Size',
      easy: "How many samples the model sees per step. Larger = more stable but uses more GPU memory.",
      expert: 'Samples per forward/backward. Larger = more stable training but more VRAM.\n\nOn a 24GB GPU: 32-64 for ACT/Diffusion, 16-32 for PI0.5 LoRA.\n\nUse grad_accum_steps to simulate a larger batch when VRAM is tight.',
    },
  },

  'Common.image_resolution': {
    ko: {
      title: '이미지 해상도 (image_resolution)',
      easy: '학습 전에 모든 카메라 이미지를 이 (높이, 너비)로 맞춰줍니다. 사이즈가 다른 데이터를 함께 학습할 수 있도록 통일해 주는 역할입니다. 잘 모르겠으면 [224, 224] 그대로 두세요.',
      expert: '전처리 파이프라인의 통일 resize 타겟. 각 frame을 (H, W)로 강제 리사이즈한 뒤 텐서화하기 때문에, 서로 다른 카메라 해상도로 수집된 데이터셋을 한 학습 run에서 자유롭게 혼합할 수 있습니다.\n\n- ResNet: torchvision Resize → ToTensor\n- DINOv2/v3: HF AutoImageProcessor를 do_resize=True + do_center_crop=False + size={H,W}로 override하여 동일 타겟 사용\n- Augment 단계의 RandomResizedCrop도 이 사이즈를 따라감\n\n학습 종료 시 `<checkpoint_dir>/train_meta.json`에 기록되어, 추론 시 동일 사이즈로 재현됩니다. 큰 값(e.g. 448, 448)은 세밀한 위치 정보를 더 보존하지만 token/연산이 그만큼 증가.',
      note: '학습/추론은 자동으로 같은 값을 사용합니다. checkpoint 단위로 저장되므로 정책마다 다른 사이즈를 쓸 수 있습니다.',
    },
    en: {
      title: 'Image Resolution (image_resolution)',
      easy: 'Every camera frame is resized to this (Height, Width) before training, so datasets recorded at different resolutions can be mixed in one run. Leave as [224, 224] if unsure.',
      expert: "Unified resize target for the preprocessing pipeline. Each frame is forced to (H, W) before tensorization, which lets you mix datasets recorded with different camera resolutions in a single training run.\n\n- ResNet: torchvision Resize → ToTensor\n- DINOv2/v3: HF AutoImageProcessor is overridden with do_resize=True + do_center_crop=False + size={H,W}\n- The training-time RandomResizedCrop augmentation follows this size too\n\nThe value is persisted to `<checkpoint_dir>/train_meta.json` and reused at inference for matching preprocessing. Larger values (e.g. 448x448) preserve more fine-grained spatial cues at the cost of compute/tokens.",
      note: 'Training and inference automatically use the same value via the checkpoint sidecar — different policies can use different sizes.',
    },
  },

  'Common.num_workers': {
    ko: {
      title: 'DataLoader Worker 수',
      easy: '데이터 로딩에 쓰는 CPU 일꾼 수. CPU 코어의 절반 정도가 무난.',
      expert: 'DataLoader가 사용하는 CPU 프로세스 수. CPU 코어 수의 절반이 무난한 기준.\n\n많을수록 데이터 적재가 빨라져 GPU가 노는 시간이 줄어듭니다.',
    },
    en: {
      title: 'Number of Workers',
      easy: 'CPU helpers for loading data. About half your CPU cores is a good rule.',
      expert: 'CPU processes used by the DataLoader. ~Half your CPU core count is a solid rule of thumb.\n\nMore workers reduce data-loading bottlenecks so the GPU stays busy.',
    },
  },

  'Common.use_peft': {
    ko: {
      title: 'LoRA (PEFT) 사용',
      easy: '전체 모델을 학습하지 않고 작은 보조 부품(어댑터)만 학습합니다. PI0.5처럼 큰 모델을 단일 GPU에서 학습할 때 켜세요.',
      expert: '전체 weights 대신 작은 LoRA 어댑터만 학습. VRAM을 크게 절약해 단일 GPU에서 큰 모델 fine-tune 가능.\n\n- PI05: vision_tower(SigLIP) + paligemma language_model + gemma_expert의 attn/MLP에 세밀하게 타깃팅\n- ACT/Diffusion: "all-linear" 일반 타깃 적용\n\n이미 작은 ACT/Diffusion에는 보통 끕니다.',
    },
    en: {
      title: 'Use LoRA (PEFT)',
      easy: "Train only small adapter modules instead of the whole model. Turn on for large models like PI0.5 on a single GPU.",
      expert: 'Trains small LoRA adapters instead of full weights — large VRAM savings, makes big models fine-tunable on one GPU.\n\n- PI05: fine-grained targeting of vision_tower (SigLIP) + paligemma language_model + gemma_expert attn/MLP\n- ACT/Diffusion: generic "all-linear" target\n\nUsually off for already-small models (ACT/Diffusion).',
    },
  },

  'Common.peft_r': {
    ko: {
      title: 'LoRA Rank (r)',
      easy: 'LoRA 어댑터의 "용량". 크면 표현력↑·메모리↑. PI0.5엔 64가 기본.',
      expert: 'LoRA adapter의 low-rank 차원. 작으면(4~16) 가볍고, 크면(32~128) 표현력 좋음.\n\nPI0.5 LoRA 기본 64. 데이터가 매우 적으면 16~32로 줄여도 됩니다.',
    },
    en: {
      title: 'LoRA Rank',
      easy: 'LoRA adapter "capacity". Larger = more expressive but more memory. 64 default for PI0.5.',
      expert: 'Low-rank dimension of the LoRA adapter. Small (4-16) = light, large (32-128) = expressive.\n\n64 is the PI0.5 LoRA default. Drop to 16-32 on very small datasets.',
    },
  },

  'Common.peft_alpha': {
    ko: {
      title: 'LoRA Alpha',
      easy: 'LoRA 업데이트의 "음량 크기". alpha = r(스케일 1.0)가 가장 안정적이에요.',
      expert: 'LoRA delta에 곱해지는 scale factor. effective scale = alpha / r.\n\nalpha = r (scale 1.0)이 가장 안정. alpha > r이면 LoRA 업데이트가 과장돼 작은 데이터셋에서 노이즈가 증폭됩니다 (과거 alpha=128, r=64 조합에서 로봇 동작이 erratic해진 사례).',
    },
    en: {
      title: 'LoRA Alpha',
      easy: '"Volume" of LoRA updates. alpha = r (scale 1.0) is the safest setting.',
      expert: 'Scale factor on the LoRA delta. effective scale = alpha / r.\n\nalpha = r (scale 1.0) is safest. alpha > r amplifies LoRA updates and can amplify noisy gradients on small datasets (alpha=128 with r=64 once caused erratic robot behavior).',
    },
  },

  'Common.use_amp': {
    ko: {
      title: 'Mixed Precision (bf16)',
      easy: '학습을 16-bit로 돌려서 속도↑·VRAM↓. 최신 NVIDIA GPU(A100, RTX 30/40)면 켜는 게 좋아요.',
      expert: 'torch.autocast(dtype=torch.bfloat16)로 학습을 bf16 정밀도로 진행 — 속도↑·VRAM↓.\n\nAmpere(A100, RTX 30 시리즈) 이상이면 거의 손해 없이 사용 가능. RTX 20 시리즈처럼 bf16 미지원 GPU에선 꺼야 합니다.',
    },
    en: {
      title: 'Mixed Precision (bf16)',
      easy: "Trains in 16-bit precision — faster and uses less VRAM. Turn on for modern NVIDIA GPUs (A100, RTX 30/40).",
      expert: 'Trains in bfloat16 via torch.autocast(dtype=torch.bfloat16) — faster + less VRAM.\n\nSafe to enable on Ampere (A100, RTX 30 series) and newer. Disable on older GPUs (e.g. RTX 20 series) that lack bf16 support.',
    },
  },

  'Common.grad_accum_steps': {
    ko: {
      title: 'Gradient Accumulation Steps',
      easy: 'GPU 메모리가 작아서 큰 배치를 못 쓸 때, 여러 번 나눠서 모았다가 한 번에 업데이트하는 방법.',
      expert: 'N step의 gradient를 모은 후 한 번 optimizer.step() 호출. effective batch = batch_size × N.\n\nVRAM 부족 시 효과적 배치 크기를 늘리는 가장 쉬운 방법.',
    },
    en: {
      title: 'Gradient Accumulation Steps',
      easy: "When VRAM is tight, accumulates several steps' gradients before updating to simulate a larger batch.",
      expert: 'Accumulate gradients over N steps before calling optimizer.step(). Effective batch = batch_size × N.\n\nEasiest way to grow effective batch size when VRAM is tight.',
    },
  },

  'Common.ema_decay': {
    ko: {
      title: 'EMA Decay',
      easy: '학습 중인 가중치의 "부드러운 평균본"을 따로 유지해서 추론에 씁니다. 0이면 끔, 0.99면 켬. Full fine-tune엔 0.99 권장.',
      expert: '학습 중 weights의 EMA(지수이동평균) 사본을 유지해 추론에 사용. 학습 노이즈를 줄여 더 안정적 모델.\n\n- 0: EMA 끔 (기본)\n- 0.99: Full fine-tune에 권장\n- LoRA에선 0 권장 (openpi LoRA preset과 일치)',
    },
    en: {
      title: 'EMA Decay',
      easy: 'Keeps a smoothed copy of weights for inference. 0 = off, 0.99 = on. Recommended 0.99 for full fine-tune only.',
      expert: 'Maintains an exponential moving average of weights during training and uses it for inference — smooths out noise.\n\n- 0: EMA off (default)\n- 0.99: recommended for full fine-tune\n- LoRA: keep at 0 (matches openpi LoRA preset)',
    },
  },

  'Common.val_smooth_window': {
    ko: {
      title: 'Validation Smoothing Window',
      easy: 'Validation 손실을 N 에폭 평균으로 부드럽게 봐서 "베스트 모델"을 더 안정적으로 고릅니다. 1이면 끄기.',
      expert: 'Validation loss를 N-에폭 moving average로 부드럽게 만든 후 best checkpoint를 선택. 한 번의 노이즈성 좋은 에폭에 휘둘리지 않게.\n\n작은 val 셋에서 outlier 에폭 피하려면 5 정도로.',
    },
    en: {
      title: 'Val Smooth Window',
      easy: "Smooths validation loss over N epochs to pick the best checkpoint more reliably. 1 = off.",
      expert: "Smooths validation loss with an N-epoch moving average when selecting the 'best' checkpoint. Reduces sensitivity to single noisy epochs.\n\nIncrease to ~5 to avoid picking a lucky-noise outlier on small val sets.",
    },
  },

  'Common.val_n_passes': {
    ko: {
      title: 'Validation Pass 수',
      easy: '에폭마다 validation을 N번 반복해서 평균냄. validation 셋이 작아 들쭉날쭉할 때 늘리세요.',
      expert: '에폭당 validation을 N회 반복하고 평균. 작은 val 셋에서 loss 노이즈가 매우 큰 경우에 유용.',
    },
    en: {
      title: 'Val Passes per Epoch',
      easy: 'Runs validation N times per epoch and averages. Useful when the val set is tiny and noisy.',
      expert: 'Run validation N times per epoch and average. Useful when a small val set makes loss very noisy.',
    },
  },

  'Common.wrist_sensor_ids': {
    ko: {
      title: '손목 카메라 센서 ID',
      easy: '손목에 달린 카메라 ID를 콤마로 적어주세요 (예: "1" 또는 "2,3"). 손목 카메라는 그리퍼 기준이라 화면을 자르거나 회전시키면 안 돼서 자동으로 증강에서 제외해줘요.',
      expert: '손목 장착 카메라의 sensor ID를 콤마로 구분 (예: "1" 또는 "2,3"). train_worker.py에서 파싱해 backend/policies/utils.py의 공간 증강(crop/rotate) 단계에서 제외 처리.\n\n손목 카메라는 EE 좌표계에 고정돼 있어 random crop/회전이 그리퍼↔물체 기하 관계를 깨뜨립니다.\n\n비워두면 모든 카메라에 동일한 증강 적용.',
    },
    en: {
      title: 'Wrist Sensor IDs',
      easy: 'Comma-separated IDs of wrist-mounted cameras (e.g. "1" or "2,3"). They follow the gripper, so the system automatically skips spatial augmentations that would warp the geometry.',
      expert: "Comma-separated wrist-camera sensor IDs (e.g. '1' or '2,3'). Parsed in train_worker.py and used by backend/policies/utils.py to skip spatial augmentations (crop/rotate).\n\nWrist cameras are fixed to the EE frame, so random crops/rotations break the gripper↔scene geometry critical for fine manipulation.\n\nLeave empty to apply the same augmentation to all cameras.",
    },
  },
}

// ------------------------------------------------------------
// Shared entries: action_type and obs_state_keys are common to
// ACT / Diffusion / PI05 and are handled at the dataset preprocessing
// layer (train_worker.py) regardless of which policy is selected.
// ------------------------------------------------------------
const SHARED_ACTION_TYPE = {
  ko: {
    title: '액션 타입 (action_type)',
    easy: '학습할 동작을 어떤 좌표계로 표현할지 정합니다. 기본은 "joint"(관절 각도) — 가장 정확하고 일반적이에요.',
    expert: '데이터셋의 액션을 어떤 표현으로 변환할지 결정 — train_worker.py에서 dataset transform 시점에 적용됩니다 (정책 모델 코드와 별개).\n\n- joint: 관절 각도 (가장 일반적, 정밀)\n- ee_delta: 엔드이펙터 좌표 변화량 (작업 공간 추론에 가까움)\n- relative_ee_pos: 첫 프레임 대비 상대 EE 위치\n\n학습 시 자동으로 데이터셋이 변환됩니다.',
  },
  en: {
    title: 'Action Type',
    easy: 'Which coordinate system to express actions in. Default "joint" (joint angles) — most accurate and common.',
    expert: 'How the dataset actions are transformed for training — handled in train_worker.py at the dataset preprocessing step (independent of the policy model code).\n\n- joint: joint angles (most common, most precise)\n- ee_delta: end-effector position deltas (closer to task-space reasoning)\n- relative_ee_pos: EE position relative to the first frame\n\nThe dataset is converted automatically at training start.',
  },
}

const SHARED_OBS_STATE_KEYS = {
  ko: {
    title: '관찰 상태 키 (obs_state_keys)',
    easy: '모델에게 보여줄 로봇 상태 종류를 고릅니다. 보통 관절 각도(qpos)만으로 충분해요.',
    expert: '정책 입력에 포함할 로봇 상태 채널 — train_worker.py가 dataset feature 선택에 사용합니다.\n\n- qpos: 관절 각도 (사실상 필수)\n- qvel: 관절 속도\n- qeffort: 관절 토크\n- eepos: 엔드이펙터 위치\n\n많이 넣을수록 정보↑·과적합 위험↑. 처음은 qpos만, 떨림이나 동적 작업에서 qvel 추가가 일반적.',
  },
  en: {
    title: 'Observation State Keys',
    easy: 'Which robot-state channels the policy sees. Joint positions (qpos) alone are usually enough.',
    expert: 'Which robot-state channels go into the policy — used by train_worker.py to pick dataset features.\n\n- qpos: joint positions (effectively required)\n- qvel: joint velocities\n- qeffort: joint torques\n- eepos: end-effector position\n\nMore channels = more info but also more overfitting risk. Start with qpos; add qvel for shaky or dynamic tasks.',
  },
}

HYPERPARAM_HELP['ACT.action_type'] = SHARED_ACTION_TYPE
HYPERPARAM_HELP['Diffusion.action_type'] = SHARED_ACTION_TYPE
HYPERPARAM_HELP['PI05.action_type'] = SHARED_ACTION_TYPE
HYPERPARAM_HELP['ACT.obs_state_keys'] = SHARED_OBS_STATE_KEYS
HYPERPARAM_HELP['Diffusion.obs_state_keys'] = SHARED_OBS_STATE_KEYS
HYPERPARAM_HELP['PI05.obs_state_keys'] = SHARED_OBS_STATE_KEYS

/**
 * Look up help text for a hyperparameter.
 * @param {string|null} policyType - 'ACT' | 'Diffusion' | 'PI05'
 * @param {string} key             - parameter key, e.g. 'n_obs_steps'
 * @returns {{ko, en}|null}
 */
export function getHyperparamHelp(policyType, key) {
  if (policyType && HYPERPARAM_HELP[policyType + '.' + key]) {
    return HYPERPARAM_HELP[policyType + '.' + key]
  }
  if (HYPERPARAM_HELP['Common.' + key]) {
    return HYPERPARAM_HELP['Common.' + key]
  }
  return null
}
