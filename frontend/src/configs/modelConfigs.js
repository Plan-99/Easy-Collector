export const POLICY_CONFIGS = {
    'ACT': { // This part can be changed to the desired policy name, e.g., 'ACT'.
        'n_obs_steps': { 'label': 'Number of Observation Steps', 'value': 1, 'type': 'number' },
        'chunk_size': { 'label': 'Chunk Size', 'value': 15, 'type': 'number' },
        'n_action_steps': { 'label': 'Number of Action Steps', 'value': 1, 'type': 'number' },
        'temporal_ensemble_coeff': { 'label': 'Temporal Ensemble Coefficient', 'value': 0.01, 'type': 'number', 'nullable': true },
        'vision_backbone': { 
            'label': 'Vision Backbone', 
            'value': 'resnet18', 
            'type': 'select', 
            'options': ['resnet18', 'resnet34', 'resnet50', 'dinov2', 'dinov3'] // Assuming DinoV2 is a valid option,
        },
        // torchvision `weights=...` enum string used to init the ResNet backbone.
        // Ignored on DINO backbones (DinoVisionBackbone loads via HF AutoModel),
        // so the UI hides this field when a dinov* backbone is selected. We mark
        // it nullable because (a) `weights=None` is a legitimate ResNet choice
        // (random init) and (b) when the user switches to DINO the watcher resets
        // the value to null — the validator would otherwise block submit even
        // though the field is hidden.
        'pretrained_backbone_weights': {
            'label': 'Pretrained Backbone Weights',
            value: 'ResNet18_Weights.IMAGENET1K_V1',
            type: 'select',
            nullable: true,
            hideIfBackbone: 'dinov',
            options: {
                'resnet18': ['ResNet18_Weights.IMAGENET1K_V1', null],
                'resnet34': ['ResNet34_Weights.IMAGENET1K_V1', null],
                'resnet50': ['ResNet50_Weights.IMAGENET1K_V1', null],
            }
        },
        'replace_final_stride_with_dilation': { 'label': 'Replace Final Stride', 'value': false, 'type': 'boolean' },
        // DINO partial unfreeze: 0 = fully frozen, N = unfreeze last N transformer blocks.
        // Only meaningful when vision_backbone is dinov2/dinov3 (ResNet path ignores it).
        // UI hides this field unless a DINO backbone is selected.
        'n_unfrozen_blocks': { 'label': 'DINO Unfrozen Blocks', 'value': 0, 'type': 'number', 'showIfBackbone': 'dinov' },
        'pre_norm': { 'label': 'Pre-Normalization', 'value': false, 'type': 'boolean' },
        'dim_model': { 'label': 'Model Dimension', 'value': 512, 'type': 'number' },
        'n_heads': { 'label': 'Number of Heads', 'value': 8, 'type': 'number' },
        'dim_feedforward': { 'label': 'Feedforward Dimension', 'value': 3200, 'type': 'number' },
        'feedforward_activation': { 'label': 'Feedforward Activation', 'value': 'relu', 'type': 'select', 'options': ['relu', 'gelu', 'silu'] },
        'n_encoder_layers': { 'label': 'Number of Encoder Layers', 'value': 4, 'type': 'number' },
        'n_decoder_layers': { 'label': 'Number of Decoder Layers', 'value': 1, 'type': 'number' },
        'use_vae': { 'label': 'Use VAE', 'value': true, 'type': 'boolean' },
        'latent_dim': { 'label': 'Latent Dimension', 'value': 32, 'type': 'number' },
        'n_vae_encoder_layers': { 'label': 'Number of VAE Encoder Layers', 'value': 4, 'type': 'number' },
        'action_type': { 'label': 'Action Type', 'value': 'joint', 'type': 'select', 'options': ['joint', 'ee_delta', 'relative_ee_pos'] },
        'obs_state_keys': { 'label': 'Observation State Keys', 'value': ['qpos'], 'type': 'multiselect', 'options': ['qpos', 'qvel', 'qeffort', 'eepos'] },
    },
    'Diffusion': {
        // --- 데이터 형식 및 시퀀스 관련 ---
        'n_obs_steps': { 'label': 'Number of Observation Steps', 'value': 2, 'type': 'number' },
        'horizon': { 'label': 'Horizon', 'value': 16, 'type': 'number' },
        'n_action_steps': { 'label': 'Number of Action Steps', 'value': 8, 'type': 'number' },
        'drop_n_last_frames': { 'label': 'Drop N Last Frames', 'value': 7, 'type': 'number' },

        // --- Vision Backbone 구조 ---
        'vision_backbone': { 'label': 'Vision Backbone', 'value': 'resnet18', 'type': 'select', 'options': ['resnet18', 'resnet34', 'resnet50'] },
        'crop_shape': { 'label': 'Crop Shape (H, W)', 'value': [150, 200], 'type': 'array' },
        'use_group_norm': { 'label': 'Use Group Norm', 'value': true, 'type': 'boolean' },
        'spatial_softmax_num_keypoints': { 'label': 'Spatial Softmax Keypoints', 'value': 32, 'type': 'number' },
        'use_separate_rgb_encoder_per_camera': { 'label': 'Use Separate RGB Encoder per Camera', 'value': false, 'type': 'boolean' },

        // --- U-Net 구조 ---
        'down_dims': { 'label': 'Down Dims', 'value': [256, 512, 1024], 'type': 'array' },
        'kernel_size': { 'label': 'Kernel Size', 'value': 5, 'type': 'number' },
        'n_groups': { 'label': 'Number of Groups for Norm', 'value': 8, 'type': 'number' },
        'diffusion_step_embed_dim': { 'label': 'Diffusion Step Embedding Dimension', 'value': 128, 'type': 'number' },
        'use_film_scale_modulation': { 'label': 'Use FiLM Scale Modulation', 'value': true, 'type': 'boolean' },

        // --- Noise Scheduler ---
        'noise_scheduler_type': { 'label': 'Noise Scheduler Type', 'value': 'DDPM', 'type': 'select', 'options': ['DDPM', 'DDIM'] },
        'num_train_timesteps': { 'label': 'Number of Train Timesteps', 'value': 100, 'type': 'number' },
        'beta_schedule': { 'label': 'Beta Schedule', 'value': 'squaredcos_cap_v2', 'type': 'select', 'options': ['linear', 'squaredcos_cap_v2'] },
        'beta_start': { 'label': 'Beta Start', 'value': 0.0001, 'type': 'number' },
        'beta_end': { 'label': 'Beta End', 'value': 0.02, 'type': 'number' },
        'prediction_type': { 'label': 'Prediction Type', 'value': 'epsilon', 'type': 'select', 'options': ['epsilon', 'sample'] },
        'clip_sample': { 'label': 'Clip Sample', 'value': true, 'type': 'boolean' },
        'clip_sample_range': { 'label': 'Clip Sample Range', 'value': 1.0, 'type': 'number' },

        // --- Loss 계산 ---
        'do_mask_loss_for_padding': { 'label': 'Mask Loss for Padding', 'value': false, 'type': 'boolean' },
        'action_type': { 'label': 'Action Type', 'value': 'joint', 'type': 'select', 'options': ['joint', 'ee_delta', 'relative_ee_pos'] },
        'obs_state_keys': { 'label': 'Observation State Keys', 'value': ['qpos'], 'type': 'multiselect', 'options': ['qpos', 'qvel', 'qeffort', 'eepos'] },
    },
    'PI05': {
        // HuggingFace access token for gated PaliGemma/Gemma models
        'hf_token': { 'label': 'HuggingFace Token (for gated PaliGemma)', 'value': '', 'type': 'password', 'nullable': true },
        // Input / output structure
        'n_obs_steps': { 'label': 'Number of Observation Steps', 'value': 1, 'type': 'number' },
        'chunk_size': { 'label': 'Chunk Size', 'value': 50, 'type': 'number' },
        'n_action_steps': { 'label': 'Number of Action Steps', 'value': 50, 'type': 'number' },
        'max_state_dim': { 'label': 'Max State Dimension', 'value': 32, 'type': 'number' },
        'max_action_dim': { 'label': 'Max Action Dimension', 'value': 32, 'type': 'number' },
        // Image preprocessing
        'image_resolution': { 'label': 'Image Resolution (H, W)', 'value': [224, 224], 'type': 'array' },
        // Flow matching
        'num_inference_steps': { 'label': 'Number of Inference Steps', 'value': 10, 'type': 'number' },
        // Tokenizer
        'tokenizer_max_length': { 'label': 'Tokenizer Max Length', 'value': 200, 'type': 'number' },
        // Model variants
        'paligemma_variant': { 'label': 'PaliGemma Variant', 'value': 'gemma_2b', 'type': 'select', 'options': ['gemma_2b'] },
        'action_expert_variant': { 'label': 'Action Expert Variant', 'value': 'gemma_300m', 'type': 'select', 'options': ['gemma_300m'] },
        // Finetuning settings — defaults tuned for single-GPU + LoRA fine-tune from pi05_base.
        // Paper §IV.D trains the full model end-to-end (all three = false) but that requires
        // multi-GPU + 400h data. For typical small-dataset fine-tune, freezing the VLM path
        // preserves PaliGemma's web-scale knowledge and keeps VRAM within 24GB.
        'freeze_vision_encoder': { 'label': 'Freeze Vision Encoder', 'value': true, 'type': 'boolean' },
        'train_expert_only': { 'label': 'Train Expert Only', 'value': true, 'type': 'boolean' },
        'gradient_checkpointing': { 'label': 'Gradient Checkpointing', 'value': true, 'type': 'boolean' },
        // Action target encoding.
        //   - 'joint'             : 절대 joint target (raw)
        //   - 'relative_joint_pos': chunk-anchored joint delta. anchor = observation.qpos
        //                           컬럼에서 직접 (obs_state_keys와 독립). PI0.5 권장.
        //   - 'ee_delta'          : EE world-frame per-step delta
        //   - 'relative_ee_pos'   : EE local-frame chunk trajectory (SE(3) transform)
        'action_type': { 'label': 'Action Type', 'value': 'relative_joint_pos', 'type': 'select', 'options': ['joint', 'relative_joint_pos', 'ee_delta', 'relative_ee_pos'] },
        // gripper / tool / done dim은 backend가 dataset features (action.joint names +
        // has_succeed) 에서 자동 식별해서 absolute로 유지. user 설정 불필요.
        'obs_state_keys': { 'label': 'Observation State Keys', 'value': ['qpos'], 'type': 'multiselect', 'options': ['qpos', 'qvel', 'qeffort', 'eepos'] },
    }
    // 'VLAsEn': {
    //     'n_obs_steps': { 'label': 'Number of Observation Steps', 'value': 1, 'type': 'number' },
    //     'chunk_size': { 'label': 'Chunk Size', 'value': 15, 'type': 'number' },
    //     'n_action_steps': { 'label': 'Number of Action Steps', 'value': 1, 'type': 'number' },
    //     'temporal_ensemble_coeff': { 'label': 'Temporal Ensemble Coefficient', 'value': 0.01, 'type': 'number', 'nullable': true },
    //     'vision_backbone': { 
    //         'label': 'Vision Backbone', 
    //         'value': 'resnet18', 
    //         'type': 'select', 
    //         'options': ['resnet18', 'resnet34', 'resnet50', 'dinov2'] // Assuming DinoV2 is a valid option,
    //     },
    //     'pretrained_backbone_weights': { 
    //         'label': 'Pretrained Backbone Weights', 
    //         value: 'ResNet18_Weights.IMAGENET1K_V1', 
    //         type: 'select',
    //         options: {
    //             'resnet18': ['ResNet18_Weights.IMAGENET1K_V1'], 
    //             'resnet34': ['ResNet34_Weights.IMAGENET1K_V1'], 
    //             'resnet50': ['ResNet50_Weights.IMAGENET1K_V1'],
    //             'dinov2': ['dinov2_vits14'] // Assuming DinoV2 is a valid option
    //         }
    //     },
    //     'replace_final_stride_with_dilation': { 'label': 'Replace Final Stride', 'value': false, 'type': 'boolean' },
    //     'pre_norm': { 'label': 'Pre-Normalization', 'value': false, 'type': 'boolean' },
    //     'dim_model': { 'label': 'Model Dimension', 'value': 512, 'type': 'number' },
    //     'n_heads': { 'label': 'Number of Heads', 'value': 8, 'type': 'number' },
    //     'dim_feedforward': { 'label': 'Feedforward Dimension', 'value': 3200, 'type': 'number' },
    //     'feedforward_activation': { 'label': 'Feedforward Activation', 'value': 'relu', 'type': 'select', 'options': ['relu', 'gelu', 'silu'] },
    //     'n_encoder_layers': { 'label': 'Number of Encoder Layers', 'value': 4, 'type': 'number' },
    //     'n_decoder_layers': { 'label': 'Number of Decoder Layers', 'value': 1, 'type': 'number' },
    //     'use_vae': { 'label': 'Use VAE', 'value': true, 'type': 'boolean' },
    //     'latent_dim': { 'label': 'Latent Dimension', 'value': 32, 'type': 'number' },
    //     'n_vae_encoder_layers': { 'label': 'Number of VAE Encoder Layers', 'value': 4, 'type': 'number' }
    // }
};

export const TRAIN_CONFIGS = {
    'ACT': {
        'dropout': { 'label': 'Dropout Rate', 'value': 0.1, 'type': 'number' },
        'kl_weight': { 'label': 'KL Divergence Weight', 'value': 10, 'type': 'number' },
        'optimizer_lr': { 'label': 'Optimizer Learning Rate', 'value': 1e-5, 'type': 'number' },
        'optimizer_weight_decay': { 'label': 'Optimizer Weight Decay', 'value': 1e-4, 'type': 'number' },
        'optimizer_lr_backbone': { 'label': 'Optimizer Backbone Learning Rate', 'value': 1e-5, 'type': 'number' }
    },
    'Diffusion': {
        // --- 데이터 증강 ---
        // 'crop_is_random': { 'label': 'Use Random Crop', 'value': true, 'type': 'boolean' },

        // --- Optimizer 설정 ---
        'optimizer_lr': { 'label': 'Optimizer Learning Rate', 'value': 1e-4, 'type': 'number' },
        'optimizer_betas': { 'label': 'Optimizer Betas', 'value': [0.95, 0.999], 'type': 'array' },
        'optimizer_eps': { 'label': 'Optimizer Epsilon', 'value': 1e-8, 'type': 'number' },
        'optimizer_weight_decay': { 'label': 'Optimizer Weight Decay', 'value': 1e-6, 'type': 'number' },

        // --- Scheduler 설정 ---
        'scheduler_name': { 'label': 'LR Scheduler Name', 'value': 'cosine', 'type': 'select', 'options': ['cosine', 'linear', 'constant'] },
        'scheduler_warmup_steps': { 'label': 'LR Scheduler Warmup Steps', 'value': 500, 'type': 'number' }
    },
    'PI05': {
        'optimizer_lr': { 'label': 'Optimizer Learning Rate', 'value': 2.5e-5, 'type': 'number' },
        'optimizer_betas': { 'label': 'Optimizer Betas', 'value': [0.9, 0.95], 'type': 'array' },
        'optimizer_eps': { 'label': 'Optimizer Epsilon', 'value': 1e-8, 'type': 'number' },
        // openpi 기본값은 1e-10 (사실상 비활성) — weight decay가 PaliGemma pretrained
        // 가중치를 over-regularize 해서 학습 초기에 잘 학습된 feature를 무너뜨릴 수 있음
        'optimizer_weight_decay': { 'label': 'Optimizer Weight Decay', 'value': 1e-10, 'type': 'number' },
        'optimizer_grad_clip_norm': { 'label': 'Gradient Clip Norm', 'value': 1.0, 'type': 'number' },
        'scheduler_warmup_steps': { 'label': 'Scheduler Warmup Steps', 'value': 1000, 'type': 'number' },
        'scheduler_decay_steps': { 'label': 'Scheduler Decay Steps', 'value': 30000, 'type': 'number' },
        'scheduler_decay_lr': { 'label': 'Scheduler Decay LR', 'value': 2.5e-6, 'type': 'number' },
    },
    'Common': {
        'num_epochs': { 'label': 'Number of Epochs', 'value': 1000, 'type': 'number' },
        'batch_size': { 'label': 'Batch Size', 'value': 32, 'type': 'number' },
        'num_workers': { 'label': 'Number of Workers', 'value': 4, 'type': 'number' },
        // Unified resize target applied in the preprocessing pipeline so datasets with
        // different camera resolutions can be mixed in a single training run.
        // Default (224, 224) matches the previous hard-coded resize used by both
        // ResNet (Resize+ToTensor) and DINO (HF AutoImageProcessor) paths.
        // Inference uses the value stored in the checkpoint's train_meta.json
        // (falls back to (224, 224) when absent).
        'image_resolution': { 'label': 'Image Resolution (H, W)', 'value': [224, 224], 'type': 'array' },
        'use_peft': { 'label': 'Use LoRA (PEFT)', 'value': false, 'type': 'boolean' },
        // alpha=r (scale=1.0) matches openpi's gemma_2b_lora config (rank=alpha=16).
        // Earlier alpha=128 with r=64 gave scale=2.0 which doubled LoRA delta magnitude
        // and amplified noisy LoRA gradients on small datasets — robot behavior became
        // erratic. r=64 keeps capacity, alpha=64 keeps update magnitude controlled.
        'peft_r': { 'label': 'LoRA Rank', 'value': 64, 'type': 'number', 'showIf': 'use_peft' },
        'peft_alpha': { 'label': 'LoRA Alpha', 'value': 64, 'type': 'number', 'showIf': 'use_peft' },
        'use_amp': { 'label': 'Mixed Precision (bf16)', 'value': false, 'type': 'boolean' },
        'grad_accum_steps': { 'label': 'Gradient Accumulation Steps', 'value': 1, 'type': 'number' },
        // EMA + smoothed validation. All defaults are safe-off (preserve prior
        // ACT/Diffusion behavior: no EMA, raw best, single val pass). Opt in for
        // PI05 LoRA on small datasets:
        //   - ema_decay=0 (recommended for LoRA — openpi LoRA preset matches)
        //   - ema_decay=0.99 (recommended for full-FT only)
        //   - val_smooth_window=5 + val_n_passes=3 (recommended for episode-based
        //     small-val-set noise fighting; ckpt 96 hit a noise outlier at epoch 98)
        'ema_decay': { 'label': 'EMA Decay (0=off, 0.99 for full-FT only)', 'value': 0, 'type': 'number' },
        'val_smooth_window': { 'label': 'Val Smooth Window (epochs, 1=raw)', 'value': 1, 'type': 'number' },
        'val_n_passes': { 'label': 'Val Passes per Epoch (1=fast)', 'value': 1, 'type': 'number' },
        // Wrist-mounted camera sensors. Options는 TrainPage가 현재 workspace의
        // sensors 목록으로 runtime injection. openpi skips spatial aug (crop+rotate)
        // on these — wrist cam은 end-effector geometry를 인코딩해서 random crop이
        // gripper↔scene invariance를 파괴.
        'wrist_sensor_ids': { 'label': 'Wrist Sensors', 'value': [], 'type': 'wrist_sensor_select' },
    }
};