export const POLICY_CONFIGS = {
    'ACT': { // This part can be changed to the desired policy name, e.g., 'ACT'.
        'n_obs_steps': { 'label': 'Number of Observation Steps', 'value': 1, 'type': 'number' },
        'chunk_size': { 'label': 'Chunk Size', 'value': 15, 'type': 'number' },
        'n_action_steps': { 'label': 'Number of Action Steps', 'value': 1, 'type': 'number' },
        'temporal_ensemble_coeff': { 'label': 'Temporal Ensemble Coefficient', 'value': 0.9, 'type': 'number', 'nullable': true },
        'vision_backbone': { 'label': 'Vision Backbone', 'value': 'resnet18', 'type': 'select', 'options': ['resnet18', 'resnet34', 'resnet50'] },
        'pretrained_backbone_weights': { 'label': 'Pretrained Backbone Weights', 'value': 'ResNet18_Weights.IMAGENET1K_V1', 'type': 'select', options: ['ResNet18_Weights.IMAGENET1K_V1', 'ResNet34_Weights.IMAGENET1K_V1', 'ResNet50_Weights.IMAGENET1K_V1'] },
        'replace_final_stride_with_dilation': { 'label': 'Replace Final Stride', 'value': false, 'type': 'boolean' },
        'pre_norm': { 'label': 'Pre-Normalization', 'value': false, 'type': 'boolean' },
        'dim_model': { 'label': 'Model Dimension', 'value': 512, 'type': 'number' },
        'n_heads': { 'label': 'Number of Heads', 'value': 8, 'type': 'number' },
        'dim_feedforward': { 'label': 'Feedforward Dimension', 'value': 3200, 'type': 'number' },
        'feedforward_activation': { 'label': 'Feedforward Activation', 'value': 'relu', 'type': 'select', 'options': ['relu', 'gelu', 'silu'] },
        'n_encoder_layers': { 'label': 'Number of Encoder Layers', 'value': 4, 'type': 'number' },
        'n_decoder_layers': { 'label': 'Number of Decoder Layers', 'value': 1, 'type': 'number' },
        'use_vae': { 'label': 'Use VAE', 'value': true, 'type': 'boolean' },
        'latent_dim': { 'label': 'Latent Dimension', 'value': 32, 'type': 'number' },
        'n_vae_encoder_layers': { 'label': 'Number of VAE Encoder Layers', 'value': 4, 'type': 'number' }
    },
    'Diffusion': {
        // --- 데이터 형식 및 시퀀스 관련 ---
        'n_obs_steps': { 'label': 'Number of Observation Steps', 'value': 2, 'type': 'number' },
        'horizon': { 'label': 'Horizon', 'value': 16, 'type': 'number' },
        'n_action_steps': { 'label': 'Number of Action Steps', 'value': 8, 'type': 'number' },
        'drop_n_last_frames': { 'label': 'Drop N Last Frames', 'value': 7, 'type': 'number' },

        // --- Vision Backbone 구조 ---
        'vision_backbone': { 'label': 'Vision Backbone', 'value': 'resnet18', 'type': 'select', 'options': ['resnet18', 'resnet34', 'resnet50'] },
        'crop_shape': { 'label': 'Crop Shape (H, W)', 'value': [84, 84], 'type': 'array' },
        'use_group_norm': { 'label': 'Use Group Norm', 'value': true, 'type': 'boolean' },
        'spatial_softmax_num_keypoints': { 'label': 'Spatial Softmax Keypoints', 'value': 32, 'type': 'number' },
        'use_separate_rgb_encoder_per_camera': { 'label': 'Use Separate RGB Encoder per Camera', 'value': false, 'type': 'boolean' },

        // --- U-Net 구조 ---
        'down_dims': { 'label': 'Down Dims', 'value': [512, 1024, 2048], 'type': 'array' },
        'kernel_size': { 'label': 'Kernel Size', 'value': 5, 'type': 'number' },
        'n_groups': { 'label': 'Number of Groups for Norm', 'value': 8, 'type': 'number' },
        'diffusion_step_embed_dim': { 'label': 'Diffusion Step Embedding Dimension', 'value': 128, 'type': 'number' },
        'use_film_scale_modulation': { 'label': 'Use FiLM Scale Modulation', 'value': true, 'type': 'boolean' },

        // --- Noise Scheduler ---
        'noise_scheduler_type': { 'label': 'Noise Scheduler Type', 'value': 'DDPM', 'type': 'select', 'options': ['DDPM', 'DPM-Solver'] },
        'num_train_timesteps': { 'label': 'Number of Train Timesteps', 'value': 100, 'type': 'number' },
        'beta_schedule': { 'label': 'Beta Schedule', 'value': 'squaredcos_cap_v2', 'type': 'select', 'options': ['linear', 'squaredcos_cap_v2'] },
        'beta_start': { 'label': 'Beta Start', 'value': 0.0001, 'type': 'number' },
        'beta_end': { 'label': 'Beta End', 'value': 0.02, 'type': 'number' },
        'prediction_type': { 'label': 'Prediction Type', 'value': 'epsilon', 'type': 'select', 'options': ['epsilon', 'sample'] },
        'clip_sample': { 'label': 'Clip Sample', 'value': true, 'type': 'boolean' },
        'clip_sample_range': { 'label': 'Clip Sample Range', 'value': 1.0, 'type': 'number' },

        // --- Loss 계산 ---
        'do_mask_loss_for_padding': { 'label': 'Mask Loss for Padding', 'value': false, 'type': 'boolean' },
    }
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
    'Common': {
        'num_epochs': { 'label': 'Number of Epochs', 'value': 1000, 'type': 'number' },
        'batch_size': { 'label': 'Batch Size', 'value': 32, 'type': 'number' },
        'num_workers': { 'label': 'Number of Workers', 'value': 4, 'type': 'number' },
    }
};