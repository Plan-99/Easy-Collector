POLICY_CONFIGS = {
    'Diffusion': {
        'timesteps': {'default': 100, 'type': int},
        'noise_schedule': {'default': 'cosine', 'type': str}
    },
    'ACT': {
        'chunk_size': {'default': 30, 'type': int},
        'kl_weight': {'default': 10.0, 'type': float},
        'lr': {'default': 1e-5, 'type': float},
        'hidden_dim': {'default': 256, 'type': int},
        'dim_feedforward': {'default': 2048, 'type': int},
        'backbone': {'default': 'resnet18', 'type': str},
        'lr_backbone': {'default': 1e-6, 'type': float},
        'enc_layers': {'default': 4, 'type': int},
        'dec_layers': {'default': 6, 'type': int},
        'nheads': {'default': 8, 'type': int},
        'position_embedding': {'default': 'sine', 'type': str, 'choices': ['sine', 'learned']},
    }
}