from orator import Model, SoftDeletes
from orator.orm import belongs_to


POLICY_CONFIGS = {
    'Diffusion': {
        'timesteps': 100,
        'noise_schedule': 'cosine'
    },
    'ACT': {
        'chunk_size': 30,
        'kl_weight': 10.0,
        'lr': 1e-5,
        'hidden_dim': 256,
        'dim_feedforward': 2048,
        'backbone': 'resnet18',
        'lr_backbone': 1e-6,
        'enc_layers': 4,
        'dec_layers': 6,
        'nheads': 8,
        'position_embedding': 'sine',
    }
}

class PolicyObserver:
    def creating(self, policy):
        if not getattr(policy, 'settings', None):
            policy_type = policy.type
            if policy_type in POLICY_CONFIGS:
                policy.settings = POLICY_CONFIGS[policy_type]
            else:
                policy.settings = {}
                

class Policy(Model, SoftDeletes):

    __fillable__ = [
        'name',
        'type',
        'batch_size',
        'num_epochs',
        'settings',
    ]

    __casts__ = {
        'settings': 'json'
    }
    
    __timestamps__ = True
    
    
    @staticmethod
    def boot():
        Policy.observe(PolicyObserver())