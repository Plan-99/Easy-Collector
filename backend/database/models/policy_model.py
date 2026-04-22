from peewee import CharField, TextField, DateTimeField
from ..config.database import SoftDeleteModel
import json
import datetime


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


class Policy(SoftDeleteModel):
    class Meta:
        table_name = 'policies'

    __casts__ = {
        'settings': 'json'
    }

    name = CharField(null=True)
    type = CharField(null=True)
    batch_size = CharField(null=True)
    num_epochs = CharField(null=True)
    settings = TextField(null=True)
    created_at = DateTimeField(default=datetime.datetime.now)
    updated_at = DateTimeField(default=datetime.datetime.now)
    deleted_at = DateTimeField(null=True)

    def save(self, *args, **kwargs):
        self.updated_at = datetime.datetime.now()
        val = getattr(self, 'settings', None)
        if val is not None and not isinstance(val, str):
            self.settings = json.dumps(val)
        return super().save(*args, **kwargs)

    @classmethod
    def create(cls, **kwargs):
        # Apply observer logic
        if 'settings' not in kwargs or not kwargs['settings']:
            policy_type = kwargs.get('type', '')
            if policy_type in POLICY_CONFIGS:
                kwargs['settings'] = POLICY_CONFIGS[policy_type]
            else:
                kwargs['settings'] = {}
        if isinstance(kwargs.get('settings'), (dict, list)):
            kwargs['settings'] = json.dumps(kwargs['settings'])
        return super().create(**kwargs)
