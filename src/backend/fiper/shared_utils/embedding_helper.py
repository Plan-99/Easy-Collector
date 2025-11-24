
from backend.lerobot.policies.act.modeling_act import ACTPolicy

class EmbeddingHelper:
    def __init__(self):
        self.model = ACTPolicy.from_pretrained("/root/src/backend/checkpoints/3")



