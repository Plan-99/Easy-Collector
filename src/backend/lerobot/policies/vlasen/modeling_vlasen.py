import torch
from torch import Tensor
from collections import deque


from ...policies.pretrained import PreTrainedPolicy
from ...policies.pi0.modeling_pi0 import PI0Policy
from .modeling_act import ACTPolicy
from .configuration_vlasen import VLAsEnConfig
from ...configs.types import PolicyFeature, FeatureType


class VLAsEnPolicy(PreTrainedPolicy):
    
    config_class = VLAsEnConfig
    name = "vlasen"

    def __init__(
            self,
            config: VLAsEnConfig,
            dataset_stats: dict[str, dict[str, Tensor]] | None = None,
    ):
        super().__init__(config)

        self.config = config
        self.vla_policy = PI0Policy.from_pretrained("lerobot/pi0")
        
        self.vla_policy.config.input_features = self.config.input_features
        self.vla_policy.config.output_features = self.config.output_features
        

    def forward(self, batch: dict[str, Tensor]) -> tuple[Tensor, dict]:
        
        with torch.inference_mode():

            batch['task'] = "pick glue and put it in the orange box" 
            
            for key in batch.keys():
                if key.startswith("observation.images"):
                    batch[key] = batch[key].squeeze(1)
                    print(batch[key].shape)

            
                    

            # print(batch['observation.images.sensor_1'].shape)
            
            action = self.vla_policy.select_action(batch)
            print(action)
            
    def get_optim_params(self) -> dict:
        return self.parameters()
    
    @torch.no_grad()
    def predict_action_chunk(self, batch: dict[str, Tensor]) -> Tensor:
        pass
    
    def reset(self):
        """This should be called whenever the environment is reset."""
        self._action_queue = deque([], maxlen=self.config.n_action_steps)
        
    @torch.no_grad()
    def select_action(self, batch: dict[str, Tensor]) -> Tensor:
        pass
            
        