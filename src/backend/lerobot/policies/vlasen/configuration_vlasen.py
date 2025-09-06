from ...configs.policies import PreTrainedConfig
from ...policies.act.configuration_act import ACTConfig
from ...configs.types import NormalizationMode
from ...optim.optimizers import AdamWConfig
from dataclasses import dataclass, field


@PreTrainedConfig.register_subclass("vlasen")
@dataclass
class VLAsEnConfig(PreTrainedConfig):
    
    type: str = "vlasen"

    # Input / output structure.
    n_obs_steps: int = 1
    chunk_size: int = 15
    n_action_steps: int = 15

    normalization_mapping: dict[str, NormalizationMode] = field(
        default_factory=lambda: {
            "VISUAL": NormalizationMode.MEAN_STD,
            "STATE": NormalizationMode.MEAN_STD,
            "ACTION": NormalizationMode.MEAN_STD,
        }
    )

    # Architecture.
    # Vision backbone.
    vision_backbone: str = "resnet18"
    pretrained_backbone_weights: str | None = "ResNet18_Weights.IMAGENET1K_V1"
    replace_final_stride_with_dilation: int = False
    # Transformer layers.
    pre_norm: bool = False
    dim_model: int = 512
    n_heads: int = 8
    dim_feedforward: int = 3200
    feedforward_activation: str = "relu"
    n_encoder_layers: int = 4
    # Note: Although the original ACT implementation has 7 for `n_decoder_layers`, there is a bug in the code
    # that means only the first layer is used. Here we match the original implementation by setting this to 1.
    # See this issue https://github.com/tonyzhaozh/act/issues/25#issue-2258740521.
    n_decoder_layers: int = 1
    # VAE.
    use_vae: bool = True
    latent_dim: int = 32
    n_vae_encoder_layers: int = 4

    # Inference.
    # Note: the value used in ACT when temporal ensembling is enabled is 0.01.
    temporal_ensemble_coeff: float | None = None

    # Training and loss computation.
    dropout: float = 0.1
    kl_weight: float = 10.0

    # Training preset
    optimizer_lr: float = 1e-5
    optimizer_weight_decay: float = 1e-4
    optimizer_lr_backbone: float = 1e-5
    
    def __post_init__(self):
        super().__post_init__()

        action_model_config = ACTConfig(   
            n_obs_steps=self.n_obs_steps,
            chunk_size=self.chunk_size,
            n_action_steps=self.n_action_steps,
            normalization_mapping=self.normalization_mapping,
            vision_backbone=self.vision_backbone,
            pretrained_backbone_weights=self.pretrained_backbone_weights,
            replace_final_stride_with_dilation=self.replace_final_stride_with_dilation,
            pre_norm=self.pre_norm,
            dim_model=self.dim_model,
            n_heads=self.n_heads,
            dim_feedforward=self.dim_feedforward,
            feedforward_activation=self.feedforward_activation,
            n_encoder_layers=self.n_encoder_layers,
            n_decoder_layers=self.n_decoder_layers,
            use_vae=self.use_vae,
            latent_dim=self.latent_dim,
            n_vae_encoder_layers=self.n_vae_encoder_layers,
            temporal_ensemble_coeff=self.temporal_ensemble_coeff,
            dropout=self.dropout,
            kl_weight=self.kl_weight,
            optimizer_lr=self.optimizer_lr,
            optimizer_weight_decay=self.optimizer_weight_decay,
            optimizer_lr_backbone=self.optimizer_lr_backbone,
        )
        
    def get_optimizer_preset(self) -> AdamWConfig:
        return AdamWConfig(
            lr=self.optimizer_lr,
            weight_decay=self.optimizer_weight_decay,
        )

    def get_scheduler_preset(self) -> None:
        return None

    def validate_features(self) -> None:
        if not self.image_features and not self.env_state_feature:
            raise ValueError("You must provide at least one image or the environment state among the inputs.")
    
    @property
    def observation_delta_indices(self) -> None:
        return None

    @property
    def action_delta_indices(self) -> list:
        return list(range(self.chunk_size))

    @property
    def reward_delta_indices(self) -> None:
        return None
