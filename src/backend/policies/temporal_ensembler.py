import torch
from torch import Tensor


class TemporalEnsembler:
    """Policy-agnostic temporal ensembler for action-chunking policies.

    Any policy that can predict a (batch, chunk_size, action_dim) action chunk can use this.
    Applies exponential weighted averaging across overlapping predictions and computes
    per-action standard deviation as an uncertainty signal.

    Args:
        temporal_ensemble_coeff: Exponential weight decay coefficient.
            - 0  = uniform weighting across all predictions
            - >0 = older predictions weighted more heavily (original ACT default: 0.01)
            - <0 = newer predictions weighted more heavily
        chunk_size: Number of actions in each predicted chunk.
    """

    def __init__(self, temporal_ensemble_coeff: float, chunk_size: int) -> None:
        self.chunk_size = chunk_size
        self.ensemble_weights = torch.exp(-temporal_ensemble_coeff * torch.arange(chunk_size))
        self.ensemble_weights_cumsum = torch.cumsum(self.ensemble_weights, dim=0)
        self.reset()

    def reset(self):
        """Resets the online computation variables."""
        self.ensembled_actions = None
        self.ensembled_actions_count = None
        self.ensembled_actions_m2 = None

    def update(self, actions: Tensor) -> tuple[Tensor, Tensor]:
        """Takes a (batch, chunk_size, action_dim) action chunk, updates the temporal ensemble,
        and returns the next action along with its standard deviation (uncertainty).
        """
        self.ensemble_weights = self.ensemble_weights.to(device=actions.device)
        self.ensemble_weights_cumsum = self.ensemble_weights_cumsum.to(device=actions.device)

        if self.ensembled_actions is None:
            self.ensembled_actions = actions.clone()
            self.ensembled_actions_count = torch.ones(
                (self.chunk_size, 1), dtype=torch.long, device=self.ensembled_actions.device
            )
            self.ensembled_actions_m2 = torch.zeros_like(self.ensembled_actions)
        else:
            new_actions_chunk = actions[:, :-1]
            current_weights = self.ensemble_weights[self.ensembled_actions_count]
            old_mean = self.ensembled_actions.clone()

            self.ensembled_actions *= self.ensemble_weights_cumsum[self.ensembled_actions_count - 1]
            self.ensembled_actions += actions[:, :-1] * self.ensemble_weights[self.ensembled_actions_count]
            self.ensembled_actions /= self.ensemble_weights_cumsum[self.ensembled_actions_count]

            self.ensembled_actions_m2 += current_weights * (new_actions_chunk - old_mean) * (new_actions_chunk - self.ensembled_actions)

            self.ensembled_actions_count = torch.clamp(self.ensembled_actions_count + 1, max=self.chunk_size)
            self.ensembled_actions = torch.cat([self.ensembled_actions, actions[:, -1:]], dim=1)
            self.ensembled_actions_count = torch.cat(
                [self.ensembled_actions_count, torch.ones_like(self.ensembled_actions_count[-1:])]
            )
            self.ensembled_actions_m2 = torch.cat(
                [self.ensembled_actions_m2, torch.zeros_like(actions[:, -1:])], dim=1
            )

        action = self.ensembled_actions[:, 0]
        action_m2 = self.ensembled_actions_m2[:, 0]
        action_count = self.ensembled_actions_count[0]

        if action_count.item() <= 1:
            variance = torch.zeros_like(action_m2)
        else:
            sum_of_weights = self.ensemble_weights_cumsum[action_count - 1]
            variance = action_m2 / sum_of_weights

        std_dev = torch.sqrt(variance + 1e-8)

        self.ensembled_actions = self.ensembled_actions[:, 1:]
        self.ensembled_actions_count = self.ensembled_actions_count[1:]
        self.ensembled_actions_m2 = self.ensembled_actions_m2[:, 1:]

        return action, std_dev
