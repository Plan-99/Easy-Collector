from .base_eval_class import BaseEvalClass
import os
from rnd import RNDBase
from hydra.utils import get_class


class RNDEval(BaseEvalClass):
    """Class for evaluating all RND models."""

    def __init__(self, cfg, method_name, device, task_data_path, dataset, **kwargs):
        super().__init__(cfg, method_name, device, task_data_path, dataset, **kwargs)

    def load_model(self):
        """Load the RND model based on the method_name specified in the configuration file."""
        # Define the model class
        class_name = self.method_name
        if self.method_name.startswith("nrnd"):
            class_name.replace("nrnd", "rnd")
        class_name = f"{class_name.upper()}"
        # Load the model (with hydra)
        checkpoint_dir = os.path.join(self.task_data_path, "rnd_models", self.method_name)
        world_model_dir = os.path.join(self.task_data_path, "sys_id")
        rnd_model: RNDBase = get_class(f"rnd.{class_name}")(
            checkpoint_dir=checkpoint_dir, desired_hparams=self.cfg.hparams.model, world_model_dir=world_model_dir
        )
        rnd_model.to(self.device)
        rnd_model.eval()
        self.model = rnd_model

    def calculate_uncertainty_score(self, rollout_tensor_dict: dict, **kwargs):
        """Calculate the uncertainty score for a single step."""
        for key in rollout_tensor_dict.keys():
            rollout_tensor_dict[key] = rollout_tensor_dict[key].unsqueeze(0).to(self.device)

        uncertainty_scores = (
            self.model(**self.model.datasets_to_model_inputs(datasets=rollout_tensor_dict)).detach().cpu()
        )
        if len(uncertainty_scores) == 1:
            uncertainty_scores = uncertainty_scores.item()
        else:
            uncertainty_scores = uncertainty_scores.numpy()
        return uncertainty_scores

