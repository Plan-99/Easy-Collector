
from backend.lerobot.policies.act.modeling_act import ACTPolicy
import torch
import numpy as np
from backend.utils.image_parser import fetch_image_with_config
from backend.policies.utils import make_policy, VISION_BACKBONE_MAP, process_image


class EmbeddingHelper:
    def __init__(self):
        self.policy = ACTPolicy.from_pretrained("/root/src/backend/checkpoints/9")
        self.sensors = ["sensor_1", "sensor_2"]  # Example sensor IDs
        self.robots = ["robot_4"]  # Example robot IDs
        self.action_batch_size = 10

    def get_embedding(self, rollout: dict) -> torch.Tensor:
        """
        Given raw observations, compute the ACT embeddings.

        Args:
            observations (dict): A dictionary containing raw observations including 'rgb_images'.

        Returns:
            torch.Tensor: The computed ACT embeddings.

        """
        action_list = []
        embedding_stds = []

        # 1. Save original eval method
        original_eval_method = self.policy.eval

        # 2. Set policy to eval mode, then enable dropout layers
        self.policy.eval()
        # def enable_dropout(m):
        #     if isinstance(m, torch.nn.Dropout):
        #         # print(f"Enabling dropout for: {m}")
        #         m.p = 0.0
        #         m.train()
        # self.policy.apply(enable_dropout)

        # 3. Monkey-patch eval to prevent it from being called inside predict_action_chunk
        self.policy.eval = lambda: self.policy

        try:
            # Loop over timesteps
            for i in range(rollout['observations']['qpos'][self.robots[0]].shape[0]):
                state = {}
                qpos_list = []
                for robot in self.robots:
                    qpos_list.append(rollout['observations']['qpos'][robot][i])
                # qpos = np.concatenate(qpos_list)
                qpos = np.concatenate(qpos_list)
                qpos = torch.from_numpy(qpos).float().cuda().unsqueeze(0)

                state['observation.state'] = qpos
                # state['action'] = rollout['qaction']

                for sensor in self.sensors:
                    image = rollout['observations']['images'][sensor][i]
                    image = process_image(image, 'resnet18', to_cuda=True)
                    state[f'observation.images.{sensor}'] = image.unsqueeze(0)

                state['language_instruction'] = ["Move the robot to the target position."]

                batched_state = {}

                # Loop to perform multiple forward passes with different dropout masks.
                for key, value in state.items():
                    if isinstance(value, torch.Tensor):
                        repeats = [1] * value.dim()
                        repeats[0] = self.action_batch_size
                        batched_state[key] = value.repeat(*repeats)
                    elif isinstance(value, list):
                        batched_state[key] = value * self.action_batch_size
                    else:
                        batched_state[key] = value

                with torch.no_grad():
                    action_batch = self.policy.predict_action_chunk(batched_state)

                embedding_std_step = torch.std(action_batch, dim=0).mean().item()
                embedding_stds.append(embedding_std_step)
                # print(f"Computed embedding with mean std: {embedding_std_step:.6f}")
                action_list.append(action_batch.cpu().numpy())
        finally:
            # 4. Restore original eval method and call it
            self.policy.eval = original_eval_method
            self.policy.eval()

        mean_rollout_std = np.mean(embedding_stds) if embedding_stds else 0
        return action_list, mean_rollout_std

    def get_embedding_batch(self, rollouts: list[dict]) -> list[list[np.ndarray]]:
        """
        Processes a batch of rollouts to generate embeddings.
        
        NOTE: This is a simple sequential implementation for demonstration. For true batch
        performance, the `get_embedding` method itself should be vectorized to process a
        batch of images and states simultaneously on the GPU.
        
        Args:
            rollouts (list[dict]): A list of rollout dictionaries.

        Returns:
            list[list[np.ndarray]]: A list of embedding lists, one for each rollout.
        """
        all_embeddings = []
        all_embedding_stds = []
        for rollout in rollouts:
            embedding, embedding_std = self.get_embedding(rollout)
            # print(rollout.keys())
            print(f"Computed embedding with mean std: {embedding_std:.6f}")

            all_embeddings.append(embedding)
            all_embedding_stds.append(embedding_std)
        return all_embeddings, all_embedding_stds



