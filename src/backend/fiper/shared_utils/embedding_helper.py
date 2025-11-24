
from backend.lerobot.policies.act.modeling_act import ACTPolicy
import torch
import numpy as np
from backend.utils.image_parser import fetch_image_with_config
from backend.policies.utils import make_policy, VISION_BACKBONE_MAP, process_image


class EmbeddingHelper:
    def __init__(self):
        self.policy = ACTPolicy.from_pretrained("/root/src/backend/checkpoints/3")
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
        for i in range(rollout['observations']['qpos'][self.robots[0]].shape[0]):
            state = {}
            qpos_list = []
            for robot in self.robots:
                qpos_list.append(rollout['observations']['qpos'][robot][i])
            qpos = np.concatenate(qpos_list)
            qpos = torch.from_numpy(qpos).float().cuda().unsqueeze(0)

            state['observation.state'] = qpos

            for sensor in self.sensors:
                image = rollout['observations']['images'][sensor][i]
                # image = fetch_image_with_config(image, {
                #     'resize': task['sensor_img_size'],
                # })
                
                image = process_image(image, 'resnet18', to_cuda=True)
                # image = image / 255.0
                # image = torch.from_numpy(image).float().cuda().unsqueeze(0)
                # image = rearrange(image, 'b h w c -> b c h w')
                state[f'observation.images.{sensor}'] = image.unsqueeze(0)

            state['language_instruction'] = ["Move the robot to the target position."]


            # --- MONKEY-PATCHING eval() to enable MC-Dropout ---
            # We suspect predict_action_chunk() is internally calling self.policy.eval(),
            # which disables dropout. To get uncertainty, we must force dropout to be active.
            # We do this by temporarily replacing the eval() method with a placeholder.
            original_eval = self.policy.eval
            self.policy.eval = lambda: self.policy  # Make eval do nothing
            
            action_batch_list = []

            def enable_dropout(m):
                if type(m) == torch.nn.Dropout:
                    m.train()

            self.policy.eval()
            # Dropout 레이어만 train 모드 (활성화)
            self.policy.apply(enable_dropout)

            original_eval = self.policy.eval
            self.policy.eval = lambda: None

            try:
                # Set the policy to training mode to activate dropout layers.
                batched_state = {}

                # Loop to perform multiple forward passes with different dropout masks.
                for key, value in state.items():
                    if isinstance(value, torch.Tensor):
                        # (1, ...) -> (Batch_Size, ...)
                        repeats = [1] * value.dim()
                        repeats[0] = self.action_batch_size
                        batched_state[key] = value.repeat(*repeats)
                    elif isinstance(value, list):
                        batched_state[key] = value * self.action_batch_size
                    else:
                        batched_state[key] = value

                with torch.no_grad():
                    # 한 번의 호출로 32개의 서로 다른 Dropout 결과 획득
                    action_batch = self.policy.predict_action_chunk(batched_state)

            finally:
                # CRITICAL: Always restore the original eval method and set mode back to eval.
                self.policy.eval = original_eval
                self.policy.eval() # 다시 완전한 평가 모드로

            # For debugging, let's check the standard deviation of the actions.
            # A non-zero value indicates that dropout is working as expected.
            action_std = torch.std(action_batch, dim=0).mean().item()
            # print(f"Action standard deviation: {action_std:.6f}")

            action_list.append(action_batch.cpu().numpy())


        return action_list

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
        for rollout in rollouts:
            all_embeddings.append(self.get_embedding(rollout))
        return all_embeddings



