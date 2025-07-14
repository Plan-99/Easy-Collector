import dataclasses
import numpy as np
import jax
import jax.numpy as jnp
import h5py

from openpi.models import model as _model
from openpi.policies import droid_policy, aloha_policy
from openpi.policies import policy_config as _policy_config
from openpi.shared import download
from openpi.training import config as _config
from openpi.training import data_loader as _data_loader



hdf5_file_path = "datasets/tmp/episode_0.hdf5"
with h5py.File(hdf5_file_path, "r") as f:

    # Use pi0_base model from S3 for VLA feature extraction
    print("Loading pi0_base model from gs...")
    checkpoint_dir = download.maybe_download("gs://openpi-assets/checkpoints/pi0_base")

    # Create a base pi0 config (without fast variant)
    config = _config.get_config("pi0_ours")  # Using base pi0 config instead of fast variant

    # Create a trained policy for VLA feature extraction
    policy = _policy_config.create_trained_policy(config, checkpoint_dir)

    print("Model loaded successfully!")
    print(f"Policy metadata: {policy.metadata}")
    
    for i in range(30):
        print(f['observations/images/camera1'][i].shape)
        # print(type(f['observations/qpos'][i]))
    
        example = {
            'observations': {
                'images': {
                    # HDF5 파일에서 'image' 데이터를 읽어옵니다. 
                    # f['observations']['image']는 데이터셋 객체이며, [0]으로 첫 번째 프레임을 선택합니다.
                    'camera1': f['observations/images/camera1'][i],
                    'camera2': f['observations/images/camera2'][i], 
                },
                # HDF5 파일에서 로봇의 'state' 데이터를 읽어옵니다.
                'qpos': f['observations/qpos'][i]
            },
            # 'action': f['action'][i],
            'prompt': "pick up the red can"
        }

        outputs, vla_feature = policy.infer(example)

        print(f"Action: {outputs['actions'].shape}")
        print(f"VLA feature: {vla_feature.shape}")
        # print(f"VLA feature dtype: {vla_feature.dtype}")

    # If you want to extract features from your own data, you can modify the example:
    # example = {
    #     'observation': {
    #         'image': your_image_array,  # Shape should be compatible with model
    #         'state': your_state_array,  # Robot state if available
    #     },
    #     'instruction': your_text_instruction  # Optional text instruction
    # }

    # Clean up memory
    del policy
    print("VLA feature extraction completed!")