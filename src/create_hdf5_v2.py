
import h5py
import numpy as np
import os

output_dir = '/root/src/backend/datasets/8'
if not os.path.exists(output_dir):
    os.makedirs(output_dir)

output_path = os.path.join(output_dir, 'episode_2.hdf5')

max_timesteps = 100
image_height = 150
image_width = 200
joint_len = 7
robot_id = 1
sensor_id = 1

with h5py.File(output_path, 'w') as root:
    root.attrs['sim'] = False

    obs = root.create_group('observations')
    images = obs.create_group('images')
    qpos = obs.create_group('qpos')
    qaction = root.create_group('qaction')

    # Create datasets
    black_image = np.zeros((image_height, image_width, 3), dtype=np.uint8)
    image_dataset = images.create_dataset(f"sensor_1", (max_timesteps, image_height, image_width, 3), dtype='uint8')
    image_dataset = images.create_dataset(f"sensor_2", (max_timesteps, image_height, image_width, 3), dtype='uint8')
    
    qpos_dataset = qpos.create_dataset(f'robot_{robot_id}', (max_timesteps, joint_len), dtype=np.float32)
    qaction_dataset = qaction.create_dataset(f'robot_{robot_id}', (max_timesteps, joint_len), dtype=np.float32)

    # Fill datasets with data
    for t in range(max_timesteps):
        image_dataset[t] = black_image
        qpos_dataset[t] = np.full(joint_len, t * 100)
        qaction_dataset[t] = np.full(joint_len, (t * 100) + 100)

print(f"HDF5 file created at {output_path} with the correct structure.")
