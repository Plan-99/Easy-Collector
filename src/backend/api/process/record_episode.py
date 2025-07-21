from tqdm import tqdm
import os
import numpy as np
from ...env.env import Env
from ...configs.global_configs import DATASET_DIR
from .leader_teleoperation import Leader
from ...utils.image_parser import fetch_image_with_config
import time
import h5py

def get_auto_index(dataset_dir, dataset_name_prefix = '', data_suffix = 'hdf5'):
    max_idx = 1000
    if not os.path.isdir(dataset_dir):
        os.makedirs(dataset_dir)
    for i in range(max_idx+1):
        if not os.path.isfile(os.path.join(dataset_dir, f'{dataset_name_prefix}episode_{i}.{data_suffix}')):
            return i
    raise Exception(f"Error getting auto index, or more than {max_idx} episodes")

def record_episode(dataset_id, robots, sensors, task, socketio_instance, task_control, tele_type='leader'):
    env = Env(robots=robots, sensors=sensors)
    dataset_dir = f"{DATASET_DIR}/{dataset_id}"

    while not task_control['stop']:

        tele_control = {
            'stop': task_control['stop'],
        }

        dataset_name = f"episode_{get_auto_index(dataset_dir)}.hdf5"

        socketio_instance.emit('log_record_episode', {
            'log': f'Recording Data: {dataset_name}',
            'type': 'stdout '
        })

        max_timesteps = task['episode_len']
        # camera_names = task_config['camera_names']
        # camera_config = task_config['camera_config']
        home_pose = task['home_pose']
        end_pose = task['end_pose']

        socketio_instance.emit('record_episode_progress', {
            'progress': 0,
            'type': 'stdout '
        })
        for agent in env.agents:
            agent.move_to(home_pose[str(agent.id)])
            if tele_type == 'leader':
                leader = Leader(agent, socketio_instance, log_emit_id='record_episode')
                leader.sync_leader_robot()
                socketio_instance.start_background_task(
                    target=leader.position_pub,
                    task_control=tele_control
                )

        # # saving dataset
        if not os.path.isdir(dataset_dir):
            os.makedirs(dataset_dir)
        dataset_path = os.path.join(dataset_dir, dataset_name)
        if os.path.isfile(dataset_path):
            print(f'Dataset already exist at \n{dataset_path}\nHint: set overwrite to True.')

        ts = env.reset()
        timesteps = [ts]
        for t in range(max_timesteps):
            socketio_instance.emit('record_episode_progress', {
                'progress': (t+1) / max_timesteps,
                'type': 'stdout '
            })
            if task_control['stop']:
                socketio_instance.emit('log_record_episode', {
                    'log': f'Stopping Data Collection',
                    'type': 'stdout '
                })
                tele_control['stop'] = True
                return

            ts = env.record_step()
            timesteps.append(ts)
            time.sleep(0.1)

        socketio_instance.emit('log_record_episode', {
            'log': f'Saving Data: {dataset_name}',
            'type': 'stdout '
        })

        data_dict = {}
        for agent in env.agents:
            data_dict[f'/observations/qpos/robot_{agent.id}'] = []
            data_dict[f'/qaction/robot_{agent.id}'] = []
        for sensor in sensors:
            data_dict[f'/observations/images/sensor_{sensor["id"]}'] = []

        timesteps.pop(len(timesteps) - 1)

        step = 0
        while timesteps:
            ts = timesteps.pop(0)

            for agent in env.agents:
                data_dict[f'/observations/qpos/robot_{agent.id}'].append(ts.observation['robot_states'][agent.id]['qpos'])
                data_dict[f'/qaction/robot_{agent.id}'].append(ts.observation['robot_states'][agent.id]['qaction'])
            
            for sensor in sensors:
                image = ts.observation['images'][sensor['id']]

                if image is not None:
                    image = fetch_image_with_config(image, {
                        'resize': task['sensor_img_size'],
                    })
                    data_dict[f'/observations/images/sensor_{sensor["id"]}'].append(image)
                else:
                    print("error")
                    
            step += 1

        
        # HDF5
        t0 = time.time()
        image_size = (task['sensor_img_size'][1], task['sensor_img_size'][0])
        with h5py.File(dataset_path, 'w', rdcc_nbytes=1024**2*2) as root:
            root.attrs['sim'] = False
            obs = root.create_group('observations')
            image = obs.create_group('images')
            qpos = obs.create_group('qpos')
            qaction = root.create_group('qaction')
            for sensor in sensors:
                _ = image.create_dataset(f"sensor_{sensor['id']}", (max_timesteps, image_size[0], image_size[1], 3), dtype='uint8',
                                        chunks=(1, image_size[0], image_size[1], 3), )

            for agent in env.agents:
                _ = qpos.create_dataset(f'robot_{agent.id}', (max_timesteps, agent.joint_len))
                _ = qaction.create_dataset(f'robot_{agent.id}', (max_timesteps, agent.joint_len))

            for name, array in data_dict.items():
                root[name][...] = array

        socketio_instance.emit('log_record_episode', {
            'log': f'Saved Data: {dataset_name} in {time.time() - t0:.2f} seconds',
            'type': 'stdout '
        })
        tele_control['stop'] = True

        time.sleep(5)

    

    socketio_instance.emit('log_record_episode', {
        'log': f'Stopping Data Collection',
        'type': 'stdout '
    })

# def get_auto_index(dataset_dir, dataset_name_prefix = '', data_suffix = 'hdf5'):
#     max_idx = 1000
#     if not os.path.isdir(dataset_dir):
#         os.makedirs(dataset_dir)
#     for i in range(max_idx+1):
#         if not os.path.isfile(os.path.join(dataset_dir, f'{dataset_name_prefix}episode_{i}.{data_suffix}')):
#             return i
#     raise Exception(f"Error getting auto index, or more than {max_idx} episodes")

# def main(args):
#     capture_one_episode(robot, )