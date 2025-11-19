from tqdm import tqdm
import os
import numpy as np
from ...env.env import Env
from ...env.agent import Agent
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

def record_episode(node, dataset_id, robots, sensors, task, language_instruction, socketio_instance, task_control, tele_type='leader', iter=100000):
    agents = []
    for robot in robots:
        agents.append(Agent(node, robot))
    env = Env(node, agents=agents, sensors=sensors)
    dataset_dir = f"{DATASET_DIR}/{dataset_id}"

    for i in range(iter):

        if task_control['stop']:
            break

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
        if home_pose is not None and tele_type != 'externel':
            for agent in env.agents:
                agent.move_to(home_pose[str(agent.id)])
            time.sleep(3)

        # Reset the environment to get the first timestep at the home_pose
        ts = env.reset()
        timesteps = [ts]
        time.sleep(1)

        if tele_type == 'leader':
            leaders = []
            for agent in env.agents:
                leader = Leader(agent, socketio_instance, agent.leader_robot_preset, log_emit_id='record_episode', port=agent.leader_robot_preset['port_name'])
                socketio_instance.start_background_task(
                    target=leader.sync_leader_robot,
                )
                leaders.append(leader)

            while not all([leader.is_synced for leader in leaders]):
                time.sleep(0.1)

            for leader in leaders:
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

        for agent in env.agents:
            if agent.joint_states is None:
                socketio_instance.emit('log_record_episode', {
                    'log': f'[ERROR]: No joint states from robot {agent.id}',
                    'type': 'stderr'
                })
                tele_control['stop'] = True
                return
            if agent.joint_actions is None:
                socketio_instance.emit('log_record_episode', {
                    'log': f'[ERROR]: No joint commands from robot {agent.id}',
                    'type': 'stderr'
                })
                tele_control['stop'] = True
                return
            
        for sensor in sensors:
            if getattr(env, f'sensor_{sensor["id"]}') is None:
                socketio_instance.emit('log_record_episode', {
                    'log': f'[ERROR]: No image data from sensor {sensor["id"]}',
                    'type': 'stderr'
                })
                tele_control['stop'] = True
                return
            
        for t in range(max_timesteps):
            socketio_instance.emit('record_episode_progress', {
                'progress': (t+1) / max_timesteps,
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

        data_dict[f'/language_instruction'] = language_instruction if language_instruction is not None else ''

        timesteps.pop(len(timesteps) - 1)

        step = 0
        while timesteps:
            ts = timesteps.pop(0)

            for agent in env.agents:
                data_dict[f'/observations/qpos/robot_{agent.id}'].append(ts.observation['robot_states'][agent.id]['qpos'])
                data_dict[f'/qaction/robot_{agent.id}'].append(ts.observation['robot_states'][agent.id]['qaction'])
                # print(np.array(ts.observation['robot_states'][agent.id]['qaction']).shape, np.array(ts.observation['robot_states'][agent.id]['qpos']).shape, agent.id)
            
            for sensor in sensors:
                image = ts.observation['images']['sensor_' + str(sensor['id'])]

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
        print("image size", image_size)
        with h5py.File(dataset_path, 'w', rdcc_nbytes=1024**2*2) as root:
            root.attrs['sim'] = False
            obs = root.create_group('observations')
            image = obs.create_group('images')
            qpos = obs.create_group('qpos')
            qaction = root.create_group('qaction')
            root.create_dataset('/language_instruction', shape=(), dtype=h5py.string_dtype(encoding='utf-8'))

            for sensor in sensors:
                _ = image.create_dataset(f"sensor_{sensor['id']}", (max_timesteps, image_size[0], image_size[1], 3), dtype='uint8',
                                        chunks=(1, image_size[0], image_size[1], 3), )

            for agent in env.agents:
                _ = qpos.create_dataset(f'robot_{agent.id}', (max_timesteps, agent.joint_len))
                _ = qaction.create_dataset(f'robot_{agent.id}', (max_timesteps, agent.joint_len))

            for name, array in data_dict.items():
                root[name][...] = array

        socketio_instance.emit('episode_added', {
            'name': dataset_name,
        })
            
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