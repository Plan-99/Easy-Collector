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
from ...database.models.teleoperator_model import Teleoperator as TeleoperatorModel

def get_auto_index(dataset_dir, dataset_name_prefix = '', data_suffix = 'hdf5'):
    max_idx = 1000
    if not os.path.isdir(dataset_dir):
        os.makedirs(dataset_dir)
    for i in range(max_idx+1):
        if not os.path.isfile(os.path.join(dataset_dir, f'{dataset_name_prefix}episode_{i}.{data_suffix}')):
            return i
    raise Exception(f"Error getting auto index, or more than {max_idx} episodes")

def record_episode(node, dataset_id, agents, assembly_id, sensors, task, language_instruction, socketio_instance, task_control, tele_type='leader', iter=100000):
    env = Env(node, agents=agents, sensors=sensors)
    dataset_dir = f"{DATASET_DIR}/{dataset_id}"

    for i in range(iter):

        if task_control['stop']:
            break

        tele_control = {
            'stop': task_control['stop'],
        }

        dataset_name = f"episode_{get_auto_index(dataset_dir)}.hdf5"

        print(f"Recording Data: {dataset_name}")

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
            for agent in agents:
                agent.move_lock = True
                agent.move_to(home_pose[str(agent.id)])
                if agent.ik_solver is not None:
                    agent.reset_ik_solver(home_pose[str(agent.id)])

        # Reset the environment to get the first timestep at the home_pose
        time.sleep(1)

        if tele_type == 'leader':
            teleop = TeleoperatorModel.where('type', 'leader').where('assembly_id', assembly_id).first()

            if teleop is None:
                print(f'[ERROR]: No leader robot preset for assembly {assembly_id}')
                tele_control['stop'] = True
                return

            leader = Leader(node, agents, socketio_instance, teleop.settings)

            socketio_instance.start_background_task(
                target=leader.leader_teleop_workflow,
                task_control=tele_control
            )

            while not leader.is_synced:
                time.sleep(0.1)            


        # # saving dataset
        if not os.path.isdir(dataset_dir):
            os.makedirs(dataset_dir)
        dataset_path = os.path.join(dataset_dir, dataset_name)
        if os.path.isfile(dataset_path):
            print(f'Dataset already exist at \n{dataset_path}\nHint: set overwrite to True.')

        for agent in agents:
            if agent.joint_states is None:
                print(f'[ERROR] No joint states from robot {agent.id}')
                tele_control['stop'] = True
                return
            if agent.joint_actions is None:
                print(f'[ERROR] No joint commands from robot {agent.id}')
                tele_control['stop'] = True
                return
            agent.move_lock = False
            
        for sensor in sensors:
            if getattr(env, f'sensor_{sensor["id"]}') is None:
                print(f'[ERROR] No data from sensor {sensor["id"]}')
                tele_control['stop'] = True
                return
            
        ts = env.reset()
        timesteps = [ts]

        for t in range(max_timesteps):
            
            if tele_type == 'keyboard':
                while not any(agent.moved_by_ui for agent in agents):
                    if task_control['stop']:
                        print('Stopping episode recording as requested.')
                        tele_control['stop'] = True
                        return
                    time.sleep(0.1)

            socketio_instance.emit('record_episode_progress', {
                'progress': (t+1) / max_timesteps,
            })
            if task_control['stop']:
                print('Stopping episode recording as requested.')
                tele_control['stop'] = True
                return

            ts = env.record_step()
            timesteps.append(ts)
            time.sleep(0.1)
            
            if tele_type == 'keyboard':
                for agent in agents:
                    agent.moved_by_ui = False

        print(f'Saving Data: {dataset_name}')

        # --- 데이터 재구성 및 HDF5 쓰기 ---
        with h5py.File(dataset_path, 'w', rdcc_nbytes=1024**2*2) as root:
            root.attrs['sim'] = False
            obs_group = root.create_group('observations')
            
            # 1. 이미지 저장 (observations/images/...)
            image_group = obs_group.create_group('images')
            for sensor in sensors:
                s_id = sensor['id']
                img_data = []
                for t_step in timesteps:
                    img = t_step.observation['images'][f'sensor_{s_id}']
                    img_data.append(fetch_image_with_config(img, {'resize': task['sensor_img_size']}))
                image_group.create_dataset(f"sensor_{s_id}", data=np.array(img_data), dtype='uint8')

            # 2. 로봇 데이터 저장 (성격에 따라 경로 분기)
            # - Observation 성격: qpos, eepos
            # - Target/Delta 성격: qaction, eetarget, qaction_delta, eetarget_delta
            obs_keys = ['qpos', 'eepos'] 

            for agent in agents:
                a_id = agent.id
                # 첫 번째 샘플로 데이터 존재 여부 및 구조 확인
                sample_robot_state = timesteps[0].observation['robot_states'][a_id]

                for key, value in sample_robot_state.items():
                    if value is None:
                        continue
                    
                    # 저장될 상위 그룹 결정
                    parent_group = obs_group if key in obs_keys else root
                    
                    # 해당 키의 그룹 생성 또는 가져오기 (예: /observations/eepos 또는 /eetarget_delta)
                    data_group = parent_group.require_group(key)
                    
                    # 딕셔너리 형태 (EE 이름별 데이터) 처리
                    if isinstance(value, dict):
                        agent_group = data_group.require_group(f'robot_{a_id}')
                        for ee_name in value.keys():
                            series_data = []
                            for t_step in timesteps:
                                series_data.append(t_step.observation['robot_states'][a_id][key][ee_name])
                            agent_group.create_dataset(ee_name, data=np.array(series_data))
                    
                    # 일반 리스트/어레이 형태 처리
                    else:
                        series_data = []
                        for t_step in timesteps:
                            series_data.append(t_step.observation['robot_states'][a_id][key])
                        agent_group = data_group # 단일 조인트 데이터는 그룹 바로 아래 생성
                        agent_group.create_dataset(f'robot_{a_id}', data=np.array(series_data))

            # 3. 기타 메타데이터
            root.create_dataset('language_instruction', data=language_instruction if language_instruction else '', 
                               dtype=h5py.string_dtype(encoding='utf-8'))
            
            time.sleep(1)  # Ensure all data is written before emitting event

        # data_dict = {}
        # for agent in agents:
        #     data_dict[f'/observations/qpos/robot_{agent.id}'] = []
        #     data_dict[f'/qaction/robot_{agent.id}'] = []
        # for sensor in sensors:
        #     data_dict[f'/observations/images/sensor_{sensor["id"]}'] = []

        # data_dict[f'/language_instruction'] = language_instruction if language_instruction is not None else ''

        # timesteps.pop(len(timesteps) - 1)

        # step = 0
        # while timesteps:
        #     ts = timesteps.pop(0)

        #     for agent in agents:
        #         data_dict[f'/observations/qpos/robot_{agent.id}'].append(ts.observation['robot_states'][agent.id]['qpos'])
        #         data_dict[f'/qaction/robot_{agent.id}'].append(ts.observation['robot_states'][agent.id]['qaction'])
        #         # print(np.array(ts.observation['robot_states'][agent.id]['qaction']).shape, np.array(ts.observation['robot_states'][agent.id]['qpos']).shape, agent.id)
            
        #     for sensor in sensors:
        #         image = ts.observation['images']['sensor_' + str(sensor['id'])]

        #         if image is not None:
        #             image = fetch_image_with_config(image, {
        #                 'resize': task['sensor_img_size'],
        #             })
        #             data_dict[f'/observations/images/sensor_{sensor["id"]}'].append(image)
        #         else:
        #             print("error")
                    
        #     step += 1

        
        # # HDF5
        # t0 = time.time()
        # image_size = (task['sensor_img_size'][1], task['sensor_img_size'][0])
        # print("image size", image_size)
        # with h5py.File(dataset_path, 'w', rdcc_nbytes=1024**2*2) as root:
        #     root.attrs['sim'] = False
        #     obs = root.create_group('observations')
        #     image = obs.create_group('images')
        #     qpos = obs.create_group('qpos')
        #     qaction = root.create_group('qaction')
        #     root.create_dataset('/language_instruction', shape=(), dtype=h5py.string_dtype(encoding='utf-8'))

        #     for sensor in sensors:
        #         _ = image.create_dataset(f"sensor_{sensor['id']}", (max_timesteps, image_size[0], image_size[1], 3), dtype='uint8',
        #                                 chunks=(1, image_size[0], image_size[1], 3), )

        #     for agent in agents:
        #         _ = qpos.create_dataset(f'robot_{agent.id}', (max_timesteps, agent.joint_len))
        #         _ = qaction.create_dataset(f'robot_{agent.id}', (max_timesteps, agent.joint_len))

        #     for name, array in data_dict.items():
        #         root[name][...] = array

        # socketio_instance.emit('episode_added', {
        #     'name': dataset_name,
        # })
            
        # print(f"Data saved at {dataset_path}, time taken: {time.time() - t0:.4f} sec")

        # tele_control['stop'] = True

        # time.sleep(5)

    print("Episode recording process ended.")

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