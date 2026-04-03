import h5py
import numpy as np

import time
import argparse
import os
import cv2
import base64 # 이미지 인코딩을 위해 추가
from .augment_dataset import adjust_lightness, draw_rectangles, add_salt_and_pepper_noise, add_gaussian_noise, generate_rect_params, prospective_transform, generate_prospective_transform, apply_hsv
from PIL import Image


config = {}


def read_hdf5(node, hdf5_path, socketio_instance, sid, task_control, move_robot=False, sensors=None, agents=None, task=None, hz=5, capture_dataset_id=None, action_key='qaction'):

    global config
    config = {}
    capture_env = None
    if move_robot:
        from ...env.env import Env
        env = Env(node, agents, sensors)
        if capture_dataset_id is not None:
            capture_env = env

    hz = 20

    while True:
        image_data = {}
        qpos_data = {}
        qaction_data = {}
        if move_robot:
            home_pose = task.get('home_pose', None)

            if home_pose is not None:
                for agent in agents:
                    if str(agent.id) in home_pose:
                        agent.move_to(home_pose[str(agent.id)], duration=5.0)

            time.sleep(5)

            # ee_delta replay를 위해 IK solver 상태를 현재 조인트에 동기화
            if action_key == 'ee_delta_action':
                for agent in agents:
                    if agent.ik_solver is not None:
                        js = agent.get_joint_states()
                        if js is not None:
                            agent.reset_ik_solver(js)

        with h5py.File(hdf5_path, 'r') as f:
            rect_params = []
            # actions = f[f"action"][:]
            # xactions = f[f"xaction"][:]
            # xvel_actions = f[f"xvel_actions"][:]
            # xpos_data = f["observations/xpos"][:]
            # xvel_data = f["observations/xvel"][:]
            sensor_names = [name for name in f["observations/images"].keys()]
            robot_names = [name for name in f["observations/qpos"].keys()] 

            for name in sensor_names:
                image_data[name] = f[f"observations/images/{name}"][:]

            for i, name in enumerate(robot_names):
                qpos_data[name] = f[f"observations/qpos/{name}"][:]
                if action_key == 'ee_delta_action' and 'ee_delta_action' in f and name in f['ee_delta_action']:
                    # ee_delta_action/robot_N/ee_name → 첫 번째 ee_name의 데이터 사용
                    ee_group = f[f'ee_delta_action/{name}']
                    first_ee = list(ee_group.keys())[0]
                    qaction_data[name] = ee_group[first_ee][:]
                else:
                    qaction_data[name] = f[f"qaction/{name}"][:]

            if "language_instruction" in f:
                language_instruction = f["language_instruction"][()].decode('utf-8')
            else:
                language_instruction = ""


            capture_timesteps = []
            if capture_env is not None:
                ts = capture_env.reset()
                capture_timesteps.append(ts)

            total_steps = len(qaction_data[robot_names[0]])
            if capture_env is not None and task and task.get('episode_len'):
                total_steps = min(total_steps, task['episode_len'])

            # 타임스텝별로 데이터 전송
            for i in range(total_steps):

                if task_control['stop']:
                    print("Stopping read_hdf5 process. Captured data discarded.")
                    return
                
                # --- 이미지를 Base64 문자열로 인코딩 ---
                encoded_images = {}

                for cam_name in sensor_names:
                    img_array = image_data[cam_name][i]

                    img = Image.fromarray(img_array)
                    if 'lightness' in config:
                        img = adjust_lightness(img, config['lightness'])
                    if 'rectangles' in config:
                        if len(rect_params) != config['rectangles'].get('count', 0):
                            rect_params = generate_rect_params(config['rectangles'], img.width, img.height)
                        img = draw_rectangles(img, rect_params)
                    if 'saltAndPepper' in config:
                        img = add_salt_and_pepper_noise(img, config['saltAndPepper'].get('amount', 0))
                    if 'gaussian' in config:
                        img = add_gaussian_noise(img, config['gaussian'].get('mean', 0), config['gaussian'].get('sigma', 0))
                    if 'prospective' in config:
                        transform_matrix = generate_prospective_transform(img.width, img.height,
                                                                          config['prospective'].get('scale_factor', 0),
                                                                          config['prospective'].get('degrees', 0),
                                                                          config['prospective'].get('shear', 0),
                                                                          config['prospective'].get('perspective', 0))
                        img = prospective_transform(img, transform_matrix)
                    
                    if 'hsv' in config and config['hsv']:
                        hsv_config = config['hsv']
                        if hsv_config.get('random'):
                            # For preview, use fixed "random" values
                            h_gain = 0.5
                            s_gain = 0.7
                            v_gain = 0.4
                            h_adj = (np.random.rand() * 2 - 1) * h_gain * 180
                            s_adj = (np.random.rand() * 2 - 1) * s_gain + 1
                            v_adj = (np.random.rand() * 2 - 1) * v_gain + 1
                        else:
                            # Use exact slider values
                            h_adj = hsv_config.get('h', 0) * 180
                            s_adj = 1 + hsv_config.get('s', 0)
                            v_adj = 1 + hsv_config.get('v', 0)
                        
                        img = apply_hsv(img, h_adj, s_adj, v_adj)

                    img_array = np.array(img)
                    
                    # 이미지를 JPEG 형식으로 메모리 버퍼에 인코딩
                    success, buffer = cv2.imencode('.jpg', img_array)
                    if not success:
                        continue
                    
                    # 버퍼의 바이너리 데이터를 Base64 문자열로 변환
                    jpg_as_text = base64.b64encode(buffer).decode('utf-8')
                    
                    # 데이터 URI 스킴을 붙여서 클라이언트 <img> 태그에서 바로 사용 가능하게 만듦
                    encoded_images[cam_name] = f"data:image/jpeg;base64,{jpg_as_text}"
                # ------------------------------------

                robot_states = {}

                for robot_name in robot_names:
                    qpos_array = qpos_data[robot_name][i]
                    qaction_array = qaction_data[robot_name][i]
                    robot_states[robot_name] = {
                        'qpos': qpos_array.tolist(),
                        'qaction': qaction_array.tolist(),
                    }
                    if move_robot:
                        for agent in agents:
                            if str(agent.id) == robot_name.replace("robot_", ""):
                                if action_key == 'ee_delta_action' and agent.role != 'tool' and agent.ik_solver is not None:
                                    ee_name = agent.ee_names[0]
                                    action_list = qaction_array.tolist()
                                    ee_delta = {ee_name: action_list[:6]}
                                    tool_positions = action_list[6:] if agent.tool_inner and len(action_list) > 6 else None
                                    agent.move_ee_delta_step(ee_delta, tool_positions=tool_positions)
                                else:
                                    agent.move_joint_step(qaction_array)

                if capture_env is not None:
                    ts = capture_env.record_step()
                    capture_timesteps.append(ts)
                    socketio_instance.emit('replay_capture_progress', {
                        'progress': (i + 1) / total_steps,
                    })

                time.sleep(1.0 / hz if hz > 0 else 0.2)
                socketio_instance.emit('show_episode_step', {
                    'hdf5_path': hdf5_path,
                    'images': encoded_images,
                    'robot_states': robot_states,
                    'language_instruction': language_instruction,
                    # 'xaction': xactions[i].tolist(),
                    # 'xvel_action': xvel_actions[i].tolist(),
                    # 'xpos': xpos_data[i].tolist(),
                    # 'xvel': xvel_data[i].tolist()
                }, to=sid)

                time.sleep(1.0 / hz)


            # Save captured episode to target dataset
            if capture_env is not None and capture_timesteps and capture_dataset_id is not None:
                _save_captured_episode(capture_dataset_id, capture_timesteps, agents, sensors, task, language_instruction)


def _save_captured_episode(dataset_id, timesteps, agents, sensors, task, language_instruction):
    from ...configs.global_configs import DATASET_DIR
    from .record_episode import get_auto_index
    from ...utils.image_parser import fetch_image_with_config

    dataset_dir = os.path.join(DATASET_DIR, str(dataset_id))
    if not os.path.isdir(dataset_dir):
        os.makedirs(dataset_dir)

    dataset_name = f"episode_{get_auto_index(dataset_dir)}.hdf5"
    dataset_path = os.path.join(dataset_dir, dataset_name)
    print(f"[Capture] Saving captured episode: {dataset_path}")

    try:
        with h5py.File(dataset_path, 'w', rdcc_nbytes=1024**2*2) as root:
            root.attrs['sim'] = False
            obs_group = root.create_group('observations')

            image_group = obs_group.create_group('images')
            for sensor in sensors:
                s_id = str(sensor['id'])
                img_data = []
                for t_step in timesteps:
                    img = t_step.observation['images'][f'sensor_{s_id}']
                    img_data.append(fetch_image_with_config(img, {
                        'resize': task.get('sensor_img_size', {}).get(s_id),
                        'cropped_area': task.get('sensor_cropped_area', {}).get(s_id, {}),
                        'rotate': task.get('sensor_rotate', {}).get(s_id, 0)
                    }))
                image_group.create_dataset(f"sensor_{s_id}", data=np.array(img_data), dtype='uint8')

            obs_keys = ['qpos', 'eepos']
            for agent in agents:
                a_id = agent.id
                sample_robot_state = timesteps[0].observation['robot_states'][a_id]
                for key, value in sample_robot_state.items():
                    if value is None:
                        continue
                    parent_group = obs_group if key in obs_keys else root
                    data_group = parent_group.require_group(key)
                    if isinstance(value, dict):
                        agent_group = data_group.require_group(f'robot_{a_id}')
                        for ee_name in value.keys():
                            series_data = [t_step.observation['robot_states'][a_id][key][ee_name] for t_step in timesteps]
                            agent_group.create_dataset(ee_name, data=np.array(series_data))
                    else:
                        series_data = [t_step.observation['robot_states'][a_id][key] for t_step in timesteps]
                        data_group.create_dataset(f'robot_{a_id}', data=np.array(series_data))

            root.create_dataset('language_instruction',
                                data=language_instruction if language_instruction else '',
                                dtype=h5py.string_dtype(encoding='utf-8'))

        print(f"[Capture] Episode saved: {dataset_name} ({len(timesteps)} timesteps)")
    except Exception as e:
        import traceback
        print(f"[Capture ERROR] Failed to save captured episode:\n{traceback.format_exc()}")


def add_config(config_data):
    global config
    config.update(config_data)
    