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

def read_hdf5(node, hdf5_path, socketio_instance, sid, task_control, move_robot=False, sensors=None, agents=None, task=None, action_key='qaction'):

    global config
    config = {}
    if move_robot:
        from ...env.env import Env
        env = Env(node, agents, sensors)

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


            # 타임스텝별로 데이터 전송
            for i in range(len(qaction_data[robot_names[0]])):

                if task_control['stop']:
                    print("Stopping read_hdf5 process.")
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
                                if action_key == 'ee_delta_action' and agent.ik_solver is not None:
                                    ee_name = agent.ee_names[0]
                                    agent.move_ee_delta_step({ee_name: qaction_array.tolist()})
                                else:
                                    agent.move_joint_step(qaction_array)


                time.sleep(0.1)
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


def add_config(config_data):
    global config
    config.update(config_data)
    