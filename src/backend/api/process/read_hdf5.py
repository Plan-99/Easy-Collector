import h5py
import numpy as np

import time
import argparse
import os
import cv2
import base64 # 이미지 인코딩을 위해 추가


def read_hdf5(hdf5_path, socketio_instance, task_control):
    while True:
        image_data = {}
        qpos_data = {}
        qaction_data = {}

        with h5py.File(hdf5_path, 'r') as f:
            # actions = f[f"action"][:]
            # xactions = f[f"xaction"][:]
            # xvel_actions = f[f"xvel_action"][:]
            # xpos_data = f["observations/xpos"][:]
            # xvel_data = f["observations/xvel"][:]
            sensor_names = [name for name in f["observations/images"].keys()]
            robot_names = [name for name in f["observations/qpos"].keys()]

            for name in sensor_names:
                image_data[name] = f[f"observations/images/{name}"][:]

            for name in robot_names:
                qpos_data[name] = f[f"observations/qpos/{name}"][:]
                qaction_data[name] = f[f"qaction/{name}"][:]

            # 타임스텝별로 데이터 전송
            for i in range(len(qaction_data[robot_names[0]])):

                if task_control['stop']:
                    print("Stopping read_hdf5 process.")
                    return
                
                # --- 이미지를 Base64 문자열로 인코딩 ---
                encoded_images = {}
                for cam_name in sensor_names:
                    img_array = image_data[cam_name][i]
                    
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

                time.sleep(0.1)
                socketio_instance.emit('show_episode_step', {
                    'hdf5_path': hdf5_path,
                    'images': encoded_images,
                    'robot_states': robot_states,
                    # 'xaction': xactions[i].tolist(),
                    # 'xvel_action': xvel_actions[i].tolist(),
                    # 'xpos': xpos_data[i].tolist(),
                    # 'xvel': xvel_data[i].tolist()
                })