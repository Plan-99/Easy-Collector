import h5py
import numpy as np
import argparse
import json
import os

import time

# Legacy code - lazy imports for ROS dependencies
# from ...env.env import Env
# import rospy

import cv2

from tqdm import tqdm
from utils import sample_box_pose, sample_insertion_pose, qpos_to_xpos, xpos_to_qpos # robot functions


def replay_episode(node, task, filename, socketio_instance, task_control):
    hdf5_path = f"../datasets/{task['id']}/{filename}"
                
    env = Env(node, task['robots'], task['sensors'])

    for agent in env.agents:
        agent.move_to(task['home_pose'][str(agent.id)])

    time.sleep(3)

    with h5py.File(hdf5_path, 'r') as f:
        qpos_data = f["observations/qpos"][:]
        images = []
        for sensor_id in task['sensor_ids']:
            images.append(f[f"observations/images/sensor_{sensor_id}"])

        for i in tqdm(range(len(qpos_data))):
            qpos = qpos_data[i]

            for agent in env.agents:
                target_qpos = qpos[agent.id * agent.joint_len:(agent.id + 1) * agent.joint_len]
                start_action_id += agent.joint_len
                agent.move_joint_step(target_qpos)

            time.sleep(0.1)