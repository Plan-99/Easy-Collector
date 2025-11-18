import time
from flask import current_app, request
from ...env.agent import Agent
import json


def subscribe_robot_topic(agent: Agent, node, socketio_instance, task_control):
    while not task_control['stop']:
        js = agent.get_joint_states()
        ja = agent.get_joint_actions()
        ep = agent.get_ee_position()
        et = agent.get_ee_target()

        socketio_instance.emit('robot_status_' + str(agent.id), {
            # .tolist()를 호출하여 파이썬 list로 변환
            'joint_states': js.tolist() if js is not None else None,
            'joint_actions': ja.tolist() if ja is not None else None,
            'ee_pos': ep if ep is not None else None,
            'ee_target': et if et is not None else None
        })
        time.sleep(0.1)