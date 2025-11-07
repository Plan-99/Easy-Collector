import time
from flask import current_app, request
from ...env.agent import Agent


def subscribe_robot_topic(robot, node, socketio_instance, task_control):
    agent = Agent(node, robot)
    while not task_control['stop']:
        js = agent.joint_states
        ja = agent.joint_actions

        # print(js) # 디버깅용으로 유지 가능

        socketio_instance.emit('robot_status_' + str(robot['id']), {
            # .tolist()를 호출하여 파이썬 list로 변환
            'joint_states': js.tolist() if js is not None else None,
            'joint_actions': ja.tolist() if ja is not None else None,
        })
        time.sleep(0.1)