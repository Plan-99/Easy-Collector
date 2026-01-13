import time
from flask import current_app, request
from ...env.agent import Agent
import json
import traceback


def subscribe_robot_topic(agent: Agent, node, socketio_instance, task_control):
    import time as _t
    connected = False
    stale_after = 3.0  # seconds
    while not task_control['stop']:
        try:
            now = _t.time()
            last_js = agent.last_joint_update or 0
            js = agent.get_joint_states()
            ja = agent.get_joint_actions()
            ep = agent.get_ee_position()
            et = agent.get_ee_target()
            connected = (now - last_js) <= stale_after and js is not None
            socketio_instance.emit('robot_status_' + str(agent.id), {
                # .tolist()를 호출하여 파이썬 list로 변환
                'joint_states': js if js is not None else None,
                'joint_actions': ja if ja is not None else None,
                'ee_pos': ep if ep is not None else None,
                'ee_target': et if et is not None else None,
                'connected': connected,
            })
            time.sleep(0.1)
        except Exception as e:
            error_msg = traceback.format_exc()
            print(f"[ERROR] subscribe_robot_topic:\n{error_msg}")
            break
