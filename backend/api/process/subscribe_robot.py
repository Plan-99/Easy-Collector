import time
from flask import current_app, request
from ...bridge.remote_agent import RemoteAgent as Agent
import json
import traceback


def subscribe_robot_topic(agent: Agent, socketio_instance, task_control):
    import time as _t
    connected = False
    stale_after = 8.0  # seconds
    while not task_control['stop']:
        try:
            now = _t.time()
            last_js = agent.last_joint_update or 0
            js = agent.get_joint_states()
            ja = agent.get_joint_actions()
            ep = agent.get_ee_position()
            et = agent.get_ee_target()
            # 로컬 캐시 업데이트 (record_episode 등에서 agent.joint_states 직접 참조)
            agent.joint_states = js
            agent.joint_actions = ja
            connected = (now - last_js) <= stale_after and js is not None

            # 시뮬레이션 동기화: joint_states를 SimEngine에 전달
            if js is not None:
                try:
                    from ..routes.sim import get_sim_engine
                    sim = get_sim_engine()
                    if sim and sim.is_running:
                        positions = {i: v for i, v in enumerate(js)}
                        sim.set_joint_positions(positions)
                except Exception:
                    pass

            socketio_instance.emit('robot_status_' + str(agent.id), {
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
