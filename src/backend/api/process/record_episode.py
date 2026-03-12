from tqdm import tqdm
import os
import numpy as np
from ...env.env import Env
from ...env.agent import Agent
from ...env.vive_controller import ViveController
from ...configs.global_configs import DATASET_DIR
from .leader_teleoperation import Leader
from ...utils.image_parser import fetch_image_with_config
import time
import h5py
from scipy.spatial.transform import Rotation as R
from ...database.models.teleoperator_model import Teleoperator as TeleoperatorModel

def get_auto_index(dataset_dir, dataset_name_prefix = '', data_suffix = 'hdf5'):
    max_idx = 1000
    if not os.path.isdir(dataset_dir):
        os.makedirs(dataset_dir)
    for i in range(max_idx+1):
        if not os.path.isfile(os.path.join(dataset_dir, f'{dataset_name_prefix}episode_{i}.{data_suffix}')):
            return i
    raise Exception(f"Error getting auto index, or more than {max_idx} episodes")

def _compute_fk_delta(agent, qaction, qpos):
    """FK(commanded) - FK(actual): 로봇이 이번 스텝에서 이동해야 할 EE 변위."""
    if agent.ik_solver is None or qaction is None or qpos is None:
        return None
    arm_action, _ = agent.get_joint_and_tool_pos(qaction)
    arm_state, _ = agent.get_joint_and_tool_pos(qpos)
    if arm_action is None or arm_state is None:
        return None
    ee_action = agent.ik_solver.get_ee_position(np.array(arm_action))
    ee_state = agent.ik_solver.get_ee_position(np.array(arm_state))
    if ee_action is None or ee_state is None:
        return None
    return {
        name: [ee_action[name][i] - ee_state[name][i] for i in range(6)]
        for name in agent.ee_names
        if name in ee_action and name in ee_state
    }

def _vive_offset_delta(curr_offset, prev_offset):
    """두 vive cumulative offset의 차이를 robot frame delta로 반환."""
    delta_xyz = [curr_offset[i] - prev_offset[i] for i in range(3)]
    delta_rot = (
        R.from_rotvec(curr_offset[3:6]) * R.from_rotvec(prev_offset[3:6]).inv()
    ).as_rotvec().tolist()
    return delta_xyz + delta_rot

def record_episode(node, dataset_id, agents, move_homepose, assembly_id, sensors, task, language_instruction, socketio_instance, task_control, tele_type='leader', iter=100000):
    env = Env(node, agents=agents, sensors=sensors, virtual_agents=(tele_type == 'vive_only'))
    dataset_dir = f"{DATASET_DIR}/{dataset_id}"

    # --- Vive: collection 전체에서 1회 초기화 (에피소드마다 재시작하지 않음) ---
    # vive_external: 실물 로봇 + vive, vive_only: vive tracker만 (이미지+ee_delta_action)
    vive = None
    if tele_type in ('vive_external', 'vive_only'):
        vive = ViveController(node, socketio_instance, scale_factor=1, step_rate=20)
        if not vive.wait_for_ready(timeout=30.0):
            task_control['stop'] = True
            vive.destroy()
            return

    try:
        for _ in range(iter):

            if task_control['stop']:
                break

            task_control['episode_stop'] = False

            dataset_name = f"episode_{get_auto_index(dataset_dir)}.hdf5"

            print(f"Recording Data: {dataset_name}")

            max_timesteps = task['episode_len']
            home_pose = task['home_pose']

            socketio_instance.emit('record_episode_progress', {
                'progress': 0,
                'type': 'stdout '
            })

            if move_homepose and tele_type not in ('externel', 'vive_only'):
                for agent in agents:
                    agent.move_lock = True
                    agent.move_to(home_pose[str(agent.id)])

                    if agent.ik_solver is not None:
                        agent.reset_ik_solver(home_pose[str(agent.id)])

            # Reset the environment to get the first timestep at the home_pose
            time.sleep(7)

            if tele_type == 'leader':
                teleop = TeleoperatorModel.where('type', 'leader').where('assembly_id', assembly_id).first()

                if teleop is None:
                    print(f'[ERROR]: No leader robot preset for assembly {assembly_id}')
                    task_control['stop'] = True
                    return

                leader = Leader(node, agents, socketio_instance, teleop.settings)

                socketio_instance.start_background_task(
                    target=leader.leader_teleop_workflow,
                    task_control=task_control
                )

                while not leader.is_synced:
                    time.sleep(0.1)

            if not os.path.isdir(dataset_dir):
                os.makedirs(dataset_dir)
            dataset_path = os.path.join(dataset_dir, dataset_name)
            if os.path.isfile(dataset_path):
                print(f'Dataset already exist at \n{dataset_path}\nHint: set overwrite to True.')

            # vive_only는 실물 로봇 없이 동작하므로 joint state 검증 생략
            if tele_type != 'vive_only':
                for agent in agents:
                    if agent.joint_states is None:
                        print(f'[ERROR] No joint states from robot {agent.id}')
                        task_control['stop'] = True
                        return
                    if agent.joint_actions is None:
                        print(f'[ERROR] No joint commands from robot {agent.id}')
                        task_control['stop'] = True
                        return
                    agent.move_lock = False

            for sensor in sensors:
                if getattr(env, f'sensor_{sensor["id"]}') is None:
                    print(f'[ERROR] No data from sensor {sensor["id"]}')
                    task_control['stop'] = True
                    return

            # reset 타임스텝: ee_delta_action = zeros
            ts = env.reset()
            for agent in agents:
                if agent.ik_solver is not None:
                    ts.observation['robot_states'][agent.id]['ee_delta_action'] = {
                        name: [0.0] * 6 for name in agent.ee_names
                    }
            timesteps = [ts]

            # 에피소드 시작: vive origin 설정 및 teleop 스레드 시작
            prev_vive_offset = None
            if vive is not None:
                on_step = None
                if tele_type == 'vive_external':
                    # ee_origin 계산 및 on_step 콜백 설정 (로봇 이동 전용)
                    ee_origins = {}
                    for agent in agents:
                        if agent.role == 'single_arm' and agent.ik_solver is not None:
                            arm_js, _ = agent.get_joint_and_tool_pos(agent.get_joint_states())
                            init_q = np.array(arm_js)
                            ee_origins[agent.id] = agent.ik_solver.get_ee_position(init_q)
                            agent.ik_solver.reset_state(init_q)

                    def on_step(offset):
                        for agent in agents:
                            if agent.role == 'single_arm' and agent.ik_solver is not None:
                                ee_name = agent.ee_names[0]
                                ee_origin = ee_origins.get(agent.id, {}).get(ee_name)
                                if ee_origin is None:
                                    continue
                                target_ee = agent.ik_solver.compute_target_from_origin(ee_origin, offset)
                                agent.move_ee_step({ee_name: target_ee})

                vive.set_origin()
                vive.start_teleop(on_step=on_step)
                prev_vive_offset = vive.get_offset()  # set_origin 직후 = [0]*6

            for t in range(max_timesteps):

                if tele_type == 'keyboard':
                    while not any(agent.moved_by_ui for agent in agents):
                        if task_control['stop']:
                            print('Stopping episode recording as requested.')
                            task_control['stop'] = True
                            return
                        time.sleep(0.1)

                socketio_instance.emit('record_episode_progress', {
                    'progress': (t+1) / max_timesteps,
                })
                if task_control['stop']:
                    print('Stopping episode recording as requested.')
                    task_control['stop'] = True
                    return

                ts = env.record_step()

                # --- ee_delta_action 계산 및 타임스텝에 주입 ---
                for agent in agents:
                    if agent.ik_solver is None:
                        continue
                    a_id = agent.id
                    robot_state = ts.observation['robot_states'][a_id]

                    if tele_type == 'vive_only':
                        # vive offset 차이로 delta 계산
                        curr_offset = vive.get_offset()
                        delta = _vive_offset_delta(curr_offset, prev_vive_offset)
                        robot_state['ee_delta_action'] = {agent.ee_names[0]: delta}

                    elif tele_type == 'keyboard':
                        # 키보드 raw delta 그대로 사용
                        if agent.last_ee_delta is not None:
                            robot_state['ee_delta_action'] = agent.last_ee_delta

                    else:
                        # leader / vive_external: FK(joint_actions) - FK(joint_states)
                        delta = _compute_fk_delta(
                            agent,
                            robot_state.get('qaction'),
                            robot_state.get('qpos'),
                        )
                        if delta is not None:
                            robot_state['ee_delta_action'] = delta

                if tele_type == 'vive_only':
                    prev_vive_offset = vive.get_offset()

                timesteps.append(ts)

                time.sleep(0.1)

                if tele_type == 'keyboard':
                    for agent in agents:
                        agent.moved_by_ui = False

            # 에피소드 종료: teleop 스레드 정지 (다음 에피소드 origin 설정 전에 멈춤)
            if vive is not None:
                vive.stop_teleop()

            if tele_type == 'leader':
                task_control['episode_stop'] = True
                time.sleep(0.5)

            print(f'Saving Data: {dataset_name}')

            try:
                # --- 데이터 재구성 및 HDF5 쓰기 ---
                with h5py.File(dataset_path, 'w', rdcc_nbytes=1024**2*2) as root:
                    root.attrs['sim'] = False
                    obs_group = root.create_group('observations')

                    # 1. 이미지 저장 (observations/images/...)
                    image_group = obs_group.create_group('images')
                    for sensor in sensors:
                        s_id = str(sensor['id'])
                        img_data = []
                        for t_step in timesteps:
                            img = t_step.observation['images'][f'sensor_{s_id}']
                            img_data.append(fetch_image_with_config(img, {
                                'resize': task['sensor_img_size'][s_id],
                                'cropped_area': task['sensor_cropped_area'][s_id],
                                'rotate': task['sensor_rotate'][s_id]
                            }))
                        image_group.create_dataset(f"sensor_{s_id}", data=np.array(img_data), dtype='uint8')

                    # 2. 로봇 데이터 저장 (None인 값은 자동 스킵 — vive_only 시 전부 None)
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
                                    series_data = []
                                    for t_step in timesteps:
                                        series_data.append(t_step.observation['robot_states'][a_id][key][ee_name])
                                    agent_group.create_dataset(ee_name, data=np.array(series_data))
                            else:
                                series_data = []
                                for t_step in timesteps:
                                    series_data.append(t_step.observation['robot_states'][a_id][key])
                                agent_group = data_group
                                agent_group.create_dataset(f'robot_{a_id}', data=np.array(series_data))

                    # 3. 기타 메타데이터
                    root.create_dataset('language_instruction', data=language_instruction if language_instruction else '',
                                    dtype=h5py.string_dtype(encoding='utf-8'))

                    time.sleep(1)

                print("Episode recording process ended.")

            except Exception:
                import traceback
                print(f"[ERROR] Error during episode recording:\n{traceback.format_exc()}")
                task_control['stop'] = True

    finally:
        # 정상 종료, stop 신호, 예외 등 모든 경우에 vive 정리
        if vive is not None:
            vive.destroy()
