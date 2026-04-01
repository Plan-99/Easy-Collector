from tqdm import tqdm
import os
import numpy as np
from ...env.env import Env
from ...env.vive_controller import ViveController
from ...configs.global_configs import DATASET_DIR
from .leader_teleoperation import Leader
from ...utils.image_parser import fetch_image_with_config
import time
import h5py
from concurrent.futures import ThreadPoolExecutor
from ...database.models.teleoperator_model import Teleoperator as TeleoperatorModel

def get_auto_index(dataset_dir, dataset_name_prefix = '', data_suffix = 'hdf5'):
    max_idx = 1000
    if not os.path.isdir(dataset_dir):
        os.makedirs(dataset_dir)
    for i in range(max_idx+1):
        if not os.path.isfile(os.path.join(dataset_dir, f'{dataset_name_prefix}episode_{i}.{data_suffix}')):
            return i
    raise Exception(f"Error getting auto index, or more than {max_idx} episodes")



def record_episode(node, dataset_id, agents, move_homepose, assembly_id, sensors, task, language_instruction, socketio_instance, task_control, tele_type='leader', ros2_service='', iter=100000):
    agents = sorted(agents, key=lambda a: a.id)
    env = Env(node, agents=agents, sensors=sensors, virtual_agents=(tele_type == 'vive_only'))
    dataset_dir = f"{DATASET_DIR}/{dataset_id}"
    thread_pool = ThreadPoolExecutor(max_workers=len(agents))

    # --- Vive: collection 전체에서 1회 초기화 (에피소드마다 재시작하지 않음) ---
    # vive_external: 실물 로봇 + vive, vive_only: vive tracker만 (이미지+ee_delta_action)
    vive = None
    hz = 20
    if tele_type in ('vive_external', 'vive_only'):
        move_robot = (tele_type == 'vive_external')
        vive = ViveController(
            node, socketio_instance,
            agents=agents, move_robot=move_robot,
            scale_factor=2, step_rate=40,
        )
        if not vive.wait_for_ready(timeout=30.0):
            task_control['stop'] = True
            vive.destroy()
            return

    task_control['episode_complete'] = False

    # --- ROS2 service (motion_planning용) ---
    from std_srvs.srv import Trigger
    service_result = {'done': False, 'success': None, 'message': ''}

    try:
        for _ in range(iter):

            if task_control['stop']:
                break

            task_control['episode_stop'] = False
            task_control['succeed'] = False

            dataset_name = f"episode_{get_auto_index(dataset_dir)}.hdf5"

            print(f"Recording Data: {dataset_name}")

            max_timesteps = task['episode_len']
            home_pose = task['home_pose']

            socketio_instance.emit('record_episode_progress', {
                'progress': 0,
                'type': 'stdout '
            })

            if move_homepose and tele_type not in ('externel', 'vive_only'):
                socketio_instance.emit('moving_homepose', {'moving': True})
                for agent in agents:
                    if tele_type == 'vive_external' and agent.role == 'tool':
                        continue  # vive_external의 single_arm은 vive로 이동하므로 home_pose 이동 생략
                    agent.move_lock = True
                    agent.move_to(home_pose[str(agent.id)])

                    if agent.ik_solver is not None:
                        agent.reset_ik_solver(home_pose[str(agent.id)])

                # is_moving 플래그로 도달 대기 (타임아웃 30초)
                timeout = 30.0
                start_wait = time.time()
                while time.time() - start_wait < timeout:
                    if task_control['stop']:
                        socketio_instance.emit('moving_homepose', {'moving': False})
                        return
                    moving_agents = [a for a in agents if a.is_moving]
                    if not moving_agents:
                        break
                    time.sleep(0.1)
                socketio_instance.emit('moving_homepose', {'moving': False})
                time.sleep(0.5)  # 안정화 대기

            else:
                # move_homepose가 아니어도 IK solver 내부 상태를 현재 조인트에 동기화
                for agent in agents:
                    if agent.ik_solver is not None:
                        js = agent.get_joint_states()
                        if js is not None:
                            arm_js, _ = agent.get_joint_and_tool_pos(js)
                            if arm_js is not None:
                                with agent.ik_lock:
                                    agent.ik_solver.reset_state(arm_js)
                time.sleep(1)

            # --- motion_planning: ROS2 service 호출 ---
            if tele_type == 'motion_planning' and ros2_service:
                service_result = {'done': False, 'success': None, 'message': ''}
                try:
                    cli = node.create_client(Trigger, ros2_service)
                    if not cli.wait_for_service(timeout_sec=5.0):
                        print(f'[ERROR] ROS2 service "{ros2_service}" not available')
                        task_control['stop'] = True
                        return

                    def _on_service_done(future):
                        try:
                            result = future.result()
                            service_result['done'] = True
                            service_result['success'] = result.success
                            service_result['message'] = result.message
                            print(f'[NOTICE] ROS2 service response: success={result.success}, message="{result.message}"')
                        except Exception as e:
                            service_result['done'] = True
                            service_result['success'] = False
                            service_result['message'] = str(e)
                            print(f'[ERROR] ROS2 service error: {e}')

                    future = cli.call_async(Trigger.Request())
                    future.add_done_callback(_on_service_done)
                    print(f'[NOTICE] ROS2 service "{ros2_service}" called, recording in parallel...')
                except Exception as e:
                    print(f'[ERROR] Failed to call ROS2 service: {e}')
                    task_control['stop'] = True
                    return

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

            # motion_planning: joint_actions 초기화 후 대기
            if tele_type == 'motion_planning':
                for agent in agents:
                    agent.joint_actions = None

            # vive_only는 실물 로봇 없이 동작하므로 joint state 검증 생략
            if tele_type != 'vive_only':
                for agent in agents:
                    if agent.joint_states is None:
                        print(f'[ERROR] No joint states from robot {agent.id}')
                        task_control['stop'] = True
                        return

                # joint commands 대기 (motion_planning은 service 응답까지 최대 60초)
                wait_timeout = 60.0 if tele_type == 'motion_planning' else 5.0
                for agent in agents:
                    if agent.joint_actions is None:
                        print(f'[NOTICE] Waiting for joint commands from robot {agent.id}...')
                        elapsed = 0.0
                        while agent.joint_actions is None:
                            if task_control['stop']:
                                return
                            time.sleep(0.1)
                            elapsed += 0.1
                            if elapsed >= wait_timeout:
                                print(f'[ERROR] No joint commands from robot {agent.id} after {wait_timeout}s timeout')
                                task_control['stop'] = True
                                return
                        print(f'[NOTICE] Joint commands received from robot {agent.id} ({elapsed:.1f}s)')
                    agent.move_lock = False

            for sensor in sensors:
                if getattr(env, f'sensor_{sensor["id"]}') is None:
                    print(f'[ERROR] No data from sensor {sensor["id"]}')
                    task_control['stop'] = True
                    return

            # reset 타임스텝: ee_delta_action, ee_delta = zeros (tool_inner인 경우 tool joint 포함)
            ts = env.reset()
            for agent in agents:
                if agent.ik_solver is not None:
                    if agent.tool_inner:
                        qpos = ts.observation['robot_states'][agent.id].get('qpos')
                        _, tool_qpos = agent.get_joint_and_tool_pos(qpos) if qpos is not None else (None, None)
                        tool_vals = list(tool_qpos) if tool_qpos else []
                        ts.observation['robot_states'][agent.id]['ee_delta_action'] = {
                            name: [0.0] * (6 + len(tool_vals)) for name in agent.ee_names
                        }
                        ts.observation['robot_states'][agent.id]['ee_delta'] = {
                            name: [0.0] * 6 + tool_vals for name in agent.ee_names
                        }
                    else:
                        zeros = {name: [0.0] * 6 for name in agent.ee_names}
                        ts.observation['robot_states'][agent.id]['ee_delta_action'] = zeros
                        ts.observation['robot_states'][agent.id]['ee_delta'] = zeros

            timesteps = [ts]
            succeed_flags = [1.0 if task_control.get('succeed') else 0.0]

            # 에피소드 시작: vive origin 설정 및 teleop 스레드 시작
            if vive is not None:
                vive.set_origin()
                vive.start_teleop()

            for t in range(max_timesteps):

                if tele_type == 'keyboard':
                    while not any(agent.moved_by_ui for agent in agents):
                        if task_control['stop']:
                            print('Stopping episode recording as requested.')
                            task_control['stop'] = True
                            return
                        if task_control.get('episode_complete'):
                            break
                        time.sleep(0.1)

                socketio_instance.emit('record_episode_progress', {
                    'progress': (t+1) / max_timesteps,
                })
                if task_control['stop']:
                    print('Stopping episode recording as requested.')
                    task_control['stop'] = True
                    return

                if task_control['episode_complete']:
                    print(f'Episode completed early at timestep {t+1}/{max_timesteps}.')
                    break

                ts = env.record_step()

                # --- ee_delta_action 계산 및 타임스텝에 주입 ---
                for agent in agents:
                    if agent.ik_solver is None:
                        continue
                    a_id = agent.id
                    robot_state = ts.observation['robot_states'][a_id]

                    if tele_type in ('vive_external', 'vive_only'):
                        # 마지막 consume 이후 누적된 delta를 소비
                        delta = vive.consume_delta()
                        ee_delta = {agent.ee_names[0]: delta}

                    elif tele_type == 'keyboard':
                        # 키보드 raw delta 그대로 사용
                        ee_delta = agent.last_ee_delta if agent.last_ee_delta is not None else None

                    else:
                        # leader / motion_planning: FK(joint_actions) - FK(joint_states)
                        ee_delta = agent.compute_fk_delta(
                            robot_state.get('qaction'),
                            robot_state.get('qpos'),
                        )

                    if ee_delta is not None:
                        if agent.tool_inner:
                            qpos = robot_state.get('qpos')
                            qaction = robot_state.get('qaction')
                            _, tool_qpos = agent.get_joint_and_tool_pos(qpos) if qpos is not None else (None, None)
                            _, tool_qaction = agent.get_joint_and_tool_pos(qaction) if qaction is not None else (None, None)
                            robot_state['ee_delta'] = {
                                name: list(ee_delta[name]) + list(tool_qpos or [])
                                for name in ee_delta
                            }
                            robot_state['ee_delta_action'] = {
                                name: list(ee_delta[name]) + list(tool_qaction or [])
                                for name in ee_delta
                            }
                        else:
                            robot_state['ee_delta_action'] = ee_delta
                            robot_state['ee_delta'] = ee_delta

                succeed_flags.append(1.0 if task_control.get('succeed') else 0.0)
                timesteps.append(ts)

                time.sleep(1.0 / hz)

                if tele_type == 'keyboard':
                    for agent in agents:
                        agent.moved_by_ui = False

                # motion_planning: service 실패 시 중단
                if tele_type == 'motion_planning' and service_result['done'] and not service_result['success']:
                    print(f'[ERROR] ROS2 service failed at step {t+1}/{max_timesteps}: {service_result["message"]}')
                    break

            # 에피소드 종료: teleop 스레드 정지 (다음 에피소드 origin 설정 전에 멈춤)
            if vive is not None:
                vive.stop_teleop()

            if tele_type == 'leader':
                task_control['episode_stop'] = True
                time.sleep(0.5)

            # motion_planning: service 응답 대기 및 결과 확인
            if tele_type == 'motion_planning' and ros2_service:
                if not service_result['done']:
                    print(f'Recording finished, waiting for service response...')
                    timeout = 120.0
                    elapsed = 0.0
                    while not service_result['done'] and elapsed < timeout:
                        if task_control['stop']:
                            break
                        time.sleep(0.5)
                        elapsed += 0.5

                if service_result['done'] and not service_result['success']:
                    print(f'[ERROR] ROS2 service failed: {service_result["message"]}, restarting episode...')
                    continue
                elif service_result['done'] and service_result['success']:
                    print(f'[NOTICE] ROS2 service completed successfully: {service_result["message"]}')
                elif not service_result['done']:
                    print(f'[WARNING] ROS2 service did not complete within timeout, restarting episode...')
                    continue

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
                        images = [
                            fetch_image_with_config(ts.observation['images'][f'sensor_{s_id}'], {
                                'resize': task['sensor_img_size'][s_id],
                                'cropped_area': task['sensor_cropped_area'][s_id],
                                'rotate': task['sensor_rotate'][s_id]
                            }) for ts in timesteps
                        ]
                        image_group.create_dataset(f"sensor_{s_id}", data=np.array(images), dtype='uint8')

                    # 2. 로봇 데이터 저장 (None인 값은 자동 스킵 — vive_only 시 전부 None)
                    obs_keys = ['qpos', 'eepos', 'ee_delta', 'qvel', 'qeffort']

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
                                    if key == 'ee_delta_action':
                                        # action을 1스텝 shift: obs_t와 delta(t→t+1)을 짝짓기
                                        for i in range(len(timesteps)):
                                            if i + 1 < len(timesteps):
                                                series_data.append(timesteps[i + 1].observation['robot_states'][a_id][key][ee_name])
                                            else:
                                                series_data.append([0.0] * len(value[ee_name]))
                                    else:
                                        for t_step in timesteps:
                                            series_data.append(t_step.observation['robot_states'][a_id][key][ee_name])
                                    agent_group.create_dataset(ee_name, data=np.array(series_data))
                            else:
                                series_data = []
                                for t_step in timesteps:
                                    series_data.append(t_step.observation['robot_states'][a_id][key])
                                agent_group = data_group
                                agent_group.create_dataset(f'robot_{a_id}', data=np.array(series_data))

                    # 3. succeed 플래그 저장 (1-step shifted: action과 동일하게 정렬)
                    succeed_shifted = []
                    for i in range(len(succeed_flags)):
                        if i + 1 < len(succeed_flags):
                            succeed_shifted.append(succeed_flags[i + 1])
                        else:
                            succeed_shifted.append(succeed_flags[-1])
                    root.create_dataset('succeed', data=np.array(succeed_shifted, dtype=np.float32))

                    # 4. 기타 메타데이터
                    root.create_dataset('language_instruction', data=language_instruction if language_instruction else '',
                                    dtype=h5py.string_dtype(encoding='utf-8'))

                    time.sleep(1)

                print("Episode recording process ended.")
                socketio_instance.emit('episode_saved', {'succeed': task_control.get('succeed', False)})

            except Exception:
                import traceback
                print(f"[ERROR] Error during episode recording:\n{traceback.format_exc()}")
                task_control['stop'] = True

            # episode_complete로 조기 저장된 경우 전체 collection 종료
            if task_control.get('episode_complete'):
                task_control['episode_complete'] = False
                socketio_instance.emit('stop_process', {'id': 'record_episode', 'episode_saved': True})
                print("Collection finished (episode completed early).")
                break

    finally:
        # 1. vive teleop 스레드 먼저 정지 (thread_pool.submit 호출 중단)
        if vive is not None:
            vive.destroy()
        # 2. 남은 thread_pool 작업 완료 후 종료
        thread_pool.shutdown(wait=True)
        # 3. 센서 구독 해제
        env.destroy()
