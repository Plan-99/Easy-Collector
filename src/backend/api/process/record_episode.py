from tqdm import tqdm
import os
from ...bridge.remote_env import RemoteEnv
from ...configs.global_configs import DATASET_DIR
from .leader_teleoperation import Leader
from ...utils.image_parser import fetch_image_with_config
import time
from concurrent.futures import ThreadPoolExecutor
from .lerobot_io import append_episode as lerobot_append_episode, get_episode_count
from ...database.models.teleoperator_model import Teleoperator as TeleoperatorModel

def get_auto_index(dataset_dir, dataset_name_prefix = '', data_suffix = 'hdf5'):
    max_idx = 1000
    if not os.path.isdir(dataset_dir):
        os.makedirs(dataset_dir)
    for i in range(max_idx+1):
        if not os.path.isfile(os.path.join(dataset_dir, f'{dataset_name_prefix}episode_{i}.{data_suffix}')):
            return i
    raise Exception(f"Error getting auto index, or more than {max_idx} episodes")



def record_episode(node, dataset_id, agents, move_homepose, assembly_id, sensors, task, language_instruction, socketio_instance, task_control, tele_type='leader', ros2_service='', iter=100000, hz=20):
    agents = sorted(agents, key=lambda a: a.id)
    env = RemoteEnv(agents=agents, sensors=sensors, virtual_agents=(tele_type == 'vive_only'))
    dataset_dir = f"{DATASET_DIR}/{dataset_id}"
    thread_pool = ThreadPoolExecutor(max_workers=len(agents))

    # --- Vive: collection 전체에서 1회 초기화 (에피소드마다 재시작하지 않음) ---
    # vive_external: 실물 로봇 + vive, vive_only: vive tracker만 (이미지+ee_delta_action)
    vive = None
    if tele_type in ('vive_external', 'vive_only'):
        from ...bridge.remote_vive import RemoteViveController
        move_robot = (tele_type == 'vive_external')
        vive = RemoteViveController(
            socketio_instance,
            agents=agents, move_robot=move_robot,
            scale_factor=2, step_rate=40,
        )
        if not vive.wait_for_ready(timeout=30.0):
            task_control['stop'] = True
            vive.destroy()
            return

    task_control['episode_complete'] = False

    # --- ROS2 service (motion_planning용) ---
    from ...bridge.client import get_bridge_client
    from ...bridge.generated import robot_bridge_pb2 as pb
    service_result = {'done': False, 'success': None, 'message': ''}

    try:
        for _ in range(iter):

            if task_control['stop']:
                break

            task_control['episode_stop'] = False
            task_control['succeed'] = False

            ep_count = get_episode_count(dataset_dir)
            dataset_name = f"episode_{ep_count:06d}"

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
                            agent.reset_ik_solver(js)
                time.sleep(1)

            time.sleep(1)
            # --- motion_planning: ROS2 service 호출 (gRPC ROSProxy 경유) ---
            if tele_type == 'motion_planning' and ros2_service:
                service_result = {'done': False, 'success': None, 'message': ''}
                try:
                    import threading as _th
                    import json as _json
                    def _call_ros2_service():
                        try:
                            client = get_bridge_client()
                            resp = client.ros_proxy.CallService(pb.ROSServiceRequest(
                                service_type='std_srvs/srv/Trigger',
                                service_name=ros2_service,
                                request_json='',
                            ))
                            # gRPC 레벨 성공 + 응답 JSON 내부의 success 필드도 함께 검사
                            inner_success = True
                            if resp.response_json:
                                try:
                                    parsed = _json.loads(resp.response_json)
                                    if isinstance(parsed, dict) and 'success' in parsed:
                                        inner_success = bool(parsed['success'])
                                except (ValueError, TypeError):
                                    pass
                            overall_success = bool(resp.success) and inner_success
                            service_result['done'] = True
                            service_result['success'] = overall_success
                            service_result['message'] = resp.response_json
                            if overall_success:
                                print(f'[NOTICE] ROS2 service "{ros2_service}" completed: {resp.response_json}')
                            else:
                                print(f'[ERROR] ROS2 service "{ros2_service}" failed: {resp.response_json}')
                        except Exception as e:
                            service_result['done'] = True
                            service_result['success'] = False
                            service_result['message'] = str(e)
                            print(f'[ERROR] ROS2 service error: {e}')

                    _th.Thread(target=_call_ros2_service, daemon=True).start()
                    print(f'[NOTICE] ROS2 service "{ros2_service}" called, recording in parallel...')
                except Exception as e:
                    print(f'[ERROR] Failed to call ROS2 service: {e}')
                    task_control['stop'] = True
                    return

            if not os.path.isdir(dataset_dir):
                os.makedirs(dataset_dir)

            # motion_planning: joint_actions 초기화 후 대기
            if tele_type == 'motion_planning':
                for agent in agents:
                    agent.joint_actions = None

            # vive_only는 실물 로봇 없이 동작하므로 joint state 검증 생략
            if tele_type != 'vive_only':
                for agent in agents:
                    js = agent.get_joint_states()
                    if js is not None:
                        agent.joint_states = js
                    if agent.joint_states is None:
                        print(f'[ERROR] No joint states from robot {agent.id}')
                        task_control['stop'] = True
                        return

            # 센서 첫 프레임 대기: get_observation()으로 이미지 확인
            for sensor in sensors:
                sensor_key = f'sensor_{sensor["id"]}'
                print(f'[NOTICE] Waiting for first frame from sensor {sensor["id"]}...')
                elapsed = 0.0
                while True:
                    if task_control['stop']:
                        return
                    try:
                        obs = env.get_observation()
                        img = obs.get('images', {}).get(sensor_key)
                        if img is not None:
                            break
                    except Exception:
                        pass
                    time.sleep(0.1)
                    elapsed += 0.1
                    if elapsed >= 10.0:
                        print(f'[ERROR] No data from sensor {sensor["id"]} after 10.0s timeout')
                        task_control['stop'] = True
                        return
                print(f'[NOTICE] Sensor {sensor["id"]} first frame received ({elapsed:.1f}s)')

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
                    # print('[NOTICE] Waiting for leader teleoperation to sync...')
                    time.sleep(0.1)

            # joint commands 대기
            # - keyboard: 사용자 입력 전까지 actions 없으므로 스킵
            # - motion_planning: /start_moving 토픽 신호로 대체하므로 스킵
            if tele_type not in ('keyboard', 'motion_planning'):
                wait_timeout = 5.0
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
            for agent in agents:
                agent.move_lock = False

            # motion_planning: 외부 플래너가 /start_moving 토픽(std_msgs/String)을 발행할 때까지 대기
            if tele_type == 'motion_planning':
                import grpc as _grpc
                _client = get_bridge_client()
                print(f'[NOTICE] Waiting for /start_moving topic (std_msgs/msg/String)...')
                _call = _client.ros_proxy.WaitForTopic.future(pb.WaitForTopicRequest(
                    topic_name='/start_moving',
                    msg_type='std_msgs/msg/String',
                    timeout=0.0,  # 서버에서 무한 대기 (클라이언트 취소로 중단)
                ))
                _service_failed_early = False
                while not _call.done():
                    if task_control['stop']:
                        _call.cancel()
                        return
                    if service_result['done'] and not service_result['success']:
                        print(f'[ERROR] ROS2 service failed before /start_moving: {service_result["message"]}')
                        _call.cancel()
                        _service_failed_early = True
                        break
                    time.sleep(0.1)

                if _service_failed_early:
                    continue  # 에피소드 재시작

                try:
                    _resp = _call.result()
                except _grpc.RpcError as _e:
                    code = _e.code() if hasattr(_e, 'code') else None
                    details = _e.details() if hasattr(_e, 'details') else str(_e)
                    print(f'[ERROR] WaitForTopic RPC error: code={code} details={details}')
                    if code == _grpc.StatusCode.UNIMPLEMENTED:
                        print('[ERROR] ROS2 gRPC bridge does not have WaitForTopic registered. '
                              'Restart easy_collector_ros2 container: docker restart easy_collector_ros2')
                    task_control['stop'] = True
                    return

                if _resp.success:
                    print(f'[NOTICE] /start_moving received: {_resp.message_json}')
                else:
                    print(f'[ERROR] WaitForTopic returned failure: {_resp.message_json}')
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
                        time.sleep(0.02)

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

                if tele_type != 'keyboard':
                    time.sleep(1.0 / hz)
                else:
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
                # --- LeRobot 포맷으로 저장 ---
                lerobot_append_episode(
                    dataset_dir=dataset_dir,
                    timesteps=timesteps,
                    agents=agents,
                    sensors=sensors,
                    task=task,
                    language_instruction=language_instruction if language_instruction else "",
                    action_key='qaction',
                    succeed_flags=succeed_flags,
                    fetch_image_fn=fetch_image_with_config,
                    fps=hz,
                )

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
