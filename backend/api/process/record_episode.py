from tqdm import tqdm
import os
from ...bridge.remote_env import RemoteEnv as Env
from ...configs.global_configs import DATASET_DIR
from .leader_teleoperation import Leader
from ...utils.image_parser import fetch_image_with_config
import time
from concurrent.futures import ThreadPoolExecutor
from ...utils.lerobot_io import append_episode as lerobot_append_episode, get_episode_count
from ...database.models.teleoperator_model import Teleoperator as TeleoperatorModel

def get_auto_index(dataset_dir, dataset_name_prefix = '', data_suffix = 'hdf5'):
    max_idx = 1000
    if not os.path.isdir(dataset_dir):
        os.makedirs(dataset_dir)
    for i in range(max_idx+1):
        if not os.path.isfile(os.path.join(dataset_dir, f'{dataset_name_prefix}episode_{i}.{data_suffix}')):
            return i
    raise Exception(f"Error getting auto index, or more than {max_idx} episodes")



def record_episode(node, dataset_id, agents, move_homepose, assembly_id, sensors, task, language_instruction, socketio_instance, task_control, tele_type='leader', ros2_service='', iter=100000, hz=20, move_homepose_duration=5.0):
    agents = sorted(agents, key=lambda a: a.id)
    tutorial = any((s.get('settings') or {}).get('is_tutorial') for s in (sensors or []))
    env = Env(agents=agents, sensors=sensors, virtual_agents=(tele_type == 'vive_only'), tutorial=tutorial)
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

            _outer_t0 = time.monotonic()
            print(f"[record_episode/T] outer iter start", flush=True)
            task_control['episode_stop'] = False
            task_control['succeed'] = False

            # Tutorial mode: snap the MuJoCo world back to its home keyframe
            # at the start of every episode so each recording begins from the
            # same arm pose AND cube location.
            # NOTE: tutorial 이 아닌데도 호출하면 mujoco reset_world ROS service 가
            # 없어서 gRPC ROSProxy 가 ROS service 기본 timeout(=10초)을 다 채우고
            # 실패한다 → episode 시작마다 10초 dead time. tutorial 일 때만 호출.
            if tutorial:
                try:
                    from ..routes.tutorial import reset_tutorial_world
                    reset_tutorial_world()
                except Exception as e:
                    print(f"[record_episode] tutorial reset skipped: {e}")

            ep_count = get_episode_count(dataset_dir)
            dataset_name = f"episode_{ep_count:06d}"

            print(f"Recording Data: {dataset_name}")

            # Validate task config before recording. Common UI gap: episode_len left
            # blank (None) → range(None) raises the cryptic "'NoneType' object cannot
            # be interpreted as an integer" error after sensor session start.
            max_timesteps = task.get('episode_len')
            if max_timesteps is None or max_timesteps <= 0:
                raise ValueError(
                    f"Task '{task.get('name', '?')}' (id={task.get('id', '?')}) has "
                    f"episode_len={max_timesteps}. Set Episode Length > 0 in the task "
                    f"settings (UI) before starting recording."
                )
            home_pose = task.get('home_pose')

            print(f"[record_episode/T] +{(time.monotonic()-_outer_t0)*1000:.0f}ms emit progress=0", flush=True)
            socketio_instance.emit('record_episode_progress', {
                'progress': 0,
                'type': 'stdout '
            })

            _ep_t0 = time.monotonic()
            if move_homepose and tele_type not in ('externel', 'vive_only'):
                print(f"[record_episode/T] +{(time.monotonic()-_outer_t0)*1000:.0f}ms emit moving_homepose=true", flush=True)
                socketio_instance.emit('moving_homepose', {'moving': True})
                # agent별 move_to + reset_ik_solver 를 thread_pool 로 동시 호출.
                # 이전엔 sequential 이어서 agent마다 gRPC RPC(이전 thread cancel/join
                # 0.5s 가능 + pinocchio reset 100-200ms) 가 누적 = "0% 후 dead time"
                # 의 주요 원인이었다.
                def _start_homepose(agent):
                    if tele_type == 'vive_external' and agent.role == 'tool':
                        return
                    agent.move_lock = True
                    _t_mv = time.monotonic()
                    agent.move_to(home_pose[str(agent.id)], duration=move_homepose_duration)
                    _t_mv = (time.monotonic() - _t_mv) * 1000
                    _t_ik = 0.0
                    if agent.ik_solver is not None:
                        _t = time.monotonic()
                        agent.reset_ik_solver(home_pose[str(agent.id)])
                        _t_ik = (time.monotonic() - _t) * 1000
                    print(f"[record_episode] agent {agent.id} move_to={_t_mv:.0f}ms "
                          f"reset_ik={_t_ik:.0f}ms", flush=True)

                _hp_futures = [thread_pool.submit(_start_homepose, a) for a in agents]
                for _f in _hp_futures:
                    try:
                        _f.result(timeout=10.0)
                    except Exception as _e:
                        print(f"[record_episode] homepose start failed: {_e}", flush=True)
                print(f"[record_episode] homepose dispatch total: "
                      f"{(time.monotonic()-_ep_t0)*1000:.0f}ms", flush=True)

                # is_moving 플래그로 도달 대기 (타임아웃 30초). move_to 가
                # 비동기 thread 라 stop 신호 시 cancel_move_to 로 명시 중단해야
                # background thread 가 명령을 계속 쏘는 걸 막을 수 있다.
                timeout = 30.0
                start_wait = time.time()
                while time.time() - start_wait < timeout:
                    if task_control['stop']:
                        for a in agents:
                            try:
                                a.cancel_move_to()
                            except Exception:
                                pass
                        socketio_instance.emit('moving_homepose', {'moving': False})
                        return
                    moving_agents = [a for a in agents if a.is_moving]
                    if not moving_agents:
                        break
                    time.sleep(0.1)
                socketio_instance.emit('moving_homepose', {'moving': False})

            else:
                # move_homepose가 아니어도 IK solver 내부 상태를 현재 조인트에 동기화
                for agent in agents:
                    if agent.ik_solver is not None:
                        js = agent.get_joint_states()
                        if js is not None:
                            agent.reset_ik_solver(js)
            # NOTE: 이전엔 here 에 time.sleep(0.5) / 1 / 2 hard wait 가 있었는데,
            # 다음 sensor 첫 프레임 polling / joint state 검증 단계가 어차피 대기를
            # 강제하므로 redundant. 제거해서 record 시작 latency 단축.
            # --- motion_planning: ROS2 service 호출 (gRPC ROSProxy 경유) ---
            if tele_type == 'motion_planning' and ros2_service:
                service_result = {'done': False, 'success': None, 'message': ''}
                try:
                    import threading as _th
                    def _call_ros2_service():
                        try:
                            client = get_bridge_client()
                            resp = client.ros_proxy.CallService(pb.ROSServiceRequest(
                                service_type='std_srvs/srv/Trigger',
                                service_name=ros2_service,
                                request_json='',
                            ))
                            service_result['done'] = True
                            service_result['success'] = resp.success
                            service_result['message'] = resp.response_json
                            if resp.success:
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
                # Peewee 쿼리 — 기존 Orator 스타일(`where('col', val)`)은 더 이상 동작 안 함.
                teleop = TeleoperatorModel.select().where(
                    TeleoperatorModel.type == 'leader',
                    TeleoperatorModel.assembly_id == assembly_id,
                    TeleoperatorModel.deleted_at.is_null(),
                ).first()

                if teleop is None:
                    print(f'[ERROR]: No leader robot preset for assembly {assembly_id}')
                    task_control['stop'] = True
                    return

                leader = Leader(node, agents, socketio_instance, teleop._settings)

                socketio_instance.start_background_task(
                    target=leader.leader_teleop_workflow,
                    task_control=task_control
                )

                while not leader.is_synced:
                    # print('[NOTICE] Waiting for leader teleoperation to sync...')
                    time.sleep(0.1)

            # joint commands 대기 (keyboard는 사용자 입력 전까지 actions 없으므로 스킵)
            if tele_type != 'keyboard':
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
            else:
                # keyboard: 사용자가 키 누르기 전엔 joint_actions가 None이라 dataset에
                # 빈 action 배열이 저장되고 replay 시 0벡터로 들어가 로봇이 영점으로 가는
                # 문제가 있음. 현재 joint_states로 채워 reset 프레임 qaction이 정상값을
                # 갖게 한다. 이후 사용자가 키를 누르면 move_*_step이 새 값으로 덮어씀.
                for agent in agents:
                    if agent.joint_actions is None:
                        js = agent.get_joint_states()
                        if js is not None:
                            agent.joint_actions = list(js)
            for agent in agents:
                agent.move_lock = False

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
            # UI 가 progress bar 대신 로딩 화면을 띄울 수 있게 saving on/off 신호.
            socketio_instance.emit('episode_saving', {'saving': True, 'name': dataset_name})

            _save_t0 = time.monotonic()
            try:
                # --- LeRobot 포맷으로 저장 ---
                # action_key 인자 안 보냄 — record 시점엔 모든 raw 데이터를 다 저장
                # 하고 학습 시 _get_action_data 가 action_key 에 따라 적절한 컬럼을
                # 선택/derive 함. action_key 는 info.json 메타용 (default 'joint').
                lerobot_append_episode(
                    dataset_dir=dataset_dir,
                    timesteps=timesteps,
                    agents=agents,
                    sensors=sensors,
                    task=task,
                    language_instruction=language_instruction if language_instruction else "",
                    succeed_flags=succeed_flags,
                    fetch_image_fn=fetch_image_with_config,
                    fps=hz,
                )

                print(f"[record_episode/T] lerobot_append_episode took {(time.monotonic()-_save_t0)*1000:.0f}ms", flush=True)
                print("Episode recording process ended.")
                socketio_instance.emit('episode_saved', {'succeed': task_control.get('succeed', False)})

            except Exception:
                import traceback
                print(f"[ERROR] Error during episode recording:\n{traceback.format_exc()}")
                task_control['stop'] = True
            finally:
                socketio_instance.emit('episode_saving', {'saving': False})
                print(f"[record_episode/T] emit episode_saving=false (total save {(time.monotonic()-_save_t0)*1000:.0f}ms)", flush=True)

            # Finish 버튼(=episode_complete) 는 "현재 에피소드 즉시 저장 후 다음 에피소드로
            # 진행" 의미로 바꿈. 전체 collection 종료는 Stop 버튼(stop_collection 라우트)
            # 에서만 일어난다. flag 만 reset 하고 outer loop 는 계속.
            if task_control.get('episode_complete'):
                task_control['episode_complete'] = False
                print("Episode completed early — proceeding to next episode.")

    finally:
        # 0. move_lock 강제 해제 — home pose 이동 중 stop/error로 빠져나간 경우
        # 이 플래그가 True로 남으면 이후 텔레옵의 move_robot_ee_delta 등 모든
        # socket handler가 즉시 return해 로봇이 안 움직이는 상태가 된다.
        for agent in agents:
            try:
                agent.move_lock = False
            except Exception:
                pass
        # 1. vive teleop 스레드 먼저 정지 (thread_pool.submit 호출 중단)
        if vive is not None:
            vive.destroy()
        # 2. 남은 thread_pool 작업 완료 후 종료
        thread_pool.shutdown(wait=True)
        # 3. 센서 구독 해제
        env.destroy()
