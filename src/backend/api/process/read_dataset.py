import numpy as np

import time
import os
import cv2
import base64
import threading
from .augment_dataset import adjust_lightness, draw_rectangles, add_salt_and_pepper_noise, add_gaussian_noise, generate_rect_params, prospective_transform, generate_prospective_transform, apply_hsv
from .lerobot_io import read_episode, append_episode as lerobot_append_episode
from ...configs.global_configs import DATASET_DIR
from ...utils.image_parser import fetch_image_with_config
from PIL import Image


config = {}


def read_dataset(node, episode_path, socketio_instance, sid, task_control, move_robot=False, sensors=None, agents=None, task=None, hz=5, capture_dataset_id=None, action_key='qaction'):
    """Read and stream episode data from a LeRobot dataset.

    episode_path is now repurposed as dataset_dir/episode_NNNNNN (kept param name for API compat).
    """
    global config
    config = {}
    capture_env = None
    if move_robot:
        from ...env.env import Env
        env = Env(node, agents, sensors)
        if capture_dataset_id is not None:
            capture_env = env

    # Parse dataset_dir and episode_index from path
    # episode_path format: "/root/src/backend/datasets/{dataset_id}/episode_{index:06d}"
    # or legacy: "/root/src/backend/datasets/{dataset_id}/episode_{index}.hdf5"
    path_parts = episode_path.rsplit("/", 1)
    dataset_dir = path_parts[0]
    episode_name = path_parts[1] if len(path_parts) > 1 else "episode_000000"

    # Extract episode index from name
    ep_name_clean = episode_name.replace(".hdf5", "")
    ep_index_str = ep_name_clean.replace("episode_", "")
    episode_index = int(ep_index_str)

    while True:
        # Read episode from LeRobot dataset
        ep_data = read_episode(dataset_dir, episode_index)

        image_data = ep_data["images"]       # {sensor_name: [np.array, ...]}
        states_by_robot = ep_data["states"]   # {robot_name: np.array (T, D)}
        actions_by_robot = ep_data["actions"]  # {robot_name: np.array (T, D)}
        language_instruction = ep_data["language_instruction"]
        num_frames = ep_data["num_frames"]

        sensor_names = sorted(image_data.keys())
        robot_names = sorted(states_by_robot.keys())

        if move_robot:
            # 홈포즈가 아니라 에피소드 첫 프레임의 qpos로 이동
            for robot_name in robot_names:
                first_qpos = states_by_robot[robot_name][0].tolist()
                agent_id_str = robot_name.replace("robot_", "")
                for agent in agents:
                    if str(agent.id) == agent_id_str:
                        agent.move_to(first_qpos, duration=5.0)
                        break

            time.sleep(10)

        capture_timesteps = []
        if capture_env is not None:
            ts = capture_env.reset()
            capture_timesteps.append(ts)

        total_steps = num_frames
        if capture_env is not None and task and task.get('episode_len'):
            total_steps = min(total_steps, task['episode_len'])

        # 타임스텝별로 데이터 전송
        # 이미지 인코딩 + socketio 전송은 무거우므로 별도 스레드에서 처리하고,
        # 메인 루프는 로봇 커맨드 전송 + 타이밍만 담당한다.
        period = 1.0 / hz
        next_tick = time.time()
        last_tick_time = time.time()

        def _encode_and_emit(step_idx, cur_config):
            """이미지 인코딩 + socketio emit (별도 스레드에서 실행)"""
            encoded_images = {}
            local_rect_params = []

            for cam_name in sensor_names:
                img_array = image_data[cam_name][step_idx]

                img = Image.fromarray(img_array)
                if 'lightness' in cur_config:
                    img = adjust_lightness(img, cur_config['lightness'])
                if 'rectangles' in cur_config:
                    if len(local_rect_params) != cur_config['rectangles'].get('count', 0):
                        local_rect_params = generate_rect_params(cur_config['rectangles'], img.width, img.height)
                    img = draw_rectangles(img, local_rect_params)
                if 'saltAndPepper' in cur_config:
                    img = add_salt_and_pepper_noise(img, cur_config['saltAndPepper'].get('amount', 0))
                if 'gaussian' in cur_config:
                    img = add_gaussian_noise(img, cur_config['gaussian'].get('mean', 0), cur_config['gaussian'].get('sigma', 0))
                if 'prospective' in cur_config:
                    transform_matrix = generate_prospective_transform(img.width, img.height,
                                                                      cur_config['prospective'].get('scale_factor', 0),
                                                                      cur_config['prospective'].get('degrees', 0),
                                                                      cur_config['prospective'].get('shear', 0),
                                                                      cur_config['prospective'].get('perspective', 0))
                    img = prospective_transform(img, transform_matrix)

                if 'hsv' in cur_config and cur_config['hsv']:
                    hsv_config = cur_config['hsv']
                    if hsv_config.get('random'):
                        h_gain = 0.5
                        s_gain = 0.7
                        v_gain = 0.4
                        h_adj = (np.random.rand() * 2 - 1) * h_gain * 180
                        s_adj = (np.random.rand() * 2 - 1) * s_gain + 1
                        v_adj = (np.random.rand() * 2 - 1) * v_gain + 1
                    else:
                        h_adj = hsv_config.get('h', 0) * 180
                        s_adj = 1 + hsv_config.get('s', 0)
                        v_adj = 1 + hsv_config.get('v', 0)

                    img = apply_hsv(img, h_adj, s_adj, v_adj)

                img_array = np.array(img)
                img_array = cv2.cvtColor(img_array, cv2.COLOR_RGB2BGR)

                success, buffer = cv2.imencode('.jpg', img_array)
                if not success:
                    continue

                jpg_as_text = base64.b64encode(buffer).decode('utf-8')
                encoded_images[cam_name] = f"data:image/jpeg;base64,{jpg_as_text}"

            # Robot states (UI 표시용 — 이미 메인 루프에서 로봇에는 보냈음)
            robot_states = {}
            for robot_name in robot_names:
                robot_states[robot_name] = {
                    'qpos': states_by_robot[robot_name][step_idx].tolist(),
                    'qaction': actions_by_robot[robot_name][step_idx].tolist(),
                }

            socketio_instance.emit('show_episode_step', {
                'episode_path': episode_path,
                'images': encoded_images,
                'robot_states': robot_states,
                'language_instruction': language_instruction,
            }, to=sid)

        for i in range(total_steps):
            now = time.time()
            dt = now - last_tick_time
            last_tick_time = now
            if dt > 0:
                print(f"[ReadDataset] {1.0/dt:.1f} Hz", flush=True)

            if task_control['stop']:
                print("Stopping read_dataset process. Captured data discarded.")
                return

            # --- 로봇 커맨드 전송 (타이밍 크리티컬) ---
            if move_robot:
                for robot_name in robot_names:
                    qaction_array = actions_by_robot[robot_name][i]
                    for agent in agents:
                        if str(agent.id) == robot_name.replace("robot_", ""):
                            if action_key == 'ee_delta_action' and agent.role != 'tool' and agent.ik_solver is not None:
                                ee_name = agent.ee_names[0]
                                action_list = qaction_array.tolist()
                                ee_delta = {ee_name: action_list[:6]}
                                tool_positions = action_list[6:] if agent.tool_inner and len(action_list) > 6 else None
                                agent.move_ee_delta_step(ee_delta, tool_positions=tool_positions)
                            else:
                                agent.move_joint_step(qaction_array)

            if capture_env is not None:
                ts = capture_env.record_step()
                capture_timesteps.append(ts)
                socketio_instance.emit('replay_capture_progress', {
                    'progress': (i + 1) / total_steps,
                })

            # --- 이미지 인코딩 + UI 전송은 백그라운드 스레드 ---
            cur_config_snapshot = dict(config)
            threading.Thread(
                target=_encode_and_emit,
                args=(i, cur_config_snapshot),
                daemon=True,
            ).start()

            # 처리 시간을 고려해 남은 시간만큼 대기
            # time.sleep()은 Linux 스케줄러 해상도(~1-4ms) 때문에 오버슬립하므로,
            # 마지막 2ms는 spin-wait로 정밀하게 맞춘다.
            next_tick += period
            remaining = next_tick - time.time()
            if remaining > 0.002:
                time.sleep(remaining - 0.002)
            if remaining > 0:
                while time.time() < next_tick:
                    pass
            else:
                next_tick = time.time()

        # Save captured episode to target dataset
        if capture_env is not None and capture_timesteps and capture_dataset_id is not None:
            _save_captured_episode(capture_dataset_id, capture_timesteps, agents, sensors, task, language_instruction)
        
        break

def _save_captured_episode(dataset_id, timesteps, agents, sensors, task, language_instruction):
    dataset_dir = os.path.join(DATASET_DIR, str(dataset_id))

    print(f"[Capture] Saving captured episode to {dataset_dir}")

    try:
        lerobot_append_episode(
            dataset_dir=dataset_dir,
            timesteps=timesteps,
            agents=agents,
            sensors=sensors,
            task=task,
            language_instruction=language_instruction if language_instruction else "",
            action_key='qaction',
            fetch_image_fn=fetch_image_with_config,
        )
        print(f"[Capture] Episode saved ({len(timesteps)} timesteps)")
    except Exception:
        import traceback
        print(f"[Capture ERROR] Failed to save captured episode:\n{traceback.format_exc()}")


def add_config(config_data):
    global config
    config.update(config_data)
