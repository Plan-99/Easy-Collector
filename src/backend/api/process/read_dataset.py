import numpy as np

import time
import os
import cv2
import base64
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

    hz = 20

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
        if move_robot:
            home_pose = task.get('home_pose', None)

            if home_pose is not None:
                for agent in agents:
                    if str(agent.id) in home_pose:
                        agent.move_to(home_pose[str(agent.id)], duration=5.0)

            time.sleep(5)

        # Read episode from LeRobot dataset
        ep_data = read_episode(dataset_dir, episode_index)

        image_data = ep_data["images"]       # {sensor_name: [np.array, ...]}
        states_by_robot = ep_data["states"]   # {robot_name: np.array (T, D)}
        actions_by_robot = ep_data["actions"]  # {robot_name: np.array (T, D)}
        language_instruction = ep_data["language_instruction"]
        num_frames = ep_data["num_frames"]

        sensor_names = sorted(image_data.keys())
        robot_names = sorted(states_by_robot.keys())

        rect_params = []

        capture_timesteps = []
        if capture_env is not None:
            ts = capture_env.reset()
            capture_timesteps.append(ts)

        total_steps = num_frames
        if capture_env is not None and task and task.get('episode_len'):
            total_steps = min(total_steps, task['episode_len'])

        # 타임스텝별로 데이터 전송
        for i in range(total_steps):

            if task_control['stop']:
                print("Stopping read_dataset process. Captured data discarded.")
                return

            # --- 이미지를 Base64 문자열로 인코딩 ---
            encoded_images = {}

            for cam_name in sensor_names:
                img_array = image_data[cam_name][i]

                img = Image.fromarray(img_array)
                if 'lightness' in config:
                    img = adjust_lightness(img, config['lightness'])
                if 'rectangles' in config:
                    if len(rect_params) != config['rectangles'].get('count', 0):
                        rect_params = generate_rect_params(config['rectangles'], img.width, img.height)
                    img = draw_rectangles(img, rect_params)
                if 'saltAndPepper' in config:
                    img = add_salt_and_pepper_noise(img, config['saltAndPepper'].get('amount', 0))
                if 'gaussian' in config:
                    img = add_gaussian_noise(img, config['gaussian'].get('mean', 0), config['gaussian'].get('sigma', 0))
                if 'prospective' in config:
                    transform_matrix = generate_prospective_transform(img.width, img.height,
                                                                      config['prospective'].get('scale_factor', 0),
                                                                      config['prospective'].get('degrees', 0),
                                                                      config['prospective'].get('shear', 0),
                                                                      config['prospective'].get('perspective', 0))
                    img = prospective_transform(img, transform_matrix)

                if 'hsv' in config and config['hsv']:
                    hsv_config = config['hsv']
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

            # --- Robot states ---
            robot_states = {}

            for robot_name in robot_names:
                qpos_array = states_by_robot[robot_name][i]
                qaction_array = actions_by_robot[robot_name][i]
                robot_states[robot_name] = {
                    'qpos': qpos_array.tolist(),
                    'qaction': qaction_array.tolist(),
                }
                if move_robot:
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

            time.sleep(1.0 / hz)
            socketio_instance.emit('show_episode_step', {
                'episode_path': episode_path,
                'images': encoded_images,
                'robot_states': robot_states,
                'language_instruction': language_instruction,
            }, to=sid)

            time.sleep(1.0 / hz)

        # Save captured episode to target dataset
        if capture_env is not None and capture_timesteps and capture_dataset_id is not None:
            _save_captured_episode(capture_dataset_id, capture_timesteps, agents, sensors, task, language_instruction)


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
