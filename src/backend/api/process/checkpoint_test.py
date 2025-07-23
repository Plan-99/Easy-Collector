import os
import torch
import pickle
import numpy as np
import time

from ...policies.utils import make_policy
from ...env.env import Env

def checkpoint_test(
    checkpoint_id,
    task,
    policy_obj,
    robots,
    sensors,
    socketio_instance,
    task_control,
    gripper=None
    ):
    
    while True:
    
        seed = 1
        
        ckpt_dir = os.path.join("/root/src/backend/checkpoints", str(checkpoint_id))
        ckpt_name = "policy_best.ckpt"
        ckpt_path = os.path.join(ckpt_dir, ckpt_name)
        policy = make_policy(ckpt_dir, seed, policy_obj, task, robots[0], sensors, gripper)
        
        temporal_agg = True
        
        loading_status = policy.load_state_dict(torch.load(ckpt_path))
        policy.cuda()
        policy.eval()
        socketio_instance.emit('log_checkpoint_test', {
            'log': f'Loaded Policy from {ckpt_path}',
            'type': 'stdout '
        })
        
        
        state_dim = 0
        for robot in robots:
            state_dim += robot['joint_dim']
        
        if policy_obj['type'] == 'ACT':
            stats_path = os.path.join(ckpt_dir, f'dataset_stats.pkl')
            with open(stats_path, 'rb') as f:
                stats = pickle.load(f)

            pre_process = lambda s_qpos: (s_qpos - stats['qpos_mean']) / stats['qpos_std']
            post_process = lambda a: a * stats['qaction_std'] + stats['qaction_mean']
            
            if temporal_agg:
                query_frequency = 1
            num_queries = policy_obj['settings']['chunk_size']
            
            max_timesteps = task['episode_len']
            
            env = Env(robots, sensors)
            
            ts = env.reset()
            timesteps = [ts]
            
            if temporal_agg:
                all_time_actions = torch.zeros([max_timesteps, max_timesteps+num_queries, state_dim]).cuda()
                
                
            socketio_instance.emit('checkpoint_test_progress', {
                'progress': 0,
                'type': 'stdout '
            })
            
            
        home_pose = task['home_pose']
        for agent in env.agents:
            agent.move_to(home_pose[str(agent.id)])
            
        socketio_instance.emit('log_checkpoint_test', {
                'log': 'Robot moved to homepose',
                'type': 'stdout '
            })
            
        
            
            
        for t in range(max_timesteps):

            start = time.time()
            ### process previous timestep to get qpos and image_list
            timesteps.append(ts)
            obs = ts.observation
                
                
            # 로봇 상태 전처리
            curr_qpos_ls = []
            for robot_state in obs['robot_states'].values():
                curr_qpos_ls.append(robot_state['qpos'])
            concatenated_pos = np.concatenate(curr_qpos_ls, axis=0)
            robot_input_raw = concatenated_pos
            robot_input = pre_process(robot_input_raw)
            robot_input = torch.from_numpy(robot_input).float().cuda().unsqueeze(0)
            
            
            # 이미지 전처리
            curr_image_ls = []
            for image in obs['images'].values():
                curr_image_ls.append(image)
            curr_image = np.stack(curr_image_ls, axis=0)
            curr_image = torch.from_numpy(curr_image / 255.0).float().cuda().unsqueeze(0).permute(0, 1, 4, 2, 3)
            time.sleep(0.5)  # Simulate processing time
            
            with torch.inference_mode():
                ### query policy
                if policy_obj['type'] == "ACT":
                    if t % query_frequency == 0:
                        all_actions = policy(robot_input, curr_image)
                    if temporal_agg:
                        all_time_actions[[t], t:t+num_queries] = all_actions
                        actions_for_curr_step = all_time_actions[:, t]
                        actions_populated = torch.all(actions_for_curr_step != 0, axis=1)
                        actions_for_curr_step = actions_for_curr_step[actions_populated]
                        k = 0.01
                        exp_weights = np.exp(-k * np.arange(len(actions_for_curr_step)))
                        exp_weights = exp_weights / exp_weights.sum()
                        exp_weights = torch.from_numpy(exp_weights).cuda().unsqueeze(dim=1)
                        raw_action = (actions_for_curr_step * exp_weights).sum(dim=0, keepdim=True)
                        # raw_action = all_actions[:, t % query_frequency]
                    else:
                        raw_action = all_actions[:, t % query_frequency]
                else:
                    raise NotImplementedError
                
            raw_action = raw_action.squeeze(0).cpu().numpy()
            action = post_process(raw_action)
            
            start_action_id = 0
            for agent in env.agents:
                target_qpos = action[start_action_id:start_action_id + agent.joint_len]
                agent.move_step(target_qpos)
                
                
            socketio_instance.emit('checkpoint_test_progress', {
                'progress': (t+1) / max_timesteps,
                'type': 'stdout '
            })
            
            socketio_instance.emit('log_checkpoint_test', {
                'log': raw_action.tolist(),
                'type': 'stdout '
            })
            
            if task_control['stop']:
                socketio_instance.emit('log_checkpoint_test', {
                    'log': f'Stopping Data Collection',
                    'type': 'stdout '
                })
                return
            
                
        time.sleep(3)  # Simulate processing time
                
    socketio_instance.emit('log_checkpoint_test', {
        'log': f'Stopping Checkpoint Test',
        'type': 'stdout '
    })
            
            # curr_image, memories = get_image(ts, camera_names, config['camera_config'], yolo_config, memories)
            
            
# def get_image(ts, camera_names, camera_config, memories, yolo_config=None):
#     curr_images = []
#     raw_images = []

#     for index, cam_name in enumerate(camera_names):
#         image = ts.observation['images'][cam_name]

#         if cam_name in camera_config:
#             image, memories[index] = fetch_image_with_config(image, camera_config[cam_name], memories[index], yolo_config)


#         raw_images.append(image)
#         curr_image = rearrange(image, 'h w c -> c h w')
#         curr_images.append(curr_image)

#     if len(raw_images):

            
#         # 이미지 크기 맞추기 (최대 크기로 맞추거나 다른 방식으로 조정)
#         max_height = 480
#         max_width = 640
#         resized_images = [cv2.resize(img, (max_width, max_height)) for img in raw_images]

#         # 이미지를 가로로 나열
#         combined_image = cv2.hconcat(resized_images)

#         # # 단일 창에 표시
#         # cv2.imshow("Combined Image", combined_image)
#         # cv2.waitKey(1)
#     else:
#         print("No images to display.")


#     curr_image = np.stack(curr_images, axis=0)
#     curr_image = torch.from_numpy(curr_image / 255.0).float().cuda().unsqueeze(0)

#     return curr_image, memories
        


# def main(args):
#     checkpoint = Checkpoint.find(args.checkpoint_id)
#     task = Task.find(checkpoint['task_id'])
#     policy = Policy.find(checkpoint['policy_id'])
#     robots = [Robot.find(rid).to_dict() for rid in task['robot_ids']]
#     sensors = [Sensor.find(sid).to_dict() for sid in task['sensor_ids']]
    
#     checkpoin(
#         checkpoint,
#         task,
#         policy,
#         robots,
#         sensors
#     )



# if __name__ == '__main__':
    
#     parser = argparse.ArgumentParser()
#     parser.add_argument('--checkpoint_id', required=True)
    
#     main(parser.parse_args())
#     sys.exit(0)
