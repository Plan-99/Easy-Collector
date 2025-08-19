import os
import torch
import pickle
import numpy as np
import time
import cv2
from einops import rearrange
from ...utils.image_parser import fetch_image_with_config

from ...policies.utils import make_policy
from ...env.env import Env

from ...lerobot.policies.act.modeling_act import ACTPolicy
from ...lerobot.policies.diffusion.modeling_diffusion import DiffusionPolicy

def checkpoint_test(
    node,
    checkpoint_id,
    task,
    policy_obj,
    robots,
    sensors,
    socketio_instance,
    task_control,
    max_timesteps,
    ):
    
    try:
        ckpt_dir = os.path.join("/root/src/backend/checkpoints", str(checkpoint_id))
        # policy = make_policy(ckpt_dir, seed, 0, 0, policy_obj, task, robots[0], sensors, gripper)
        if policy_obj['type'] == 'ACT':
            policy = ACTPolicy.from_pretrained(ckpt_dir)
        elif policy_obj['type'] == 'Diffusion':
            policy = DiffusionPolicy.from_pretrained(ckpt_dir)
        
        socketio_instance.emit('log_checkpoint_test', {
            'log': f'Loaded Policy from {ckpt_dir}',
            'type': 'stdout '
        })
        
        state_dim = 0
        for robot in robots:
            state_dim += robot['joint_dim']

            
        env = Env(node, robots, sensors)
    except Exception as e:
        import traceback
        error_string = traceback.format_exc()
        print(error_string)
        socketio_instance.emit('log_checkpoint_test', {
            'log': f'Error in Checkpoint Test: {error_string}',
            'type': 'stdout '
        })
        return
            
    while True:
        policy.reset()
        
        print(policy.config.output_features)
        
        # if temporal_agg:
        #     all_time_actions = torch.zeros([max_timesteps, max_timesteps+num_queries, state_dim]).cuda()
            
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
        
        ts = env.reset()

        timesteps = [ts]
        
        for t in range(max_timesteps):

            try:
                state = {}

                obs = ts.observation
                qpos_list = [item['qpos'] for item in obs['robot_states'].values()]
                qpos = np.concatenate(qpos_list)
                qpos = torch.from_numpy(qpos).float().cuda().unsqueeze(0)

                state['observation.state'] = qpos
                for sensor in sensors:
                    image = obs['images'][f'sensor_{sensor["id"]}']
                    image = fetch_image_with_config(image, {
                        'resize': task['sensor_img_size'],
                    })
                    image = image / 255.0
                    image = torch.from_numpy(image).float().cuda().unsqueeze(0)
                    image = rearrange(image, 'b h w c -> b c h w')
                    state[f'observation.images.sensor_{sensor["id"]}'] = image
                
                with torch.inference_mode():
                    action = policy.select_action(state)

                # Prepare the action for the environment
                action = action.squeeze(0).to("cpu").numpy()
                start_action_id = 0
                for agent in env.agents:
                    target_qpos = action[start_action_id:start_action_id + agent.joint_len]
                    print(f"To:{target_qpos} / Now:{qpos_list}")
                    start_action_id += agent.joint_len
                    agent.move_step(target_qpos)
                    
                    
                socketio_instance.emit('checkpoint_test_progress', {
                    'progress': (t+1) / max_timesteps,
                    'type': 'stdout '
                })

                time.sleep(0.1)  # Simulate processing time

                ts = env.record_step()
                timesteps.append(ts)
                
                if task_control['stop']:
                    return
            
            except Exception as e:
                import traceback
                error_string = traceback.format_exc()
                socketio_instance.emit('log_checkpoint_test', {
                    'log': f'Error in Checkpoint Test: {error_string}',
                    'type': 'stdout '
                })
                return

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
