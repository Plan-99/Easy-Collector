import torch

import sys
import os
import numpy as np
from tqdm import tqdm
from copy import deepcopy
import argparse

# Import policies and utilities
from ..policies.policies import ACTPolicy
from ..policies.utils import make_policy, make_optimizer, forward_pass, detach_dict, compute_dict_mean, set_seed, load_data

# Import database and data models
import os
from ..database.db_init import get_db_connection
from ..database.models.robot_model import RobotModel
from ..database.models.policy_model import PolicyModel
from ..database.models.task_model import TaskModel
from ..database.models.gripper_model import GripperModel
from ..database.models.sensor_model import SensorModel
from ..database.models.checkpoint_model import CheckpointModel


def eval_bc(checkpoint_config, task_config, policy_config, robot_config, sensor_configs_ls):

    seed = 1
    set_seed(seed)
    # ckpt_dir = config['ckpt_dir']
    # state_dim = config['state_dim']
    # real_robot = config['real_robot']
    # policy_class = config['policy_class']
    # onscreen_render = config['onscreen_render']
    # policy_config = config['policy_config']
    # camera_names = config['camera_names']
    # max_timesteps = config['episode_len']
    # task_name = config['task_name']
    # dataset_dir = config['dataset_dir']
    # temporal_agg = config['temporal_agg']
    # onscreen_cam = 'angle'
    # home_pose = config['home_pose']
    # end_pose = config['end_pose']
    # pose_sleep = config['pose_sleep']
    # task_space = config['task_space']
    # vel_control = config['vel_control']

    # dataset_dir = f'{dataset_dir}/original'

    # yolo_config = None
    # if USE_YOLO:
    #     from ultralytics import YOLO

    #     yolo_config = {
    #         'model': YOLO('runs/detect/train3/weights/best.pt'),
    #         'conf': 0.7,
    #         'padding': 3
    #     }
    
    # load policy and stats
    ckpt_dir = checkpoint_config['dir']
    ckpt_path = os.path.join(ckpt_dir, checkpoint_config['file_name'])
    policy = make_policy(ckpt_path, seed, policy_config, task_config, robot_config, sensor_configs_ls)
    loading_status = policy.load_state_dict(torch.load(ckpt_path))
    print(loading_status)
    policy.cuda()
    policy.eval()
    print(f'Loaded: {ckpt_path}')
    stats_path = os.path.join(ckpt_dir, f'dataset_stats.pkl')
    quit()
    with open(stats_path, 'rb') as f:
        stats = pickle.load(f)

    pre_process = lambda s_qpos: (s_qpos - stats['qpos_mean']) / stats['qpos_std']
    post_process = lambda a: a * stats['action_std'] + stats['action_mean']


    # load environment
    # if real_robot:
    # from aloha_scripts.robot_utils import move_grippers # requires aloha
    from env import AlohaEnv
    import rclpy

    rclpy.init(args=None)
    node = rclpy.create_node("imitate_episode_node")
    env = AlohaEnv(camera_names, robot_name='yaskawa')
    env_max_reward = 0

    # else:
        # from sim_env import make_sim_env
        # env = make_sim_env(task_name)
        # env_max_reward = env.task.max_reward

    query_frequency = policy_config['num_queries']
    num_queries = 0
    if temporal_agg:
        query_frequency = 1
        num_queries = policy_config['num_queries']

    data_timesteps = max_timesteps
    max_timesteps = int(max_timesteps * 1.3) # may increase for real-world tasks

    num_rollouts = 100
    episode_returns = []
    highest_rewards = []
    for rollout_id in range(num_rollouts):

        rollout_id += 0

        # if 'sim_transfer_cube' in task_name:
        #     BOX_POSE[0] = sample_box_pose() # used in sim reset
        # elif 'sim_insertion' in task_name:
        #     BOX_POSE[0] = np.concatenate(sample_insertion_pose()) # used in sim reset

        # for i in range(3):
        print('go home pose!')
        for pos in home_pose:
            env.move_joint(pos)
            time.sleep(pose_sleep)

        ts = env.reset()
        timesteps = [ts]
        actions = []
        xactions = []
        
        ### evaluation loop
        if temporal_agg:
            all_time_actions = torch.zeros([max_timesteps, max_timesteps+num_queries, state_dim]).cuda() 

        rewards = []

        if onscreen_render:
            plt.ion()  # interactive mode on
            fig, ax = plt.subplots()


        print('move!')
        memories = [None] * len(camera_names)
        
        ts = env.reset()
        timesteps = [ts]
        actions = []
        
        ### evaluation loop
        if temporal_agg:
            all_time_actions = torch.zeros([max_timesteps, max_timesteps+num_queries, state_dim]).cuda()

        rewards = []
        # q_init = [TASK_CONFIGS[task_name]['home_pose'][0][:-1], TASK_CONFIGS[task_name]['home_pose'][0][:-1]]
        q_init = list(TASK_CONFIGS[task_name]['home_pose'][0][:-1])
        print("Start")

        stop = False
        for t in tqdm(range(max_timesteps)):
            # try:
            start = time.time()
            ### process previous timestep to get qpos and image_list
            timesteps.append(ts)
            obs = ts.observation
                
            cur_qpos = np.array(obs['qpos'])
            cur_xpos = np.array(obs['xpos'])
            cur_qvel = np.array(obs['qvel'])
            cur_xvel = np.array([0] * state_dim)

            if task_space:
                if vel_control:
                    # if t == 0:
                    #     pos_numpy = np.array([0] * state_dim)
                    # else:
                    #     pos_numpy = np.array(obs['xpos']) - np.array(last_xpos)
                    # last_xpos = obs['xpos']
                    robot_input_raw = cur_xvel
                else:
                    robot_input_raw = cur_xpos
            else:
                robot_input_raw = cur_qpos
            
            robot_input = pre_process(robot_input_raw)
            robot_input = torch.from_numpy(robot_input).float().cuda().unsqueeze(0)

            curr_image, memories = get_image(ts, camera_names, config['camera_config'], yolo_config, memories)
            
            with torch.inference_mode():
                if not stop:

                    ### query policy
                    if config['policy_class'] == "ACT":
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
                    elif config['policy_class'] == "CNNMLP":
                        raw_action = policy(robot_input, curr_image)
                    else:
                        raise NotImplementedError
            
            if not stop:
                if show_grad_cam:
                    policy.show_grad_cam_heatmap(curr_image, 3)
                ### post-process actions
                raw_action = raw_action.squeeze(0).cpu().numpy()
                action = post_process(raw_action)

                if task_space:
                    if vel_control:
                        target_xpos = cur_xpos + action
                        print(f"act:{action}")
                        print(f"target_xpos:{target_xpos}")
                    else:
                        target_xpos = action
                    q_init[0], q_init[2] = q_init[0], q_init[2]
                    target_qpos = xpos_to_qpos(target_xpos, kn, q_init)
                    print(f"target_qpos:{target_qpos}")
                else:
                    target_qpos = action
                    target_xpos = qpos_to_xpos(target_qpos, kn)
                # print(f"Get Action Time: {time.time() - start}")
                ### step the environment
                
                print(target_qpos)
                ts = env.move_step(target_qpos)
                
                # while True:
                #     qdiff_vec = target_qpos[:-1] - cur_qpos[:-1]
                #     qdiff = np.linalg.norm(qdiff_vec)
                #     # xdiff = np.linalg.norm(current_xpos[:-1] - target_xpos[:-1])
                    
                #     if qdiff < 5:
                #         q_init = list(cur_qpos[:-1])
                #         # print("it moved")
                #         print(target_qpos)
                #         ts = env.move_step(target_qpos)
                #         break
                #     else:
                #         target_qpos = np.concatenate((cur_qpos[:-1] + qdiff_vec * 0.5, target_qpos[-1:]))
                #         # print(target_qpos)
            else:
                # 정지
                while True:
                    key_pressed = tcp_ctr.decide_direction()
                    if (not tcp_ctr.controlling) or key_pressed:
                        break 
                    
                all_time_actions[:, :, :] = torch.zeros([max_timesteps, max_timesteps+num_queries, state_dim])
                ts = env.record_step()
                target_qpos = ts.observation['qpos']
                target_xpos = ts.observation['xpos']

                # # 그리퍼 캘리브레이션
                # gripper_pos = target_qpos[-1]
                # gripper_old_rng = (0.0, 0.786)
                # gripper_new_rng = (0.087, 0.001)
                # gripper_pos = gripper_new_rng[0] + (gripper_new_rng[1] - gripper_new_rng[0]) * ((gripper_pos - gripper_old_rng[0]) / (gripper_old_rng[1] - gripper_old_rng[0]))
                # target_qpos[-1] = gripper_pos
                # target_xpos[-1] = gripper_pos

            ### for visualization
            rewards.append(ts.reward)
            # print(f"Get Move Time: {time.time() - start}")

            actions.append(target_qpos)
            xactions.append(target_xpos)

            # except Exception as e:
            #     print("Error: %s" % e)
            #     return False
            
        plt.close()
        if real_robot:
            # move_grippers([env.puppet_bot_left, env.puppet_bot_right], [PUPPET_GRIPPER_JOINT_OPEN] * 2, move_time=0.5)  # open
            pass

        # if len(end_pose) > 0:
        #     for pos in end_pose:
        #         env.move_joint(pos)
        #         time.sleep(pose_sleep)

            
        rewards = np.array(rewards)
        episode_return = np.sum(rewards[rewards!=None])
        episode_returns.append(episode_return)
        episode_highest_reward = np.max(rewards)
        highest_rewards.append(episode_highest_reward)
        print(f'Rollout {rollout_id}\n{episode_return=}, {episode_highest_reward=}, {env_max_reward=}, Success: {episode_highest_reward==env_max_reward}')

        if record_episode:
            d = input("Will you save this data? (Input 'y' for yes)")
            if len(d) > 0 and d[-1] == 'y':
                record_eval_episode(timesteps, actions, xactions, camera_names, config['camera_config'], data_timesteps, state_dim, dataset_dir, yolo_config)
            else:
                d = input("Will you record new data? (Input 'y' for yes)")
                if d == 'y':
                    record_new_episode(task_name)


    success_rate = np.mean(np.array(highest_rewards) == env_max_reward)
    avg_return = np.mean(episode_returns)
    summary_str = f'\nSuccess rate: {success_rate}\nAverage return: {avg_return}\n\n'
    for r in range(env_max_reward+1):
        more_or_equal_r = (np.array(highest_rewards) >= r).sum()
        more_or_equal_r_rate = more_or_equal_r / num_rollouts
        summary_str += f'Reward >= {r}: {more_or_equal_r}/{num_rollouts} = {more_or_equal_r_rate*100}%\n'

    print(summary_str)

    # save success rate to txt
    result_file_name = 'result_' + ckpt_name.split('.')[0] + '.txt'
    with open(os.path.join(ckpt_dir, result_file_name), 'w') as f:
        f.write(summary_str)
        f.write(repr(episode_returns))
        f.write('\n\n')
        f.write(repr(highest_rewards))

    return success_rate, avg_return



def main(args):
    """Main execution function to load configs and start training."""
    
    # Connect to the database
    conn = get_db_connection()
    cursor = conn.cursor()
    
    # Fetch configurations from the database
    checkpoint_config = vars(CheckpointModel.find_one({'id': args['checkpoint_id']}))['args']
    task_config = vars(TaskModel.find_one({'id': checkpoint_config['task_id']}))['args']
    policy_config = vars(PolicyModel.find_one({'id': checkpoint_config['policy_id']}))['args']
    robot_config = vars(RobotModel.find_one({'id': task_config['robot_id']}))['args']
    sensor_configs_ls = [vars(SensorModel.find_one({'id': sid}))['args'] for sid in task_config['sensor_ids']]
        
    
    results = []
    success_rate, avg_return = eval_bc(checkpoint_config, task_config, policy_config, robot_config, sensor_configs_ls)
    results.append([ckpt_name, success_rate, avg_return])

    for ckpt_name, success_rate, avg_return in results:
        print(f'{ckpt_name}: {success_rate=} {avg_return=}')
    exit()
    
    
if __name__ == '__main__':
    
    parser = argparse.ArgumentParser()
    parser.add_argument('--checkpoint_id', default=None, required=True)
    
    main(vars(parser.parse_args()))
    sys.exit(0)