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


def train(train_dataloader, val_dataloader, ckpt_dir, task_config, policy_config, robot_config, sensor_configs_ls, checkpoint_config=None):
    """Function to train the policy model."""
    seed = 1
    
    # Set training parameters
    num_epochs = policy_config['num_epochs']
    policy_class = policy_config['type']
    load_model = checkpoint_config['path'] if checkpoint_config else None

    set_seed(seed) # Set seed for reproducibility

    # Create policy model
    policy = make_policy(ckpt_dir, seed, policy_config, task_config, robot_config, sensor_configs_ls)

    # Load pre-trained model for fine-tuning
    if load_model is not None:
        model_path = os.path.join(load_model, 'policy_best.ckpt')
        loading_status = policy.load_state_dict(torch.load(model_path))
        print(loading_status)
        
    policy.cuda() # Move model to GPU
    optimizer = make_optimizer(policy_class, policy) # Create optimizer

    train_history = []
    validation_history = []
    min_val_loss = np.inf
    best_ckpt_info = None

    # Main training loop
    for epoch in tqdm(range(num_epochs)):
        print(f'\nEpoch {epoch}')
        
        # --- Validation Step ---
        with torch.inference_mode():
            policy.eval() # Set model to evaluation mode
            epoch_dicts = []
            for batch_idx, data in enumerate(val_dataloader):
                forward_dict = forward_pass(data, policy)
                epoch_dicts.append(forward_dict)
            epoch_summary = compute_dict_mean(epoch_dicts)
            validation_history.append(epoch_summary)

            # Save the best model checkpoint based on validation loss
            epoch_val_loss = epoch_summary['loss']
            if epoch_val_loss < min_val_loss:
                min_val_loss = epoch_val_loss
                best_ckpt_info = (epoch, min_val_loss, deepcopy(policy.state_dict()))
        print(f'Val loss:   {epoch_val_loss:.5f}')

        # --- Training Step ---
        policy.train() # Set model to training mode
        optimizer.zero_grad()
        for batch_idx, data in enumerate(train_dataloader):
            forward_dict = forward_pass(data, policy)
            
            # Backpropagation
            loss = forward_dict['loss']
            loss.backward()
            optimizer.step()
            optimizer.zero_grad()
            train_history.append(detach_dict(forward_dict))
            
        epoch_summary = compute_dict_mean(train_history[(batch_idx+1)*epoch:(batch_idx+1)*(epoch+1)])
        print(f'Train loss: {epoch_summary["loss"]:.5f}')

    # Save the best checkpoint after training is finished
    best_epoch, min_val_loss, best_state_dict = best_ckpt_info
    ckpt_path = os.path.join(ckpt_dir, f'policy_epoch_{best_epoch}_seed_{seed}.ckpt')
    torch.save(best_state_dict, ckpt_path)
    print(f'Training finished:\nSeed {seed}, val loss {min_val_loss:.6f} at epoch {best_epoch}')

    return best_ckpt_info


def main(args):
    """Main execution function to load configs and start training."""
    
    # Connect to the database
    conn = get_db_connection()
    cursor = conn.cursor()
    
    # Fetch configurations from the database
    task_config = vars(TaskModel.find_one({'id': args['task_id']}))['args']
    policy_config = vars(PolicyModel.find_one({'id': task_config['policy_id']}))['args']
    if args['checkpoint_id'] is not None:
        checkpoint_config = vars(CheckpointModel.find_one({'id': args['checkpoint_id']}))['args']
    else:
        checkpoint_config = None
    robot_config = vars(RobotModel.find_one({'id': args['policy_id']}))['args']
    sensor_configs_ls = [vars(SensorModel.find_one({'id': sid}))['args'] for sid in task_config['sensor_ids']]
        
    # Get parameters from configs
    dataset_dir = task_config['dataset_dir']
    sensor_names = [sensor['name'] for sensor in sensor_configs_ls]
    batch_size = policy_config['batch_size']
    
    ckpt_dir = "root/src/backend/policies/act/checkpoints/pick_can1"
    
    # Load data
    num_episodes = 3
    train_dataloader, val_dataloader, stats, _ = load_data(f"{dataset_dir}/tmp", num_episodes, sensor_names, batch_size, batch_size)

    # Start the training process
    best_ckpt_info = train(train_dataloader, val_dataloader, ckpt_dir, task_config, policy_config, robot_config, sensor_configs_ls)
    
    
# Script entry point
if __name__ == '__main__':
    
    parser = argparse.ArgumentParser()
    parser.add_argument('--task_id', required=True)
    parser.add_argument('--policy_id', required=True)
    parser.add_argument('--checkpoint_id', default=None, required=False)
    
    main(vars(parser.parse_args()))
    sys.exit(0)