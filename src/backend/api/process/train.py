import torch

import os
import numpy as np
from copy import deepcopy
import shutil
import pickle

# Import policies and utilities
from ...policies.utils import make_policy, make_optimizer, forward_pass, detach_dict, compute_dict_mean, set_seed, load_data

# Import database and data models
from ...database.models.robot_model import Robot
from ...database.models.policy_model import Policy
from ...database.models.task_model import Task
from ...database.models.gripper_model import Gripper
from ...database.models.sensor_model import Sensor
from ...database.models.checkpoint_model import Checkpoint


def train_task(task_id, policy_id, dataset_ids, num_epochs, batch_size, checkpoint_id, load_model_id, socketio_instance, task_control):
    # Create a temporary directory to store dataset files
    temp_dir = "/root/src/backend/datasets/tmp"
    if os.path.exists(temp_dir):
        shutil.rmtree(temp_dir)
    os.makedirs(temp_dir)
    
    try:
        # Copy dataset files to the temporary directory
        episode_counter = 0

        for ds_id in dataset_ids:
            dataset_path = f"/root/src/backend/datasets/{ds_id}"
            
            episode_files = [f for f in os.listdir(dataset_path) if f.startswith('episode_') and os.path.isfile(os.path.join(dataset_path, f))]

            for ep_file in episode_files:
                src_ep_path = os.path.join(dataset_path, ep_file)
                dest_ep_path = os.path.join(temp_dir, f'episode_{episode_counter}.hdf5')
                shutil.copy(src_ep_path, dest_ep_path)
                episode_counter += 1
                socketio_instance.emit(f'log_train_task', {
                    'log': f"Copied episode file: {ep_file} to {dest_ep_path}",
                    'type': 'stdout '
                })

        # Fetch configurations from the database
        task = Task.find(task_id).to_dict()
        policy = Policy.find(policy_id).to_dict()
        robot = Robot.find(task['robot_ids'][0]).to_dict() # Assuming first robot
        sensors = [Sensor.find(sid).to_dict() for sid in task['sensor_ids']]
        gripper = Gripper.find(robot['gripper_id']).to_dict() if 'gripper_id' in robot else None
        load_model = Checkpoint.find(load_model_id).to_dict() if load_model_id else None
        
        # Get parameters from configs
        sensor_ids = [sensor['id'] for sensor in sensors]
        
        num_epochs = int(num_epochs)
        batch_size = int(batch_size)
        
        # Load data from the temporary directory
        train_dataloader, val_dataloader, stats, _ = load_data(temp_dir, episode_counter, sensor_ids, batch_size, batch_size)

        ckpt_dir = f"/root/src/backend/checkpoints/{checkpoint_id}"
        # Start the training process
        """Function to train the policy model."""
        seed = 1
        
        policy_class = policy['type']
        load_model_path = f"/root/src/backend/checkpoints/{load_model.id}" if load_model else None

        set_seed(seed) # Set seed for reproducibility

        # Create policy model
        policy = make_policy(ckpt_dir, seed, policy, task, robot, sensors, gripper)

        # Load pre-trained model for fine-tuning
        if load_model_path is not None:
            model_path = os.path.join(load_model_path, 'policy_best.ckpt')
            loading_status = policy.load_state_dict(torch.load(model_path))
            socketio_instance.emit(f'log_train_task', {
                'log': f'Loaded Policy from {model_path}',
                'type': 'stdout '
            })

            
        policy.cuda() # Move model to GPU
        optimizer = make_optimizer(policy_class, policy) # Create optimizer

        train_history = []
        validation_history = []
        min_val_loss = np.inf
        best_ckpt_info = None

        # Main training loop
        for epoch in range(num_epochs):
            socketio_instance.emit(f'log_train_task', {
                'log': f'Epoch {epoch}',
                'type': 'stdout '
            })
                
            print('aaaa')
            # --- Validation Step ---
            with torch.inference_mode():
                print('bbbb')
                policy.eval() # Set model to evaluation mode
                print('cccc')

                epoch_dicts = []
                for batch_idx, data in enumerate(val_dataloader):
                    print('dddd')
                    forward_dict = forward_pass(data, policy)
                    epoch_dicts.append(forward_dict)
                epoch_summary = compute_dict_mean(epoch_dicts)
                validation_history.append(epoch_summary)
                # Save the best model checkpoint based on validation loss
                epoch_val_loss = epoch_summary['loss']
                if epoch_val_loss < min_val_loss:
                    min_val_loss = epoch_val_loss
                    best_ckpt_info = (epoch, min_val_loss, deepcopy(policy.state_dict()))
                print('eeee')
            socketio_instance.emit(f'log_train_task', {
                'log': f'Validation loss: {epoch_val_loss:.5f}',
                'type': 'stdout '
            })

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
            socketio_instance.emit(f'log_train_task', {
                'log': f'Train loss: {epoch_summary["loss"]:.5f}',
                'type': 'stdout '
            })

            if task_control['stop']:
                socketio_instance.emit(f'log_train_task', {
                    'log': 'Training stopped by user.',
                    'type': 'stdout '
                })
                return
            
            # save dataset stats
        if not os.path.isdir(ckpt_dir):
            os.makedirs(ckpt_dir)
            socketio_instance.emit(f'log_train_task', {
                'log': f"Folder created: {ckpt_dir}",
                'type': 'stdout '
            })
        stats_path = os.path.join(ckpt_dir, f'dataset_stats.pkl')
        with open(stats_path, 'wb') as f:
            pickle.dump(stats, f)

        # Save the best checkpoint after training is finished
        best_epoch, min_val_loss, best_state_dict = best_ckpt_info

        ckpt_path = os.path.join(ckpt_dir, f'policy_epoch_{best_epoch}_seed_{seed}.ckpt')
        torch.save(best_state_dict, ckpt_path)
        ckpt_path = os.path.join(ckpt_dir, f'policy_best.ckpt')
        torch.save(best_state_dict, ckpt_path)
        
        socketio_instance.emit(f'log_train_task', {
            'log': f'Training finished:\nSeed {seed}, val loss {min_val_loss:.6f} at epoch {best_epoch}',
            'type': 'stdout '
        })
        

        return best_ckpt_info
        
    finally:
        # Clean up the temporary directory
        shutil.rmtree(temp_dir)
    