import torch

import sys
import os
import numpy as np
from tqdm import tqdm
from copy import deepcopy
import argparse
import json
import shutil
import pickle

# Import policies and utilities
from ..policies.utils import make_policy, make_optimizer, forward_pass, detach_dict, compute_dict_mean, set_seed, load_data

# Import database and data models
from orator import DatabaseManager
from orator.orm import Model
from ..database.models.robot_model import Robot
from ..database.models.policy_model import Policy
from ..database.models.task_model import Task
from ..database.models.gripper_model import Gripper
from ..database.models.sensor_model import Sensor
from ..database.models.checkpoint_model import Checkpoint


def train(
    train_dataloader, 
    val_dataloader, 
    ckpt_dir,
    num_epochs,
    learning_rate,
    lr_backbone,
    stats,
    task,
    policy_obj,
    robot,
    sensors,
    gripper=None,
    load_model=None
    ):
    """Function to train the policy model."""
    seed = 100
    
    policy_class = policy_obj['type']
    load_model_path = f"/root/src/backend/checkpoints/{load_model['id']}" if load_model else None

    set_seed(seed) # Set seed for reproducibility

    # Create policy model

    policy = make_policy(ckpt_dir, seed, learning_rate, lr_backbone, policy_obj, task, robot, sensors, gripper)
    if load_model_path is not None:
        model_path = os.path.join(load_model_path, 'policy_best.ckpt')
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
        print(f'\nEpoch {epoch} / {num_epochs}')
        
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
        for batch_idx, data in enumerate(train_dataloader):
            optimizer.zero_grad()
            forward_dict = forward_pass(data, policy)
            # Backpropagation
            loss = forward_dict['loss']
            loss.backward()
            optimizer.step()
            
            train_history.append(detach_dict(forward_dict))
            
        epoch_summary = compute_dict_mean(train_history[(batch_idx+1)*epoch:(batch_idx+1)*(epoch+1)])
        print(f'Train loss: {epoch_summary["loss"]:.5f}')
        
        # save dataset stats
    if not os.path.isdir(ckpt_dir):
        os.makedirs(ckpt_dir)
        print("Folder created:", ckpt_dir)
    stats_path = os.path.join(ckpt_dir, f'dataset_stats.pkl')
    with open(stats_path, 'wb') as f:
        pickle.dump(stats, f)

    # Save the best checkpoint after training is finished
    best_epoch, min_val_loss, best_state_dict = best_ckpt_info

    ckpt_path = os.path.join(ckpt_dir, f'policy_epoch_{best_epoch}_seed_{seed}.ckpt')
    torch.save(best_state_dict, ckpt_path)
    ckpt_path = os.path.join(ckpt_dir, f'policy_best.ckpt')
    torch.save(best_state_dict, ckpt_path)
    
    print(f'Training finished:\nSeed {seed}, val loss {min_val_loss:.6f} at epoch {best_epoch}')
    

    return best_ckpt_info


def main(args):
    """Main execution function to load configs and start training."""
    
    # Setup database connection using Orator
    db_path = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'database', 'main.db'))
    config = {
        'sqlite': {
            'driver': 'sqlite',
            'database': db_path,
        }
    }
    db = DatabaseManager(config)
    Model.set_connection_resolver(db)
    
    # Create a temporary directory to store dataset files
    temp_dir = "/root/src/backend/datasets/tmp"
    if os.path.exists(temp_dir):
        shutil.rmtree(temp_dir)
    os.makedirs(temp_dir)
    
    try:
        # Copy dataset files to the temporary directory
        dataset_ids = json.loads(args.dataset_ids)
        episode_counter = 0
        for ds_id in dataset_ids:
            dataset_path = f"/root/src/backend/datasets/{ds_id}"
            
            episode_files = [f for f in os.listdir(dataset_path) if f.startswith('episode_') and os.path.isfile(os.path.join(dataset_path, f))]

            for ep_file in episode_files:
                src_ep_path = os.path.join(dataset_path, ep_file)
                dest_ep_path = os.path.join(temp_dir, f'episode_{episode_counter}.hdf5')
                shutil.copy(src_ep_path, dest_ep_path)
                episode_counter += 1
                print("Copied episode file:", ep_file, "to", dest_ep_path)

        # Fetch configurations from the database
        task = Task.find(args.task_id).to_dict()
        policy = Policy.find(args.policy_id).to_dict()
        robot = Robot.find(task['robot_ids'][0]).to_dict() # Assuming first robot
        sensors = [Sensor.find(sid).to_dict() for sid in task['sensor_ids']]
        # gripper = Gripper.find(robot.gripper_id) if robot.gripper_id else None
        load_model = Checkpoint.find(args.load_model_id).to_dict() if args.load_model_id else None
        sensor_ids = [sensor['id'] for sensor in sensors]
        
        num_epochs = int(args.num_epochs)
        batch_size = int(args.batch_size)
        
        learning_rate = float(args.learning_rate)
        lr_backbone = float(args.lr_backbone)
        
        
        # Load data from the temporary directory
        train_dataloader, val_dataloader, stats, _ = load_data(temp_dir, episode_counter, sensor_ids, batch_size, batch_size)

        ckpt_dir = f"/root/src/backend/checkpoints/{args.checkpoint_id}"
        # Start the training process
        best_ckpt_info = train(
            train_dataloader,
            val_dataloader,
            ckpt_dir,
            num_epochs,
            learning_rate,
            lr_backbone,
            stats,
            task,
            policy,
            robot,
            sensors,
            gripper=None,
            load_model=load_model)
        
        Checkpoint.find(args.checkpoint_id).update({
            'is_training': False
        })
        
    except Exception as e:
        Checkpoint.find(args.checkpoint_id).delete()
        raise e
        
    finally:
        # Clean up the temporary directory
        shutil.rmtree(temp_dir)
    
    
# Script entry point
if __name__ == '__main__':
    
    parser = argparse.ArgumentParser()
    parser.add_argument('--task_id', required=True)
    parser.add_argument('--policy_id', required=True)
    parser.add_argument('--load_model_id', default=None, required=False)
    parser.add_argument('--checkpoint_id', required=True)
    parser.add_argument('--dataset_ids', required=True)
    parser.add_argument('--num_epochs', required=True)
    parser.add_argument('--batch_size', required=True)
    parser.add_argument('--learning_rate', default=1e-5, required=False)
    parser.add_argument('--lr_backbone', default=1e-6, required=False)

    main(parser.parse_args())
    sys.exit(0)
