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
from ..policies.utils import make_policy, make_optimizer, forward_pass, detach_dict, compute_dict_mean, set_seed, load_data, convert_lists_to_tuples

# Import database and data models
from orator import DatabaseManager
from orator.orm import Model
from ..database.models.robot_model import Robot
from ..database.models.policy_model import Policy
from ..database.models.task_model import Task
from ..database.models.gripper_model import Gripper
from ..database.models.sensor_model import Sensor
from ..database.models.checkpoint_model import Checkpoint

from ..lerobot.policies.act.configuration_act import ACTConfig
from ..lerobot.policies.act.modeling_act import ACTPolicy
from ..lerobot.policies.diffusion.configuration_diffusion import DiffusionConfig
from ..lerobot.policies.diffusion.modeling_diffusion import DiffusionPolicy
from ..lerobot.datasets.utils import dataset_to_policy_features
from ..lerobot.configs.types import FeatureType
from safetensors.torch import load_file


def train(
    train_dataloader,
    val_dataloader,
    input_features,
    output_features,
    stats,
    policy_obj,
    checkpoint_obj,
    load_model=None,
    ):
    """Function to train the policy model."""
    seed = 100
    
    policy_settings = convert_lists_to_tuples(policy_obj['settings'])
    train_settings = convert_lists_to_tuples(checkpoint_obj['train_settings'])
    load_model_path = f"/root/src/backend/checkpoints/{load_model['id']}" if load_model else None

    set_seed(seed) # Set seed for reproducibility

    num_epochs = train_settings['num_epochs']
    del train_settings['num_epochs'] # Remove num_epochs from train_settings
    del train_settings['batch_size'] # Remove batch_size from train_settings
    del train_settings['num_workers'] # Remove num_workers from train_settings

    
    if policy_obj['type'] == 'ACT':
        # Create policy model
        cfg = ACTConfig(
            input_features=input_features,
            output_features=output_features,
            **policy_settings,
            **train_settings,
        )
        policy = ACTPolicy(cfg, dataset_stats=stats)
    elif policy_obj['type'] == 'Diffusion':
        
        cfg = DiffusionConfig(
            input_features=input_features,
            output_features=output_features,
            **policy_settings,
            **train_settings,
        )
        policy = DiffusionPolicy(cfg, dataset_stats=stats)
    

    policy.train()
    policy.cuda()

    if load_model_path is not None:
        model_path = os.path.join(load_model_path, 'model.safetensors')
        state_dict = load_file(model_path, device='cuda')
        policy.load_state_dict(state_dict)
        
    policy.cuda() # Move model to GPU

    optimizer = torch.optim.Adam(policy.parameters(), lr=cfg.optimizer_lr)

    train_history = []
    validation_history = []
    min_val_loss = np.inf
    best_ckpt_info = None

    # Main training loop
    for epoch in range(num_epochs):
        
        # --- Validation Step ---
        with torch.inference_mode():
            policy.eval() # Set model to evaluation mode
            
            val_loss_sum = 0
            for batch_idx, data in enumerate(val_dataloader):
                loss, _ = forward_pass(data, policy)
                val_loss_sum += loss.item()

            epoch_val_loss = val_loss_sum / (batch_idx + 1)
            validation_history.append(epoch_val_loss)

            if epoch_val_loss < min_val_loss:
                min_val_loss = epoch_val_loss
                best_ckpt_info = (epoch, min_val_loss, deepcopy(policy))

        # --- Training Step ---
        policy.train() # Set model to training mode
        for batch_idx, data in enumerate(train_dataloader):
            optimizer.zero_grad()
            loss, _ = forward_pass(data, policy)
            # Backpropagation
            loss.backward()
            optimizer.step()
            
            train_history.append({'loss': loss.item()})
            
        epoch_summary = compute_dict_mean(train_history[(batch_idx+1)*epoch:(batch_idx+1)*(epoch+1)])
        train_log = {
            'epoch': epoch,
            'total_epoch': num_epochs,
            'val_loss': epoch_val_loss,
            'train_loss': epoch_summary["loss"]
        }
        print(f"[TRAIN_LOG] {json.dumps(train_log)}")

        
    # save dataset stats
    ckpt_dir = f"/root/src/backend/checkpoints/{checkpoint_obj['id']}"

    # Save the best checkpoint after training is finished
    best_epoch, min_val_loss, best_policy = best_ckpt_info

    # ckpt_path = os.path.join(ckpt_dir, f'policy_best.ckpt')
    best_policy.save_pretrained(ckpt_dir)
    stats_path = os.path.join(ckpt_dir, f'dataset_stats.pkl')
    with open(stats_path, 'wb') as f:
        pickle.dump(stats, f)

    # torch.save(best_state_dict, ckpt_path)
    
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
        checkpoint = Checkpoint.find(args.checkpoint_id).to_dict()
        load_model = Checkpoint.find(args.load_model_id).to_dict() if args.load_model_id else None

        batch_size = checkpoint['train_settings']['batch_size']
        sensor_ids = task['sensor_ids']
        if policy['type'] in ['ACT']:
            chunk_size = policy['settings']['chunk_size']
        elif policy['type'] in ['Diffusion']:
            chunk_size = policy['settings']['horizon']
        num_workers = checkpoint['train_settings']['num_workers']
        n_obs_steps = policy['settings']['n_obs_steps']  # Default to 1 if not specified
        
        # Load data from the temporary directory
        train_dataloader, val_dataloader, stats, input_features, output_features = load_data(temp_dir, episode_counter, sensor_ids, batch_size, batch_size, chunk_size, num_workers, n_obs_steps)

        # Start the training process
        best_epoch, min_val_loss, best_state_dict = train(
            train_dataloader,
            val_dataloader,
            input_features,
            output_features,
            stats,
            policy,
            checkpoint,
            load_model=load_model,
        )
        
        Checkpoint.find(args.checkpoint_id).update({
            'is_training': False,
            'best_epoch': best_epoch,
            'loss': min_val_loss,
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
    
    # # Add arguments for training parameters
    # # This is a bit of a hack to get all the training parameters from the command line
    # # A better way would be to pass a config file
    # for key, value in ACTConfig.model_fields.items():
    #     if key not in ['input_features', 'output_features']:
    #         parser.add_argument(f'--{key}', type=type(value.default), default=value.default)

    main(parser.parse_args())
    sys.exit(0)
