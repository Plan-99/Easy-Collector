"""
This script serves as the main entry point for the failure prediction pipeline.

The pipeline consists of the following main components:

1. **TaskManager**:
   - Interfaces with task environments.
   - Initializes and manages the `ProcessedRolloutDataset` from raw rollouts.

2. **ProcessedRolloutDataset**:
   - Handles the data for evaluation, training, and results generation.
   - Provides utilities for loading, normalizing, and iterating over rollouts.

3. **RNDTrainer**:
   - Trains Random Network Distillation (RND) models for failure prediction.

4. **EvaluationManager**:
   - Interfaces with method-specific evaluation classes.
   - Evaluates failure prediction methods and generates metrics.

5. **ResultsManager**:
   - Combines evaluation results.
   - Creates summaries and visualizations of the results.

### Configuration:
- Pipeline settings: `/configs/default.yaml`
- Base Evaluation settings: `/configs/eval/base.yaml`
- Method-specific settings: `/configs/eval/{method}.yaml`
- Task-specific settings: `/configs/task/{task}.yaml`

Each stage can be executed independently or as part of the complete pipeline.
"""

import sys
import os
import pathlib
import hydra
from omegaconf import DictConfig, OmegaConf
import torch
import shutil

# Define paths relative to this file's location to ensure robustness.
FIPER_ROOT = "/root/src/backend/fiper"
BASE_CONFIG_PATH = FIPER_ROOT + "/configs"
BASE_DATA_PATH = FIPER_ROOT + "/data"
ORI_DATA_ROOT = "/root/src/backend/datasets"

from ..fiper.rnd import RNDTrainer
from ..fiper.evaluation import EvaluationManager, ResultsManager
from ..fiper.tasks import TaskManager

from ..fiper.shared_utils.hydra_utils import load_config
from ..fiper.shared_utils.utility_functions import get_required_tensors, set_seed
from ..fiper.shared_utils.embedding_helper import EmbeddingHelper


# Add the config directory to the path
def train_fiper(checkpoint, task_obj, chunk_size):
    
    task_id = str(task_obj["id"])
    create_dynamic_main_config(task_id)

    # Create the dynamic config file BEFORE hydra initializes
    create_dynamic_task_config(task_obj, chunk_size)

    with hydra.initialize(config_path="../fiper/configs", version_base="1.1", job_name="train_fiper"):
        
        # Compose config from the dynamically generated main config file
        cfg = hydra.compose(config_name=f"{task_id}")

        task = task_id
        tasks = [task]
        task_cfg = cfg.task # The task config is now nested under cfg.task

        embedding_helper = EmbeddingHelper(checkpoint["id"])

        rnd_models = cfg.get("rnd_models", [])
        methods = cfg.get("methods", [])
        methods.extend(rnd_models)


        # Logical combination of methods
        combine_methods = cfg.get("combine_methods", False)
        combined_methods = cfg.get("combined_methods", {})
        combined_methods = OmegaConf.to_container(combined_methods, resolve=True)
        cfg.combined_methods = combined_methods

        train_rnd = cfg.get("train_rnd", True)
        # Check if the pipeline inputs are valid
        check_inputs(
            cfg,
            tasks,
            methods,
            combined_methods,
            combine_methods,
            str(BASE_CONFIG_PATH),
        )
        # Check which tensors are required for the methods
        required_tensors, optional_tensors = get_required_tensors(methods, str(BASE_CONFIG_PATH))
        
        device = "cuda" if torch.cuda.is_available() else "cpu"

        # Get the random seeds for evaluation
        seed_list = cfg.eval.get("random_seeds", [0])
        seed = seed_list[0]
        
        set_seed(seed)
        
        task_data_path = os.path.join(BASE_DATA_PATH, task)

        # Pre-training cleanup: delete all folders in the task directory except 'rollouts'
        if os.path.exists(task_data_path):
            print(f"Cleaning up existing task directory: {task_data_path}")
            for item in os.listdir(task_data_path):
                item_path = os.path.join(task_data_path, item)
                if os.path.isdir(item_path) and item != 'rollouts':
                    print(f"  Deleting {item_path}")
                    shutil.rmtree(item_path)

        # Ensure calibration directory exists
        calibration_path = os.path.join(task_data_path, 'rollouts', 'calibration')
        print(f"Ensuring calibration directory exists: {calibration_path}")
        os.makedirs(calibration_path, exist_ok=True)
        
        ori_data_path = os.path.join(ORI_DATA_ROOT, "tmp") # os.path.join(ORIDATA_ROOT, task)
        # Initial Data Gathering and Processing
        taskmanager = TaskManager(
            task_cfg=cfg.task,
            eval_cfg=cfg.eval,
            task=task,
            base_config_path=str(BASE_CONFIG_PATH),
            task_data_path=task_data_path,
            ori_data_path=ori_data_path,
            embedding_helper=embedding_helper,
            required_tensors=required_tensors,
            optional_tensors=optional_tensors,
            device=device,
        )
        # Create or load or update the dataset
        dataset = taskmanager.get_rollout_dataset(
            load_dataset_if_exists=False,
        )

        # Train RND models if required
        if train_rnd:
            rndtrainer = RNDTrainer(str(BASE_CONFIG_PATH), task_data_path, dataset, device=device, task_cfg=task_cfg, seed=seed)
            rndtrainer.train(rnd_models)
        else:
            pass


        # Initialize the evaluation interface
        is_inference = False
        evaluationmanager = EvaluationManager(str(BASE_CONFIG_PATH), task_data_path, dataset, is_inference=is_inference, device=device, seed=seed)

        for method in methods:
            evaluationmanager.save_thresholds(method)

        # Evaluate the methods including the combined methods
        results = evaluationmanager.evaluate(methods, combine_methods, combined_methods)

        # Post-training cleanup: delete data in the calibration directory
        if os.path.exists(calibration_path):
            print(f"Cleaning up calibration data in: {calibration_path}")
            for item in os.listdir(calibration_path):
                item_path = os.path.join(calibration_path, item)
                try:
                    if os.path.isfile(item_path) or os.path.islink(item_path):
                        os.unlink(item_path)
                    elif os.path.isdir(item_path):
                        shutil.rmtree(item_path)
                except Exception as e:
                    print(f'Failed to delete {item_path}. Reason: {e}')

        return

def create_dynamic_main_config(task_id):
    template_path = os.path.join(FIPER_ROOT, "configs", "default.yaml")
    cfg = OmegaConf.load(template_path)

    # Modify defaults: find the 'task' key and update its value
    for default_item in cfg.defaults:
        if isinstance(default_item, DictConfig) and 'task' in default_item:
            default_item.task = task_id
            break
    
    # Modify tasks list
    cfg.tasks = [task_id]

    cfg.available_tasks = [task_id]

    # Save the new config
    output_path = os.path.join(FIPER_ROOT, "configs", f"{task_id}.yaml")
    OmegaConf.save(config=cfg, f=output_path)
    print(f"Generated dynamic main config at: {output_path}")

def create_dynamic_task_config(task_obj, chunk_size):
    task_id = str(task_obj["id"])
    
    # Load the template config from ec_test.yaml
    template_path = os.path.join(FIPER_ROOT, "configs", "task", "ec_test.yaml")
    cfg = OmegaConf.load(template_path)

    # Modify the config with dynamic values
    OmegaConf.set_struct(cfg, False)
    
    cfg.name = task_id
    cfg.environment.name = f"{task_id}_Env-v0"
    cfg.environment.max_episode_steps = task_obj["episode_len"]
    cfg.observation_space.rgb.shape = tuple(task_obj["sensor_settings"][0]["img_size"]) + (3,)
    cfg.state_space.state_dim = task_obj['joint_dim']
    cfg.state_space.state_mapping.position = list(range(task_obj['joint_dim']))
    cfg.action_space.actions.dim = task_obj['joint_dim']
    cfg.action_space.actions.action_mapping.position = list(range(task_obj['joint_dim']))
    cfg.action_space.action_pred.action_prediction_horizon = chunk_size
    
    # Save the new config file, overwriting if it exists
    output_path = os.path.join(FIPER_ROOT, "configs", "task", f"{task_id}.yaml")
    OmegaConf.save(config=cfg, f=output_path)
    print(f"Generated dynamic task config at: {output_path}")

def check_inputs(cfg: DictConfig, tasks, methods, combined_methods, combine_methods, base_config_path):
    """
    Check the inputs for the pipeline.
    """
    available_tasks = cfg.get("available_tasks", [])
    assert all(task in available_tasks for task in tasks)
    implemented_methods = cfg.get("implemented_methods", [])
    assert all(method in implemented_methods for method in methods)
    available_rnd_models = cfg.get("available_rnd_models", [])
    assert all(model in available_rnd_models for model in methods if model.startswith("rnd_"))
    if combine_methods:
        assert all(
            combined_methods[key]["m1"]["name"] in implemented_methods
            and combined_methods[key]["m1"]["name"] in methods
            and combined_methods[key]["m2"]["name"] in implemented_methods
            and combined_methods[key]["m2"]["name"] in methods
            and combined_methods[key]["operation"] in ["or", "and"]
            for key in combined_methods.keys()
        )

    # Check if the config files exist
    config_files = os.listdir(os.path.join(base_config_path, "eval"))
    assert all(f"{method}.yaml" in config_files for method in methods), (
        f"Some config files are missing for the methods: {methods}"
    )
    config_files = os.listdir(os.path.join(base_config_path, "task"))
    assert all(f"{task}.yaml" in config_files for task in tasks), (
        f"Some config files are missing for the tasks: {tasks}"
    )
    return