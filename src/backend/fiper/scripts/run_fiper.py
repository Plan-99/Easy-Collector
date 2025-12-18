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

# Add the root directory to the path
ROOT_DIR = str(pathlib.Path(__file__).parent.parent)
sys.path.append(ROOT_DIR)
os.chdir(ROOT_DIR)


from rnd import RNDTrainer
from evaluation import EvaluationManager, ResultsManager
from tasks import TaskManager

from shared_utils.hydra_utils import load_config
from shared_utils.utility_functions import get_required_tensors, set_seed


# Determine Config Path
base_config_path = os.path.join(ROOT_DIR, "configs")
base_data_path = os.path.join(ROOT_DIR, "data")


# Add the config directory to the path
@hydra.main(config_path=base_config_path, config_name="default.yaml", version_base="1.1")
def main(cfg: DictConfig):
    # Tasks to be processed
    tasks = cfg.get("tasks", [])
    # Methods to be evaluated
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
        base_config_path,
    )
    # Check which tensors are required for the methods
    required_tensors, optional_tensors = get_required_tensors(methods, base_config_path)
    
    device = "cuda" if torch.cuda.is_available() else "cpu"

    # Get the random seeds for evaluation
    seed_list = cfg.eval.get("random_seeds", [0, 1, 2])  #
    all_seed_results = []
    for seed in seed_list:
        set_seed(seed)
        print(f"-------------- Seed: {seed} ----------------")
        total_results = {}
        for task in tasks:
            print(f"-------------- Task: {task} ----------------")
            # Load the complete config with only the task overridden
            cfg = load_config("task", task, return_only_subdict=False)

            task_data_path = os.path.join(base_data_path, task)
            # Initial Data Gathering and Processing
            taskmanager = TaskManager(
                cfg,
                task,
                base_config_path,
                task_data_path,
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
                rndtrainer = RNDTrainer(base_config_path, task_data_path, dataset, device=device, task_cfg=cfg, seed=seed)
                rndtrainer.train(rnd_models)
            else:
                pass

            # Initialize the evaluation interface
            evaluationmanager = EvaluationManager(base_config_path, task_data_path, dataset, device=device, seed=seed)
            # Evaluate the methods including the combined methods
            results = evaluationmanager.evaluate(methods, combine_methods, combined_methods)
            # Save the results for the task
            total_results[task] = results

        all_seed_results.append(total_results.copy())

    # Create the results manager
    resultsmanager = ResultsManager(base_config_path, base_data_path)

    # Combine the results from all seeds ()
    total_results = resultsmanager.accumulate_seed_results(all_seed_results)

    # Extract the method names from the results (required due to combined methods)
    method_names = [method for method in total_results[task].keys()]
    
    # Update the saved results with the new results
    resultsmanager.combine_results(total_results, method_names=method_names)
    # Summarize the results as defined in results config. Saves the summary(ies) as csv file(s).
    resultsmanager.create_summary()
    # # Generate uncertainty plots for the results as defined in results config. Saves the plots as pdf files.
    # resultsmanager.generate_uncertainty_plots()


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


if __name__ == "__main__":
    main()
