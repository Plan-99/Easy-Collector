import sys
import os
import pathlib
import hydra
from omegaconf import DictConfig
import torch
import pandas as pd
# Add the root directory to the path
ROOT_DIR = str(pathlib.Path(__file__).parent.parent)
sys.path.append(ROOT_DIR)
os.chdir(ROOT_DIR)

from tasks import TaskManager
from datasets import ProcessedRolloutDataset
from shared_utils.hydra_utils import load_config
from shared_utils.utility_functions import get_required_tensors



# Determine Config Path
base_config_path = os.path.join(ROOT_DIR, "configs")
base_data_path = os.path.join(ROOT_DIR, "data")

@hydra.main(config_path=base_config_path, config_name="default.yaml", version_base="1.1")
def main(cfg: DictConfig):
    # Tasks to be processed
    tasks = ["sorting", "stacking", "push_t", "pretzel", "push_chair", "ec_test"] # list(cfg.available_tasks)
    methods = list(cfg.implemented_methods)

    # tasks = ["push_chair"]


    # Check which tensors are required for the methods
    required_tensors, optional_tensors = get_required_tensors(methods, base_config_path)
    required_tensors.remove("rgb_images")
    device = "cuda" if torch.cuda.is_available() else "cpu"

    # Initialize an empty list to store dataset statistics
    dataset_statistics = []

    for task in tasks:
        # Load the complete config with only the task-specific config
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
        dataset:ProcessedRolloutDataset = taskmanager.get_rollout_dataset(
            load_dataset_if_exists=False,
        )
        # Get dataset statistics as a DataFrame row
        row = dataset.get_dataset_statistics(return_df=True)
        row["Task"] = task  # Add the task name as a column
        dataset_statistics.append(row)

    # Combine all rows into a single DataFrame
    statistics_df = pd.concat(dataset_statistics, ignore_index=True)
    column_order = ["Task"] + [col for col in statistics_df.columns if col != "Task"]
    statistics_df = statistics_df[column_order]
    filepath = os.path.join(base_data_path, "task_statistics.csv")
    statistics_df.to_csv(filepath, index=False)

    # Save the DataFrame to a CSV file (optional)
    # statistics_df.to_csv("dataset_statistics.csv", index=False)

    print("Dataset statistics collected for all tasks:")
    print(statistics_df)


if __name__ == "__main__":
    main()
