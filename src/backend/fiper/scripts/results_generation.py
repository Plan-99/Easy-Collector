"""This script is for showing results and creating summaries.

The results/summaries creation is controlled by the "results" config file in "configs/eval/" and optionally by keword arguments to the "create_summary" function which override the config/
"""

import sys
import os
import pathlib
import hydra
from omegaconf import DictConfig

# Add the root directory to the path
ROOT_DIR = str(pathlib.Path(__file__).parent.parent)
sys.path.append(ROOT_DIR)
os.chdir(ROOT_DIR)

from evaluation import ResultsManager


# Determine Config Path
base_config_path = os.path.join(ROOT_DIR, "configs")
base_data_path = os.path.join(ROOT_DIR, "data")


# Add the config directory to the path
@hydra.main(config_path=base_config_path, config_name="default.yaml", version_base="1.1")
def main(cfg: DictConfig):
    # kwargs can be used to override the config file (not recommended)
    kwargs = {}

    # Create the results manager
    resultsmanager = ResultsManager(base_config_path, base_data_path)
    # Summarize and visualize results as defined in results config. Saves the summary(ies) as csv file(s).
    resultsmanager.create_summary(**kwargs)
    resultsmanager.extract_warning_frames(**kwargs)
    resultsmanager.generate_uncertainty_plots(**kwargs)
    resultsmanager.plot_quantile_impact(**kwargs)
    resultsmanager.plot_window_impact(**kwargs)
    resultsmanager.plot_threshold_impact(**kwargs)
    resultsmanager.plot_rollout_type_stats(**kwargs)


if __name__ == "__main__":
    main()
