import os
import pickle
from abc import ABC, abstractmethod
from pathlib import Path
from typing import Union
import time
import numpy as np
from omegaconf import DictConfig
from evaluation.utils import (
    calculate_metrics,
    compute_thresholds,
    normalize_scores_by_threshold,
)
from datasets.rollout_datasets import ProcessedRolloutDataset


class BaseEvalClass(ABC):
    """
    Abstract base class that provides the common functionality of the evaluation methods.
    It handles the treshold calculation via calibration rollouts, method evaluation via test rollouts, and metrics calculation.
    Subclasses (one for each method) must only implement the following methods:
    - calculate_uncertainty_score
    When loading a model, the subclass must also implement the load_model method.
    When preprocessing based on calibration and/or test rollouts is required, the subclass must implement the _execute_preprocessing method.

    """

    def __init__(
        self,
        cfg: DictConfig,
        method_name: str,
        device: str,
        task_data_path: Union[str, Path],
        dataset: ProcessedRolloutDataset,
        **kwargs,
    ):
        """
        Initialize the base evaluation class.

        Args:
            method_name (str): Name of the evaluation method.
            cfg (dict): Configuration dictionary or object.
            policy (BaseImagePolicy): Trained diffusion policy.
            device (str): Device to use for evaluation.
            task_data_path (str): Base path to the data "data_all/{task}.
        """
        self.method_name = method_name
        self.cfg = cfg
        self.device = device
        self.task_data_path = task_data_path
        self.dataset = dataset
        self.required_tensors = list(cfg.get("required_tensors", []))
        self.optional_tensors = list(cfg.get("optional_tensors", []))
        self.required_actions = list(cfg.get("required_actions", []))
        self.optional_actions = list(cfg.get("optional_actions", []))
        self.normalize_tensors = dict(cfg.get("normalize_tensors", {}))
        # Create the results directory
        self.results_dir = os.path.join(task_data_path, "results", method_name)
        os.makedirs(self.results_dir, exist_ok=True)
        # Load the model (must be implemented in the subclass, not all methods require a model)
        self.load_model()

    @abstractmethod
    def calculate_uncertainty_score(self, rollout_tensor_dict: dict, **kwargs) -> float:
        """
        Calculate the uncertainty score for a rollout step and the inference time in doing so. Must be implemented in the subclass.

        Args:
            rollout_step: Dict that includes the requested tensors for one step of the rollout.

        Returns:
            uncertainty_score : Uncertainty score for one step.
        """
        pass

    def load_model(self):
        """
        Loads models, modules that are required for the evaluation. Optional, not all methods require a model.

        Returns:
            Nothing. Main model is stored in self.model, other models as other class attributes.
        """
        self.model = None

    def _execute_preprocessing(self):
        """Placeholder for preprocessing steps that are required before the evaluation.
        Should be implemented in the subclass if needed. The method can use the dataset class to access the data."""
        pass

    def _compound_uncertainty_scores(self, score_list: list) -> float:
        """Compunds multiple uncertainty scores if a rollout is recorded with multiple robots.

        Args:
            score_list: List of uncertainty scores. For each robot/arm, there is one score.
        Returns:
            uncertainty_score: Compounded uncertainty score.
        """
        assert len(score_list) >= 1
        if len(score_list) == 1:
            return score_list[-1]

        if self.cfg.score_compounding_method == "mean":
            return np.mean(score_list)
        if self.cfg.score_compounding_method == "max":
            return np.max(score_list)
        if self.cfg.score_compounding_method == "mult":
            return np.prod(score_list)
        # Default
        return np.max(score_list)

    def _get_thresholds(self, uncertainty_scores_by_window_size: dict) -> tuple[dict, dict]:
        """Get the thresholds for the quantiles and window sizes defined in self.cfg.

        Args:
            uncertainty_scores_by_window_size: Dictionary containing the uncertainty scores and success labels for each window size.

        Returns:
            (thresholds, time_varying_thresholds, scores_by_threshold, scores_by_threshold_tvt):
                - Dictionary containing the constant thresholds for each quantile and window size.
                - Dictionary containing the lists of time-varying thresholds for each quantile and window size.
                - Dictionary containing the normalized uncertainty scores for each quantile and window size.
                - Dictionary containing the normalized uncertainty scores for each quantile and window size using time-varying thresholds.
        """
        # Set up the dictionaries to store the thresholds and scores
        thresholds = {}
        scores_by_threshold = {}
        for threshold_style in self.cfg.thresholds:
            if self.cfg.thresholds[threshold_style]:
                thresholds[threshold_style] = {}
                scores_by_threshold[threshold_style] = {}

        for threshold_style in thresholds.keys():
            for quantile in self.cfg.quantiles:
                thresholds[threshold_style][quantile] = {}
                scores_by_threshold[threshold_style][quantile] = {}
                for window_size in self.cfg.window_sizes:
                    # We use only the successful episodes for the calibration
                    uncertainty_scores = [
                        episode_info["uncertainty_scores"]
                        for episode_info in uncertainty_scores_by_window_size[window_size]
                        if episode_info["successful"]
                    ]
                    thresholds[threshold_style][quantile][window_size] = compute_thresholds(
                        threshold_style,
                        quantile,
                        uncertainty_scores,
                    )

                    # Calculate the scores by threshold (also for failed episodes)
                    uncertainty_scores = [
                        episode_info["uncertainty_scores"]
                        for episode_info in uncertainty_scores_by_window_size[window_size]
                    ]
                    scores_by_threshold[threshold_style][quantile][window_size] = normalize_scores_by_threshold(
                        uncertainty_scores,
                        thresholds[threshold_style][quantile][window_size],
                        cfg=self.cfg,
                    )

        return thresholds, scores_by_threshold

    def _get_metrics(self, thresholds: dict, test_scores: dict) -> tuple[dict, dict]:
        """Evaluate the test results using the constant and time-varying tresholds in the calibration results."""
        # Obtain some dataset statistics
        dataset_stats = {
            "max_episode_length": max(self.dataset.data["metadata"]["episode_lengths"]),
            "id_rollouts": self.dataset.get_rollout_subtypes(subset="test", subsubset="id"),
            "ood_rollouts": self.dataset.get_rollout_subtypes(subset="test", subsubset="ood"),
            "successful_rollouts": self.dataset.get_rollout_types(
                subset=["test", "successful"], reduce_mask_to=["test"]
            ),
        }
        scores_by_threshold_test = {}
        metrics = {}
        for threshold_style in thresholds.keys():
            scores_by_threshold_test[threshold_style] = {}
            metrics[threshold_style] = {}
            for quantile in self.cfg.quantiles:
                scores_by_threshold_test[threshold_style][quantile] = {}
                metrics[threshold_style][quantile] = {}
                for window_size in self.cfg.window_sizes:
                    # Extract the uncertainty scores from the test scores
                    uncertainty_scores = [
                        episode_info["uncertainty_scores"] for episode_info in test_scores[window_size]
                    ]
                    # List of successful episodes
                    # successful_rollouts = [episode_info["successful"] for episode_info in test_scores[window_size]]
                    # Normalize the test scores by the thresholds
                    scores_by_threshold_test[threshold_style][quantile][window_size] = normalize_scores_by_threshold(
                        uncertainty_scores,
                        thresholds[threshold_style][quantile][window_size],
                        cfg=self.cfg,
                    )
                    # Calculate the metrics using the normalized scores and the successful rollouts
                    metrics[threshold_style][quantile][window_size] = calculate_metrics(
                        scores_by_threshold_test[threshold_style][quantile][window_size],
                        dataset_stats, self.cfg.detection_patience,
                    )

        return metrics, scores_by_threshold_test

    def evaluate(self) -> dict:
        """Evaluate a method."""

        # Full freedom to use the dataset class
        self._execute_preprocessing()

        # Process the calibration rollouts
        uncertainty_scores_by_window_size_calibration, avg_inference_time_calibration = self._process_rollouts(
            subset="calibration"
        )
        # Thresholds are dicts with structure: thresholds[threshold_style][quantile][window_size] = threshold, where threshold is a float or a list of floats (for time-varying thresholds)
        thresholds, scores_by_threshold_calibration = self._get_thresholds(
            uncertainty_scores_by_window_size_calibration
        )
        # Process the test rollouts
        uncertainty_scores_by_window_size_test, avg_inference_time_test = self._process_rollouts(subset="test")

        # Calculate the metrics for the test rollouts
        metrics, scores_by_threshold_test = self._get_metrics(thresholds, uncertainty_scores_by_window_size_test)

        eval_results = {
            "method": self.method_name,
            "quantiles": self.cfg.quantiles,
            "window_sizes": self.cfg.window_sizes,
            "calibration_uncertainty_scores": uncertainty_scores_by_window_size_calibration,
            "calibration_thresholds": thresholds,
            "calibration_scores_by_threshold": scores_by_threshold_calibration,
            "test_uncertainty_scores": uncertainty_scores_by_window_size_test,
            "test_metrics": metrics,
            "test_scores_by_threshold": scores_by_threshold_test,
            "avg_inference_time": np.mean([avg_inference_time_test, avg_inference_time_calibration]),
            "cfg": self.cfg,
            "max_episode_length": max(self.dataset.data["metadata"]["episode_lengths"]),
            "successful_test_rollouts": self.dataset.get_rollout_types(
                subset=["test", "successful"], reduce_mask_to=["test"]
            ),
            "id_test_rollouts": self.dataset.get_rollout_subtypes(subset="test", subsubset="id"),
            "ood_test_rollouts": self.dataset.get_rollout_subtypes(subset="test", subsubset="ood"),
        }
        self._save_pickle(self.results_dir, eval_results, "eval_results.pkl")
        return eval_results

    def _process_one_rollout(self, rollout_dict: dict) -> tuple[list[float], float]:
        """
        Obtain the uncertainty scores and average inference time for one rollout.

        Args:
            rollout_dict: One rollout that is a dict with the required tensors and one success label as entries.

        Returns:
            uncertainty_scores: List of uncertainty scores for each step in a rollout.
            avg_inference_time: Average inference time.
        """
        # rollout_dict contains the success labels and the tensors for the episode
        rollout_length = rollout_dict[self.required_tensors[0]].shape[0]
        num_robots = self.dataset.data["metadata"].get("num_robots", 1)
        inference_times = []
        uncertainty_scores_one_rollout = []
        for i in range(rollout_length):
            new_dict = {}
            for key in rollout_dict.keys():
                if key == "successful":
                    continue
                else:
                    new_dict[key] = rollout_dict[key][i]
            start_time = time.time()
            uncertainty_score = self.calculate_uncertainty_score(rollout_tensor_dict=new_dict)
            inference_times.append(time.time() - start_time)
            if self.cfg.get("handle_zero_thresholds", {"style": "whatever"})["style"] == "add_small_score":
                # Add a small value to avoid division by zero
                uncertainty_score = uncertainty_score + 1e-6
            uncertainty_scores_one_rollout.append(uncertainty_score)

        # If the dataset contains multiple robots, compound the uncertainty scores
        if num_robots > 1:
            # Split the uncertainty scores into sublists, each sublist corresponds to one step
            sublists = [
                uncertainty_scores_one_rollout[i : i + num_robots]
                for i in range(0, len(uncertainty_scores_one_rollout), num_robots)
            ]
            uncertainty_scores_one_rollout = []
            for sublist in sublists:
                # Compound the uncertainty scores for each robot
                compounded_score = self._compound_uncertainty_scores(sublist)
                uncertainty_scores_one_rollout.append(compounded_score)
        assert len(uncertainty_scores_one_rollout) == rollout_length // num_robots
        avg_inference_time = np.mean(inference_times)
        return uncertainty_scores_one_rollout, avg_inference_time

    def _process_rollouts(self, subset: str) -> tuple[dict, float]:
        """Processes the rollouts in the dataset of the given subset and returns the uncertainty scores for each window size and the average inference time.

        Returns:
            (uncertainty_scores_by_window_size, avg_inference_time):
                - uncertainty_scores_by_window_size: A dictionary containing the uncertainty scores for each window size and sucess labels.
                - avg_inference_times: Average inference time during rollout procession.
        """
        inference_times = []
        uncertainty_scores_by_window_size = {}
        for window_size in self.cfg.window_sizes:
            uncertainty_scores_by_window_size[window_size] = []

        for rollout_dict in self.dataset.iterate_episodes(
            subset=subset,
            required_tensors=self.required_tensors,
            optional_tensors=self.optional_tensors,
            required_actions=self.required_actions,
            optional_actions=self.optional_actions,
            with_success_labels=True,
            normalize_tensors=self.normalize_tensors,
            history=self.cfg.history_length,
        ):
            # Process the rollout
            uncertainty_scores_one_rollout, inference_time = self._process_one_rollout(rollout_dict)

            inference_times.append(inference_time)
            for window_size in self.cfg.window_sizes:
                uncertainty_scores = self._apply_window_size(uncertainty_scores_one_rollout, window_size)
                episode_info = {
                    "successful": rollout_dict["successful"],
                    "uncertainty_scores": uncertainty_scores,
                }
                uncertainty_scores_by_window_size[window_size].append(episode_info)
        # Calculate the average inference time
        avg_inference_time = np.mean(inference_times)
        return uncertainty_scores_by_window_size, avg_inference_time

    def _apply_window_size(self, uncertainty_scores_one_rollout: list, window_size: int):
        """
        Apply the window size to the uncertainty scores.

        Args:
            uncertainty_scores_one_rollout: Uncertainty scores for one rollout.
            window_size: Window size, either "all" (cumulative score), or an integer (size of the window).

        Returns:
            uncertainty_scores: Uncertainty scores with the window size applied.
        """
        if len(uncertainty_scores_one_rollout) == 0:
            return np.zeros(0)

        # Convert inputs
        uncertainty_scores_one_rollout = np.asarray(uncertainty_scores_one_rollout, dtype=np.float64)
        original_length = len(uncertainty_scores_one_rollout)

        # Handle "all" window size
        if window_size == "all":
            result = np.cumsum(uncertainty_scores_one_rollout)
            return result

        # Use a fixed window size
        fixed_window_size = int(window_size)

        # Pad the rollout with zeros if it's shorter than the fixed window size
        if len(uncertainty_scores_one_rollout) < fixed_window_size:
            padding = np.zeros(fixed_window_size - len(uncertainty_scores_one_rollout))
            uncertainty_scores_one_rollout = np.concatenate([padding, uncertainty_scores_one_rollout])

        # Define weights uniform windows
        weights = np.ones(fixed_window_size, dtype=np.float64)

        # Apply the weighted window
        uncertainty_scores = np.array(
            [
                np.sum(
                    uncertainty_scores_one_rollout[max(0, i - fixed_window_size + 1) : i + 1]
                    * weights[-(i - max(0, i - fixed_window_size + 1) + 1) :]
                )
                for i in range(len(uncertainty_scores_one_rollout))
            ]
        )

        # Remove padding from the result
        if len(uncertainty_scores) > original_length:
            uncertainty_scores = uncertainty_scores[-original_length:]

        return uncertainty_scores

    def _save_pickle(self, save_dir, data, filename):
        if not filename.endswith(".pkl"):
            filename += ".pkl"
        os.makedirs(save_dir, exist_ok=True)
        with open(os.path.join(save_dir, filename), "wb") as f:
            pickle.dump(data, f)

    def _load_pickle(self, load_dir, filename):
        if not filename.endswith(".pkl"):
            filename += ".pkl"
        with open(os.path.join(load_dir, filename), "rb") as f:
            data = pickle.load(f)
        return data
