import torch
import hydra
import numpy as np
from shared_utils import load_config
from .method_eval_classes import BaseEvalClass
from omegaconf import OmegaConf, DictConfig
from evaluation.utils import calculate_metrics
from datasets.rollout_datasets import ProcessedRolloutDataset


class EvaluationManager:
    def __init__(
        self,
        config_path: str,
        task_data_path: str,
        dataset: ProcessedRolloutDataset,
        device: str = None,
        seed: int = None,
        **kwargs,
    ):
        """The EvaluationManager class provides the inferface of the evaluation of the failure prediction methods.
        It loads the method-specific configuration and then calls the method-specific evaluation classes.

        Important:
        - Task selection is done via given dataset, config_path, and task_data_path.
        - To run the evaluation, call the evaluate method with the methods to evaluate.
        - Evaluation results are returned as a dictionary with the method names as keys.

        Returns:
            evaluation_manager: An EvaluationManager object

        Args:
            config_path (str): The path to the base configuration folder.
            task_data_path (str): The base path to the data (task-specific).
            dataset (ProcessedRolloutDataset): The dataset to use for evaluation.
            device (str, optional): The device ["cuda", "cpu"] to use for evaluation. If not provided, "cuda" is used if available.
        """
        self.base_config_path = config_path
        self.task_data_path = task_data_path
        self.base_cfg = self._load_config()
        if not isinstance(dataset, ProcessedRolloutDataset):
            raise ValueError("dataset must be of type ProcessedRolloutDataset.")
        self.dataset = dataset
        self.kwargs = kwargs
        self.seed = seed

        if device is not None and device in ["cpu", "cuda:0"]:
            self.device = device if torch.cuda.is_available() else "cpu"
        else:
            self.device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")

    def evaluate(self, methods, combine_methods: bool = False, combined_methods: dict = None) -> dict:
        """
        Evaluate the given methods.

        Args:
            methods (list): The list of methods to evaluate.
            visualize (bool, optional): If True, visualize the results. Defaults to False.
            combine_methods (bool, optional): If True, logically combine the methods given in the combined_methods dict. Defaults to False.
            combined_methods (dict, optional): Dictionary containing the methods to combine. Defaults to None.

        Returns:
            dict: A dictionary containing the evaluation results for each method.
        """
        total_results = {}  # Dictionary to store results for all methods

        for method in methods:
            # Load the configuration for the method
            cfg = self._load_config(method_name=method)
            if self.seed is not None and "seed" in cfg.hparams.model:
                cfg.hparams.model.seed = self.seed

            # Get the method-specific evaluation class
            methodEvalClass: BaseEvalClass = self._get_method_eval_class(method, cfg)

            # Evaluate the method and store the results
            total_results[method] = methodEvalClass.evaluate()

        # Combine methods if specified
        if combine_methods:
            for key in combined_methods:
                total_results = self._combine_two_methods(combined_methods[key], total_results)

        return total_results

    def _combine_two_methods(self, combination_config, total_results):
        """Logically combine two failure prediction methods based on the given combination configuration."""
        method1 = combination_config["m1"]["name"]
        method2 = combination_config["m2"]["name"]
        operation = combination_config["operation"]
        method_keyword = method1 + "_" + operation + "_" + method2

        # Get the results of the two methods
        method1_results = total_results[method1]
        method2_results = total_results[method2]
        # Merge the configuration of the two methods
        method1_cfg = OmegaConf.to_container(method1_results["cfg"], resolve=True)
        method2_cfg = OmegaConf.to_container(method2_results["cfg"], resolve=True)
        combined_config = OmegaConf.merge(OmegaConf.create(method1_cfg), OmegaConf.create(method2_cfg))

        # Get the combined quantiles and window sizes
        quantiles1: list = combination_config["m1"].get("quantiles", None)
        if quantiles1 is None:
            quantiles1: list = method1_results["quantiles"]
        window_sizes1 = combination_config["m1"].get("window_sizes", None)
        if window_sizes1 is None:
            window_sizes1 = method1_results["window_sizes"]
        quantiles2: list = combination_config["m2"].get("quantiles", None)
        if quantiles2 is None:
            quantiles2: list = method2_results["quantiles"]
        window_sizes2 = combination_config["m2"].get("window_sizes", None)
        if window_sizes2 is None:
            window_sizes2: list = method2_results["window_sizes"]

        # Combine quantiles and window sizes
        if quantiles1 != quantiles2:
            new_quantiles1, new_quantiles2 = [], []
            for i in range(len(quantiles1)):
                for j in range(len(quantiles2)):
                    new_quantiles1.append(quantiles1[i])
                    new_quantiles2.append(quantiles2[j])
            quantiles1, quantiles2 = new_quantiles1, new_quantiles2

        new_window_sizes1, new_window_sizes2 = [], []
        for i in range(len(window_sizes1)):
            for j in range(len(window_sizes2)):
                new_window_sizes1.append(window_sizes1[i])
                new_window_sizes2.append(window_sizes2[j])
        window_sizes1, window_sizes2 = new_window_sizes1, new_window_sizes2

        total_results[method_keyword] = {
            "method": method_keyword,
            "quantiles": None,
            "window_sizes": None,
            "calibration_uncertainty_scores": {
                method1: method1_results["calibration_uncertainty_scores"],
                method2: method2_results["calibration_uncertainty_scores"],
            },
            "test_uncertainty_scores": {
                method1: method1_results["test_uncertainty_scores"],
                method2: method2_results["test_uncertainty_scores"],
            },
            "calibration_thresholds": {
                method1: method1_results["calibration_thresholds"],
                method2: method2_results["calibration_thresholds"],
            },
            "test_scores_by_threshold": {},
            "test_metrics": {},
            "avg_inference_time": method1_results["avg_inference_time"] + method2_results["avg_inference_time"],
            "max_episode_length": method1_results["max_episode_length"],
            "successful_test_rollouts": method1_results["successful_test_rollouts"],
            "id_test_rollouts": method1_results["id_test_rollouts"],
            "ood_test_rollouts": method1_results["ood_test_rollouts"],
            "cfg": combined_config,
        }

        dataset_stats = {
            "id_rollouts": method1_results["id_test_rollouts"],
            "ood_rollouts": method1_results["ood_test_rollouts"],
            "successful_rollouts": method1_results["successful_test_rollouts"],
            "max_episode_length": method1_results["max_episode_length"],
        }
        threshold_styles = list(method1_results["calibration_thresholds"].keys())

        comb_quantiles, test_metrics, test_scores_by_threshold = [], {}, {}
        for threshold_style in threshold_styles:
            test_metrics[threshold_style] = {}
            test_scores_by_threshold[threshold_style] = {}
            for quantile1, quantile2 in zip(quantiles1, quantiles2):
                comb_quantile = quantile1 if quantile1 == quantile2 else f"{quantile1}/{quantile2}"
                comb_quantiles.append(comb_quantile)
                test_metrics[threshold_style][comb_quantile] = {}
                test_scores_by_threshold[threshold_style][comb_quantile] = {}

                comb_window_sizes = []
                for window_size1, window_size2 in zip(window_sizes1, window_sizes2):
                    comb_window_size = (
                        window_size1 if window_size1 == window_size2 else f"{window_size1}/{window_size2}"
                    )
                    comb_window_sizes.append(comb_window_size)

                    scores_by_threshold_m1 = method1_results["test_scores_by_threshold"][threshold_style][quantile1][
                        window_size1
                    ]
                    scores_by_threshold_m2 = method2_results["test_scores_by_threshold"][threshold_style][quantile2][
                        window_size2
                    ]

                    scores_by_threshold = []
                    for episode in range(len(scores_by_threshold_m1)):
                        if operation == "and":
                            scores_by_threshold.append(
                                np.minimum(scores_by_threshold_m1[episode], scores_by_threshold_m2[episode])
                            )
                        elif operation == "or":
                            scores_by_threshold.append(
                                np.maximum(scores_by_threshold_m1[episode], scores_by_threshold_m2[episode])
                            )

                    test_metrics[threshold_style][comb_quantile][comb_window_size] = calculate_metrics(
                        scores_by_threshold, dataset_stats, combined_config["detection_patience"]
                    )
                    test_scores_by_threshold[threshold_style][comb_quantile][comb_window_size] = scores_by_threshold

        total_results[method_keyword]["test_metrics"] = test_metrics
        total_results[method_keyword]["test_scores_by_threshold"] = test_scores_by_threshold
        total_results[method_keyword]["quantiles"] = comb_quantiles
        total_results[method_keyword]["window_sizes"] = comb_window_sizes
        return total_results

    def _get_method_eval_class(self, method_name: str, cfg):
        """Get the method-specific evaluation class. The classes are subclasses from the base evaluation class and only implement the model-specific functions."""
        # Get the class name from the method name (normalized version use the same class as the non-normalized version)
        class_name = method_name
        # All RND method used the same evaluation class
        if "rnd" in class_name:
            class_name = "rnd"

        class_name = f"{class_name.replace('_', '').upper()}Eval"  # Class name follows the pattern {METHOD}Eval where METHOD is the method name in uppercase without underscores
        module_name = "evaluation.method_eval_classes."  # Base module name for method_eval_classes

        # Import the method_eval_classes module (which exposes all classes in its __init__.py)
        method_eval_class: BaseEvalClass = hydra.utils.get_class(module_name + class_name)

        class_obj = method_eval_class(
            cfg=cfg,
            method_name=method_name,
            device=self.device,
            task_data_path=self.task_data_path,
            dataset=self.dataset,
            **self.kwargs,
        )
        return class_obj

    def _load_config(self, method_name: str = "base") -> DictConfig:
        """Load the configuration file for the respective method_name with hydra."""
        cfg = load_config(
            module="eval", filename=method_name, return_only_subdict=True, base_config_dir=self.base_config_path
        )
        return cfg
