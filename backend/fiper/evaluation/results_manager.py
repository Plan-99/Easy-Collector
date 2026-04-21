import os
import torch
import numpy as np
from shared_utils import save_data, load_config, load_data, compare_dicts
from shared_utils.data_management import _get_filenames, ensure_list
from omegaconf import OmegaConf, DictConfig
import matplotlib.pyplot as plt
import pandas as pd
from typing import List, Optional, Tuple
from evaluation.utils import save_videos_with_warning

class ResultsManager:
    def __init__(
        self,
        config_path: str,
        base_data_path: str,
        **kwargs,
    ):
        """The ResultsManager class is responsible for managing the results of the evaluation.

        It saves/loads the individual results for all tasks/methods/hparams, combines them, and generates the final results using the result configuration file.

        It optionally visualizes the results.

        Results structure: nested dictionary with the following structure: {task_name: {method_name: hyperparameter_id: {window_size: {quantile: {metrics}}}}}}

        The results are saved in {base_data_path}/results/.

        Returns:
            results_manager: An EvaluationManager object

        Args:
            config_path (str): The path to the base configuration folder.
            base_data_path (str): The base path to the data.
        """
        self.base_config_path = config_path
        self.base_data_path = base_data_path
        self.results_dir = os.path.join(self.base_data_path, "results")
        os.makedirs(self.results_dir, exist_ok=True)
        self.summary_dir = os.path.join(self.results_dir, "summaries")
        os.makedirs(self.summary_dir, exist_ok=True)
        self.method_results_dir = os.path.join(self.results_dir, "method_results")
        os.makedirs(self.method_results_dir, exist_ok=True)

        self.cfg = self._load_config()
        self.kwargs = kwargs

    def _load_config(self, filename: str = "base") -> DictConfig:
        """Load the results configuration file with hydra."""
        cfg = load_config(
            module="results", filename=filename, return_only_subdict=True, base_config_dir=self.base_config_path
        )
        return cfg

    def _load_results(self, method_names: Optional[List[str]] = None) -> dict:
        """Load results for specific methods or all methods if no method_names are provided."""
        results = {}
        filenames = _get_filenames(self.method_results_dir, keywords="_results", data_types="pkl")
        for filename in filenames:
            method_name = filename.split("_results.pkl")[0]
            if method_names and method_name not in method_names:
                continue
            method_results = load_data(
                self.method_results_dir, keywords=filename, data_types="pkl", error_if_not_found=False
            )
            if isinstance(method_results, list) and len(method_results) > 0:
                method_results = method_results[0]
            if isinstance(method_results, dict):
                results[method_name] = method_results
        return results

    def _save_results(self, results: dict, method_names: Optional[List[str]] = None) -> None:
        """Save results for specific methods or all methods if no method_names are provided."""
        for method, method_results in results.items():
            if method_names and method not in method_names:
                continue
            filename = f"{method}_results.pkl"
            save_data(self.method_results_dir, filename, method_results, data_types="pkl", overwrite=True)

    def _save_dataframe(self, df: pd.DataFrame, filename="complete_results") -> None:
        if not filename.endswith(".csv"):
            filename += ".csv"
        df.to_csv(os.path.join(self.results_dir, filename), index=False)

    def _save_summary(self, df: pd.DataFrame, filename="summary") -> None:
        if not filename.endswith(".csv"):
            filename += ".csv"
        df.to_csv(os.path.join(self.summary_dir, filename), index=False)

    def _load_dataframe(self, filename="complete_results") -> pd.DataFrame:
        """Load the results of the evaluation from the results directory."""
        if not filename.endswith(".csv"):
            filename += ".csv"
        df = pd.read_csv(os.path.join(self.results_dir, filename))
        return df

    def _convert_new_results(self, new_results: dict = {}, method_names: list = [None]) -> dict:
        """Convert the new results to the format of the complete results.

        The new results are in the format {task_name: {method_name: {metrics}}
        The complete results are in the format {method_name: hyperparameter_id: {task_name: {metrics}}}

        Args:
            new_results (dict): The results of the evaluation.
            method_name (str): The name of the method to extract.

        Returns:
            dict: The results of the specified method.
        """
        # Check if the new results are empty
        if not new_results or not method_names:
            return {}

        # Check if the new results are a dictionary
        if not isinstance(new_results, dict):
            raise ValueError("New results must be a dictionary.")

        # Convert the new results to the format of the complete results
        converted_results = {}
        for method_name in method_names:
            method_results = {}
            method_results[0] = {}
            hparams = {}
            for task in new_results.keys():
                if method_name not in new_results[task]:
                    continue
                method_results[0][task], hparams = self._filter_new_results(new_results[task][method_name])
            if hparams:
                method_results[0]["hparams"] = hparams
            converted_results[method_name] = method_results
        return converted_results

    def _filter_new_results(self, new_results: dict) -> Tuple[dict, dict]:
        hparams = new_results["cfg"]["hparams"]
        new_results.pop("cfg")
        return new_results, hparams

    def _create_complete_df(self) -> pd.DataFrame:
        """Create a complete DataFrame from the results.

        The DataFrame contains the following columns:
            - Task
            - Method
            - HID
            - Window
            - Quantile
            - Metric 1
            - Metric 2
            - ...
        """
        # Create a DataFrame from the results
        complete_df = pd.DataFrame()
        # Obtain the available results
        filenames = _get_filenames(self.method_results_dir, keywords="_results", data_types="pkl")
        methods = [filename.split("_results.pkl")[0] for filename in filenames]
        for method in methods:
            method_results = self._load_results(method_names=[method]).get(method, {})
            if not method_results:
                continue
            for hparam_id in method_results.keys():
                for task in method_results[hparam_id].keys():
                    if task == "hparams":
                        continue
                    for window_size in method_results[hparam_id][task]["window_sizes"]:
                        for quantile in method_results[hparam_id][task]["quantiles"]:
                            for threshold_style in method_results[hparam_id][task]["test_metrics"].keys():
                                metrics = method_results[hparam_id][task]["test_metrics"][threshold_style][quantile][
                                    window_size
                                ]
                                row = {
                                    "Method": method,
                                    "Task": task,
                                    "TWA": float(metrics["TWA"]),
                                    "TWA_std": float(metrics["TWA_std"]) if "TWA_std" in metrics else 0.0,
                                    "Accuracy": float(metrics["balanced_accuracy"]),
                                    "Accuracy_std": float(metrics["balanced_accuracy_std"]) if "balanced_accuracy_std" in metrics else 0.0,
                                    "Det. Time": float(metrics["avg_detection_time"]),
                                    "Det. Time_std": float(metrics["avg_detection_time_std"]) if "avg_detection_time_std" in metrics else 0.0,
                                    "HID": int(hparam_id),
                                    "Window": str(window_size),
                                    "Quantile": float(quantile),
                                    "TPR": float(metrics["TPR"]),
                                    "TPR_std": float(metrics["TPR_std"]) if "TPR_std" in metrics else 0.0,
                                    "TNR": float(metrics["TNR"]),
                                    "TNR_std": float(metrics["TNR_std"]) if "TNR_std" in metrics else 0.0,
                                    "Threshold": str(threshold_style),
                                }
                                if "avg_episode_stats_by_type" in metrics:
                                    for key, value in metrics["avg_episode_stats_by_type"].items():
                                        if key == "id_success":
                                            keyword = "ID S"
                                        elif key == "ood_success":
                                            keyword = "OOD S"
                                        elif key == "id_failure":
                                            keyword = "ID F"
                                        elif key == "ood_failure":
                                            keyword = "OOD F"
                                        else:
                                            continue

                                        row[keyword] = float(value["mean"])
                                        # row[keyword + " STD"] = value["std"]
                                        # for k, v in value.get("percentiles", {}).items():
                                        #     row[keyword + " P" + str(k)] = v

                                complete_df = pd.concat([complete_df, pd.DataFrame([row])], ignore_index=True)

        complete_df = complete_df.round(3)
        return complete_df

    def combine_results(self, new_results: dict = {}, method_names: list = [], **kwargs) -> None:
        """Combine the new results with the existing results."""
        if not new_results or not method_names:
            # If self.results_dir does not contain complete_results.csv, create it from the existing method results
            if not os.path.exists(os.path.join(self.results_dir, "complete_results.csv")):
                complete_df: pd.DataFrame = self._create_complete_df()
                self._save_dataframe(complete_df, filename="complete_results")
            
            return

        if not isinstance(new_results, dict):
            raise ValueError("New results must be a dictionary.")

        cfg = self.cfg.copy()
        cfg.update(kwargs)

        for method_name in method_names:
            new_method_results = self._convert_new_results(new_results, [method_name]).get(method_name, {})
            if cfg.overwrite_data:
                old_method_results = {}
            else:
                old_method_results = self._load_results([method_name]).get(method_name, {})

            combined_method_results = old_method_results.copy()

            for new_hid in new_method_results.keys():
                if not old_method_results:
                    combined_method_results = {new_hid: new_method_results[new_hid]}
                else:
                    match_found = False
                    new_hyperparamter_dict = new_method_results[new_hid]["hparams"]
                    for old_hid in old_method_results.keys():
                        old_hyperparamter_dict = old_method_results[old_hid]["hparams"]
                        if compare_dicts(new_hyperparamter_dict, old_hyperparamter_dict):
                            combined_method_results[old_hid] = self._update_dictionary_recursively(
                                combined_method_results[old_hid], new_method_results[new_hid]
                            )
                            match_found = True
                            break

                    if not match_found:
                        combined_method_results[max(combined_method_results.keys(), default=-1) + 1] = (
                            new_method_results[new_hid]
                        )
            # Save the combined results for the method
            self._save_results({method_name: combined_method_results}, method_names=[method_name])

        complete_df: pd.DataFrame = self._create_complete_df()
        self._save_dataframe(complete_df, filename="complete_results")

    def create_summary(self, **kwargs):
        """Create a summary of the results.

        The summary is saved as "{base_data_path}/results/{summary_name}.csv".

        Args:
            new_results (dict): The results of the current evaluation. Only used if only_latest is set to True.
            kwargs (dict): Additional arguments to control the summary generation. Overrides the config file.
        """
        cfg = self.cfg.copy()
        cfg.update(kwargs)
        if not cfg.create_summary:
            return
        # Load the complete DataFrame
        summary_df = self._load_dataframe()

        cfg.metric_columns = OmegaConf.to_container(cfg.metric_columns, resolve=True)
        # metric_columns: dict = dict(metric_columns)
        # Filter the DataFrame based on the filter columns and values
        summary_df = self._filter_dataframe(
            df=summary_df,
            cfg=cfg,
        )

        # Average the DataFrame over the specified columns
        summary_df = self._average_dataframe(
            df=summary_df,
            cfg=cfg,
        )

        summary_df = self._sort_and_reorder_dataframe(
            df=summary_df,
            cfg=cfg,
        )
        # Print the summary DataFrame
        print(summary_df)
        # Print the hyperparameters if desired
        hp_df_save_dict = {}
        if "Method" in summary_df.columns and "HID" in summary_df.columns:
            if cfg.print_hyperparameters or cfg.save_hyperparameters:
                for method in summary_df["Method"].unique():
                    reduced_df = summary_df[summary_df["Method"] == method]
                    # All hyperparameter IDs
                    hparam_ids = [int(hparam_id) for hparam_id in reduced_df["HID"].unique()]
                    # Calculate the best hyperparameter ID
                    best_hparam_id = self._get_best_hyperparameter_id(reduced_df, cfg)

                    # Create a DataFrame for the hyperparameters
                    hp_df, hp_dict = self._create_hyperparameter_dataframe(method, hparam_ids, cfg)

                    # Differences between the hyperparameters
                    hp_df_diff = self._get_hyperparameter_differences(hp_df)

                    # Print and save the hyperparameters
                    hp_df_save_dict[method] = self._output_hyperparameters(
                        method=method,
                        best_hparam_id=best_hparam_id,
                        hp_df=hp_df,
                        hp_df_diff=hp_df_diff,
                        hp_dict=hp_dict,
                        hparam_ids=hparam_ids,
                        cfg=cfg,
                    )

        # Save the summary DataFrame to a CSV file

        # Get existing summaries and hparam summaries
        filenames = _get_filenames(self.summary_dir, keywords="summary", data_types="csv")
        # Filter out the hparams
        filenames_s = [filename for filename in filenames if "hparams" not in filename]
        # Remove complete_summary
        # Determine new filename
        if len(filenames_s) == 0:
            new_filename = "summary_00.csv"
        else:
            overwrite = cfg.overwrite_summary
            if overwrite == "latest":
                new_filename = filenames_s[-1]
            elif overwrite == "all":
                for filename in filenames:
                    os.remove(os.path.join(self.summary_dir, filename))
                new_filename = "summary_00.csv"
            elif overwrite == "oldest":
                new_filename = filenames_s[0]
            else:
                # Get the latest summary file
                latest_filename = sorted(filenames_s)[-1]
                # Extract the number from the filename
                number = int(latest_filename.split("_")[1].split(".")[0])
                new_filename = f"summary_{number + 1:02d}.csv"
        self._save_summary(summary_df, filename=new_filename)
        if hp_df_save_dict:
            for key, value in hp_df_save_dict.items():
                if not value:
                    continue
                hid_filename = new_filename.replace(".csv", "_" + key + "_hparams.csv")
                self._save_summary(value, filename=hid_filename)

    def _update_dictionary_recursively(self, dict1: dict, dict2: dict):
        """Recursively update dict1 with dict2."""
        for key, value in dict2.items():
            if isinstance(value, dict) and key in dict1:
                self._update_dictionary_recursively(dict1[key], value)
            else:
                if key in ["window_sizes", "quantiles"] and key in dict1:
                    # If the key is "window_sizes" or "quantiles", append the new values to the existing list
                    dict1[key] = list(set(dict1[key] + value))
                else:
                    dict1[key] = value
        return dict1

    def _get_hyperparameters(self, method, hparam_id, resolve=True, include_hparams: list = []) -> dict:
        """Get the hyperparameters of the method."""
        complete_results = self._load_results()
        method_results = complete_results[method]
        if hparam_id not in method_results:
            raise ValueError(f"Hyperparameter ID {hparam_id} not found for method {method}.")
        hparams: DictConfig = method_results[hparam_id].get("hparams", {})
        if not hparams:
            raise ValueError(f"Hyperparameters not found for method {method} and hyperparameter ID {hparam_id}.")

        if resolve:
            hparams = OmegaConf.to_container(hparams, resolve=True)
        if include_hparams:
            hparams = {key: value for key, value in hparams.items() if key in include_hparams}
        return hparams

    def _remove_hparam_ids_from_results(self, methods: list, hparam_ids: list) -> None:
        """Remove a hyperparameter ID for a method from the results."""
        methods, hparam_ids = ensure_list(methods, hparam_ids)
        assert len(methods) == len(hparam_ids), "Methods and hyperparameter IDs must have the same length."
        combined_results: dict = self._load_results()
        for method, hparam_id in zip(methods, hparam_ids):
            if method not in combined_results:
                continue
            method_results: dict = combined_results[method]
            if hparam_id not in method_results:
                continue
            method_results.pop(hparam_id)
        # Save the combined results
        self._save_results(combined_results)
        # Recreate the complete DataFrame
        complete_df: pd.DataFrame = self._create_complete_df(combined_results)
        self._save_dataframe(complete_df, filename="complete_results")

    def _clean_results(self, clean_dict: dict) -> None:
        """Clean the results of a method by removing specified hyperparameter IDs.

        Args:
            clean_dict (dict): A dictionary with the following structure:
                {method_name: {keyword: keyword, value: value}}, where key is in "del", "keep", "keep_best", "del_worst" and value is a list of hyperparameter IDs or a integer.
        """
        assert isinstance(clean_dict, dict), "clean_dict must be a dictionary."
        methods_to_clean = []
        hparam_ids_to_clean = []
        for method in clean_dict:
            if not clean_dict[method] or clean_dict[method] is None:
                continue
            if not isinstance(clean_dict[method], dict):
                continue
            if "keyword" not in clean_dict[method] or "value" not in clean_dict[method]:
                continue
            keyword = clean_dict["keyword"]
            value = clean_dict["value"]
            assert keyword in ["del", "keep", "keep_best", "del_worst"], (
                f"Invalid keyword: {keyword}. Must be one of ['del', 'keep', 'keep_best', 'del_worst']."
            )
            assert isinstance(value, list) or isinstance(value, int), "value must be a list or an integer."
            if isinstance(value, int):
                value = [value]
            if keyword == "del":
                hparam_ids_to_clean.extend(value)
                methods_to_clean.extend([method for _ in range(len(value))])
            elif keyword == "del_worst":
                value = value[0]

        raise NotImplementedError

    def _get_differences_dict(self, dict1, dict2):
        """Get the differences between two dictionaries."""
        differences = {}
        for key in dict1.keys():
            if key not in dict2:
                differences[key] = dict1[key]
            elif isinstance(dict1[key], dict) and isinstance(dict2[key], dict):
                nested_differences = self._get_differences_dict(dict1[key], dict2[key])
                if nested_differences:
                    differences[key] = nested_differences
            elif dict1[key] != dict2[key]:
                differences[key] = (dict1[key], dict2[key])
        return differences

    def extract_warning_frames(self, **kwargs) -> None:
        """Generate uncertainty plots for methods and parameters defined in the configuration optionally overwritten by kwargs.

        Args:
            kwargs (dict): Additional arguments to control the plot generation. Overrides the config file.
        """
        # Update and unpack the config
        cfg = self.cfg.copy()
        OmegaConf.set_struct(cfg, False)
        cfg.update(kwargs)
        cfg.update(cfg.uncertainty_plots)
        OmegaConf.set_struct(cfg, True)
        if not cfg.extract_warning_frames:
            return
        cfg.metric_columns = OmegaConf.to_container(cfg.metric_columns, resolve=True)
        cfg.filter_values = OmegaConf.to_container(cfg.filter_values, resolve=True)
        # Load the results
        results = self._load_results()

        for task in results['rnd_oe_and_entropy'][0].keys():
            # ["tvt_cp_band", "ct_quantile", "tvt_quantile"]
            predictions = results['rnd_oe_and_entropy'][0][task]['test_scores_by_threshold']['tvt_cp_band'][0.95][7]

            # For each list x in predictions, find all indices i for which x[i] > 0.999
            warning_frames = []
            episode_lengths = []
            for x in predictions:
                warning_timesteps = [i for i, value in enumerate(x) if value > 0.9]
                warning_frames.append(warning_timesteps)
                episode_lengths.append(len(x))

            save_videos_with_warning(task, warning_frames, episode_lengths, self.base_data_path)

            # print("Warning frames:", warning_frames)
            # print("Episode lengths:", episode_lengths)
            # print("Average warning frame:", np.mean(warning_frames))


    def generate_uncertainty_plots(self, **kwargs) -> None:
        """Generate uncertainty plots for methods and parameters defined in the configuration optionally overwritten by kwargs.

        Args:
            kwargs (dict): Additional arguments to control the plot generation. Overrides the config file.
        """
        # Update and unpack the config
        cfg = self.cfg.copy()
        OmegaConf.set_struct(cfg, False)
        cfg.update(kwargs)
        cfg.update(cfg.uncertainty_plots)
        OmegaConf.set_struct(cfg, True)
        if not cfg.create_plots:
            return
        cfg.metric_columns = OmegaConf.to_container(cfg.metric_columns, resolve=True)
        cfg.filter_values = OmegaConf.to_container(cfg.filter_values, resolve=True)
        # Load the results
        results = self._load_results()
        df = self._load_dataframe()

        methods = list(results.keys())
        if cfg.filter_values.Method is not None:
            methods = [method for method in methods if method in cfg.filter_values.Method]
        # Option to exclude all windows
        if cfg.exclude_all:
            df = df[~df["Window"].astype(str).str.contains("all", case=False, na=False)]

        for method_name in methods:
            if method_name not in results:
                continue

            # Get the results for the method
            method_results = results[method_name]
            # Filter the DataFrame for the method
            df_method = self._filter_dataframe(df=df[df["Method"].isin([method_name])], cfg=cfg)
            # Get available tasks
            tasks = [key for key in df_method["Task"].unique()]
            tasks = list(set(tasks))
            # Extract the values of the first row
            best_values = {}
            for task in tasks:
                best_values[task] = dict(df_method[df_method["Task"] == task].iloc[0])

            # Extract the uncertainty scores for each task, normalizing them to the treshold
            uncertainty_scores_by_task = {}

            for task in tasks:
                # Get the results for the task and the best values
                uncertainty_scores_by_task[task] = self._extract_uncertainty_scores(
                    task_results=method_results[best_values[task]["HID"]][task],
                    best_values=best_values[task],
                    show_scores=cfg.show,
                    combine_scores=cfg.combine_scores,
                )

            # Create individual plots for each method-task combination
            for task in tasks:
                fig, ax = plt.subplots(figsize=(12, 6))
                ax.set_title(
                    f"Normalized Scores for Method: {cfg.method_name_mapping[method_name]}, Threshold Type: {best_values[task]['Threshold']}, Task: {task}",
                    fontsize=20,
                )
                ax.set_xlabel("Steps", fontsize=20)
                ax.set_ylabel("Uncertainty Score", fontsize=20)

                uncertainty_data = uncertainty_scores_by_task[task]
                for key, values in uncertainty_data.items():
                    if isinstance(values, (list, np.ndarray)):
                        max_length = len(values)
                        ax.plot(range(max_length), values, label=key)
                    elif isinstance(values, (int, float)):
                        ax.axhline(y=values, linestyle="--", label=key)

                ax.legend(fontsize=20)
                ax.set_ybound(lower=0.0, upper=3.0)
                ax.grid(True, linestyle="--", alpha=0.7)

                # Adjust layout and save the figure
                plt.tight_layout()
                save_dir = os.path.join(self.results_dir, "uncertainty_plots", method_name)
                os.makedirs(save_dir, exist_ok=True)
                plot_filename = os.path.join(save_dir, f"{task}_scores.pdf")
                fig.savefig(plot_filename, dpi=300, format="pdf")
                plt.close(fig)

    def _filter_dataframe(self, df, cfg):
        """Filter the DataFrame based on the configuration."""
        metric_columns = cfg.metric_columns
        metric_columns = {col: metric_columns[col] for col in metric_columns if col in df.columns}
        # Remove specified columns
        if cfg.filter_columns:
            df = df.drop(columns=[col for col in cfg.filter_columns if col in df.columns])
            metric_columns = {col: metric_columns[col] for col in metric_columns if col not in cfg.filter_columns}
        cfg.metric_columns = metric_columns

        filter_values = cfg.filter_values
        for column, values in filter_values.items():
            if column not in df.columns or not values:
                continue
            # Handle "best" keyword
            if "best" in values:
                best_position = values.index("best")
                if best_position != 0:
                    values = values[:best_position]
                    if isinstance(values[0], str) and values[0].startswith("!"):
                        values = [value.replace("!", "") for value in values]
                        df = df[~df[column].isin(values)]
                    else:
                        df = df[df[column].isin(values)]

                if not metric_columns:
                    continue

                # Group by specified columns and compute averages
                group_by_columns = [col for col in df.columns if col in cfg.groups_for_best or col == column]
                agg_dict = {col: "mean" for col in metric_columns.keys()}  # Compute the average of metric columns

                grouped_df = (
                    df.groupby(group_by_columns, as_index=False)
                    .agg(agg_dict)
                    .round(3)
                    .sort_values(by=list(metric_columns.keys()), ascending=list(metric_columns.values()))
                    .reset_index(drop=True)
                )

                # Identify non-grouped columns
                non_grouped_columns = [col for col in group_by_columns if col != column]

                # Iterate over all unique combinations of non-grouped columns
                filtered_dfs = []
                for combination in grouped_df[non_grouped_columns].drop_duplicates().to_dict(orient="records"):
                    # Filter the grouped DataFrame for the current combination
                    combination_filter = (grouped_df[list(combination)] == pd.Series(combination)).all(axis=1)
                    filtered_combination_df = grouped_df[combination_filter]

                    # Select the best value for the current combination
                    best_value = filtered_combination_df.iloc[0][column]
                    filtered_df = df[
                        (df[list(combination)] == pd.Series(combination)).all(axis=1) & df[column].isin([best_value])
                    ]
                    filtered_dfs.append(filtered_df)

                # Concatenate all filtered DataFrames
                if filtered_dfs:
                    df = pd.concat(filtered_dfs, ignore_index=True)

                if cfg.drop_after_best:
                    # Remove the column after processing "best"
                    df = df.drop(columns=[column])
            else:
                # Handle negated values (!value)
                if isinstance(values[0], str) and values[0].startswith("!"):
                    values = [value.replace("!", "") for value in values]
                    df = df[~df[column].isin(values)]
                else:
                    df = df[df[column].isin(values)]
        return df

    def _sort_and_reorder_dataframe(self, df, cfg):
        """Sort and reorder the DataFrame columns."""
        if cfg.column_order:
            column_order = [col for col in cfg.column_order if col in df.columns]
            column_order += [col for col in df.columns if col not in column_order]
            df = df[column_order]
        if cfg.sorting:
            sort_by = list(cfg.sorting.keys())
            sort_by = [col for col in sort_by if col in df.columns]
            orders = [cfg.sorting[col] for col in sort_by]
            df = df.sort_values(by=sort_by, ascending=orders)
        return df

    def _average_dataframe(self, df, cfg):
        """
        Average the DataFrame over specified columns.

        Parameters
        ----------
        df : pd.DataFrame
            The input dataframe.
        cfg : object
            Configuration object with attributes:
            - average_columns : list of columns to average over (e.g., ['Quantile', 'Task'])
            - metric_columns : dict with metrics as keys (values can be None)

        Returns
        -------
        pd.DataFrame
            Averaged dataframe.
        """
        if not cfg.average_columns:
            return df

        # Ensure only columns that exist in df
        average_columns = [col for col in cfg.average_columns if col in df.columns]
        metric_columns = [col for col in cfg.metric_columns.keys() if col in df.columns]

        # Columns to group by: everything except metrics, std columns and columns to be averaged
        std_columns = [f"{col}_std" for col in metric_columns if f"{col}_std" in df.columns]
        
        cols_to_process = metric_columns + std_columns
        groupby_columns = [col for col in df.columns if col not in cols_to_process and col not in average_columns]

        # Aggregation dict for metrics
        agg_dict = {col: "mean" for col in metric_columns}
        
        # For std columns, we need to calculate sqrt(sum(std_i^2)) / N
        def std_agg(x):
            n = len(x)
            if n == 0:
                return 0
            return np.sqrt(np.sum(x**2)) / n

        for col in std_columns:
            agg_dict[col] = std_agg

        # Group and aggregate
        df_avg = df.groupby(groupby_columns, as_index=False).agg(agg_dict).round(3)
        return df_avg
        # """Average the DataFrame over specified columns."""
        # if not cfg.average_columns:
        #     return df
        # average_columns = [col for col in cfg.average_columns if col in df.columns]
        # remaining_columns = [
        #     col for col in df.columns if col not in average_columns and col not in cfg.metric_columns.keys()
        # ]
        # agg_dict = {col: "mean" for col in cfg.metric_columns.keys()}
        # return df.groupby(remaining_columns, as_index=False).agg(agg_dict).round(4)

    def _get_best_hyperparameter_id(self, reduced_df: pd.DataFrame, cfg) -> list:
        """
        Calculate the best hyperparameter ID based on metrics.

        Args:
            reduced_df (pd.DataFrame): The DataFrame filtered for a specific method.
            cfg: The configuration object.

        Returns:
            list: The best hyperparameter ID(s).
        """
        metric_columns = cfg.metric_columns
        remaining_columns = [col for col in reduced_df.columns if col not in metric_columns.keys() and col != "Task"]
        agg_dict = {col: "mean" for col in metric_columns.keys()}
        reduced_df = reduced_df.groupby(remaining_columns, as_index=False).agg(agg_dict)
        return reduced_df["HID"].iloc[0]

    def _create_hyperparameter_dataframe(self, method: str, hparam_ids: list, cfg) -> tuple:
        """
        Create a DataFrame and dictionary for hyperparameters.

        Args:
            method (str): The method name.
            hparam_ids (list): List of hyperparameter IDs.
            cfg: The configuration object.

        Returns:
            tuple: A tuple containing the hyperparameter DataFrame and dictionary.
        """
        hp_df = pd.DataFrame()
        hp_dict = {}

        for hparam_id in hparam_ids:
            hp = self._get_hyperparameters(method, hparam_id, include_hparams=cfg.include_hparams)
            hp_dict[hparam_id] = hp
            hp["HID"] = hparam_id
            hp["Method"] = method
            hp_df_new = pd.json_normalize(hp)

            # Drop columns with list values or None
            for col in hp_df_new.columns:
                value = hp_df_new[col].iloc[0]
                if isinstance(value, list) or value is None:
                    hp_df_new = hp_df_new.drop(columns=col)

            hp_df = pd.concat([hp_df, hp_df_new], ignore_index=True)

        return hp_df, hp_dict

    def _get_hyperparameter_differences(self, hp_df: pd.DataFrame) -> pd.DataFrame:
        """
        Identify differences between hyperparameters.

        Args:
            hp_df (pd.DataFrame): The hyperparameter DataFrame.

        Returns:
            pd.DataFrame: A DataFrame containing only the differing hyperparameters.
        """
        hp_df_diff = hp_df.copy()
        for col in hp_df_diff.columns:
            if len(hp_df_diff[col].unique()) == 1:
                hp_df_diff = hp_df_diff.drop(columns=col)
        check_cols = [col for col in hp_df_diff.columns if col not in ["HID", "index", "Method"]]
        return hp_df_diff.drop_duplicates(subset=check_cols, keep="first")

    def _output_hyperparameters(
        self, method: str, best_hparam_id, hp_df: pd.DataFrame, hp_df_diff, hp_dict, hparam_ids, cfg
    ) -> pd.DataFrame:
        if cfg.print_hyperparameters:
            print_hparams = cfg.print_hparams
            if print_hparams.only_best:
                print(f"Best Hyperparameters for {method} with HID {best_hparam_id}:")
                print(OmegaConf.to_yaml(hp_dict[best_hparam_id]))
            elif print_hparams.only_differences:
                print(f"Hyperparameter differences for {method}:")
                print(hp_df_diff)
            else:
                for hparam_id in hparam_ids:
                    print(f"Hyperparameters for {method} with HID {hparam_id}:")
                    print(OmegaConf.to_yaml(hp_dict[hparam_id]))

        if cfg.save_hyperparameters:
            save_hparams = cfg.save_hparams
            if save_hparams.only_best:
                return hp_df[hp_df["HID"].isin(best_hparam_id)]
            elif save_hparams.only_differences and not hp_df_diff.empty:
                return hp_df_diff
            else:
                return hp_df

    def _extract_uncertainty_scores(
        self,
        task_results: dict,
        best_values: dict,
        show_scores: dict,
        combine_scores: str = "median",
    ) -> dict:
        def convert_scores(scores_by_threshold: dict, max_episode_length: int) -> np.ndarray:
            # Transpose the combined scores to have a list of lists (steps as rows)
            scores = [[] for _ in range(max_episode_length)]
            for arr in scores_by_threshold:
                for i, value in enumerate(arr):
                    scores[i].append(value)

            # Apply the specified aggregation method (e.g., mean or median)
            aggregated_scores = []
            for i in range(len(scores)):
                if len(scores[i]) == 0:
                    continue
                if combine_scores == "mean":
                    new_score = np.mean(scores[i])
                elif combine_scores == "median":
                    new_score = np.median(scores[i])
                else:
                    raise ValueError(f"Unknown combine scores method: {combine_scores}")
                aggregated_scores.append(new_score)
            return np.array(aggregated_scores)

        best_window = best_values["Window"]
        if isinstance(best_window, str) and best_window.isdigit():
            best_window = int(best_window)
        data = {
            "successful_test_rollouts": task_results["successful_test_rollouts"],
            "ood_test_rollouts": task_results["ood_test_rollouts"],
            "id_test_rollouts": task_results["id_test_rollouts"],
            "max_episode_length": task_results["max_episode_length"],
            "scores_by_threshold": task_results["test_scores_by_threshold"][best_values["Threshold"]][
                best_values["Quantile"]
            ][best_window],
        }
        data_new = {}
        for key in show_scores.test.keys():
            if not show_scores.test[key]:
                continue
            mask = np.ones(len(data["successful_test_rollouts"]), dtype=bool)
            mask = mask & data["successful_test_rollouts"] if "success" in key else mask
            mask = mask & data["id_test_rollouts"] if "id_" in key else mask
            mask = mask & ~data["successful_test_rollouts"] if "fail" in key else mask
            mask = mask & data["ood_test_rollouts"] if "ood_" in key else mask

            if sum(mask) == 0:
                continue

            scores_by_threshold = [score for i, score in enumerate(data["scores_by_threshold"]) if mask[i]]

            data_new["test_" + key] = convert_scores(
                scores_by_threshold,
                data["max_episode_length"],
            )
        if show_scores.calibration and "calibration_scores_by_threshold" in data:
            scores_by_threshold = task_results["calibration_scores_by_threshold"][best_values["Threshold"]][
                best_values["Quantile"]
            ][best_window]
            data_new["calibration"] = convert_scores(
                scores_by_threshold,
                data["max_episode_length"],
            )

        # data_new["threshold"] = data["calibration_thresholds"] if not normalize else 1
        if show_scores.threshold:
            data_new["threshold"] = 1

        return data_new

    def plot_quantile_impact(self, **kwargs):
        """Plot the impact of quantiles (1-delta) on metrics, averaged over tasks, for the best/averaged thresholds."""
    
        # Update and unpack config
        cfg = self.cfg.copy()
        OmegaConf.set_struct(cfg, False)
        cfg.update(kwargs)
        cfg.update(cfg.quantile_impact)
        OmegaConf.set_struct(cfg, True)

        print(cfg.create_plots)

        if not cfg.create_plots:
            return

        # Load and filter DataFrame
        df = self._load_dataframe()
        df = self._filter_dataframe(df=df, cfg=cfg)
        df = self._average_dataframe(df=df, cfg=cfg)


        metrics = cfg.metrics_to_plot
        thresholds = cfg.thresholds_to_plot
        methods = cfg.filter_values.Method

        # Define per-metric y-limits dynamically
        ylims = {}
        for metric in metrics:
            metric_min = df[metric].min()
            metric_max = df[metric].max()
            padding = (metric_max - metric_min) * 0.1
            ylims[metric] = (metric_min - padding, metric_max + padding)


        colors = ['#2ca02c', '#1f77b4', '#d62728']

        threshold_mappings = {
            "tvt_cp_band": "CP band",
            "tvt_quantile": "time-varying",
            "ct_quantile": "CP constant",
        }

        # Create subplot grid
        fig, axes = plt.subplots(
            len(thresholds),
            len(metrics),
            figsize=(5 * len(metrics), 6 * len(thresholds)),
            sharex=False,
            sharey=False
        )

        if len(thresholds) == 1:
            axes = np.expand_dims(axes, axis=0)
        if len(metrics) == 1:
            axes = np.expand_dims(axes, axis=1)

        for j, threshold in enumerate(thresholds):
            threshold_df = df[df["Threshold"] == threshold]
            for i, metric in enumerate(metrics):
                ax = axes[j, i]
                ax.set_title(f"Threshold: {threshold_mappings.get(threshold, threshold)}",
                            fontsize=20)
                ax.set_xlabel("Quantile $1-\\delta$", fontsize=20)
                ax.set_ylabel(metric, fontsize=20)
                ax.tick_params(axis='x', labelsize=18)
                ax.tick_params(axis='y', labelsize=18)
                ax.set_ylim(ylims[metric])
                ax.grid(True, linestyle="--", alpha=0.7)

                std_col = f"{metric}_std"
                for k, method in enumerate(methods):
                    method_df = threshold_df[threshold_df["Method"] == method]
                    if metric not in method_df.columns:
                        continue

                    # Group by quantile and compute mean/std
                    def std_agg(x):
                        n = len(x)
                        if n == 0:
                            return 0
                        return np.sqrt(np.sum(x**2)) / n

                    agg_dict = {metric: "mean"}
                    if std_col in method_df.columns:
                        agg_dict[std_col] = std_agg
                    
                    grouped = method_df.groupby("Quantile").agg(agg_dict).reset_index()
                    grouped = grouped.sort_values("Quantile")
                    
                    yerr = grouped[std_col] if std_col in grouped.columns else None

                    ax.errorbar(
                        grouped["Quantile"],
                        grouped[metric],
                        yerr=yerr,
                        marker='o',
                        label=cfg.method_name_mapping.get(method, method),
                        color=colors[k % len(colors)],
                        capsize=4,
                        linewidth=2
                    )

        # Shared legend below the plot
        handles, labels = axes[-1, -1].get_legend_handles_labels()
        fig.legend(handles, labels, loc='lower center', bbox_to_anchor=(0.5, -0.05), ncol=len(methods), fontsize=20)

        # Tighter layout
        plt.tight_layout()
        plt.subplots_adjust(wspace=0.3, hspace=0.4, bottom=0.2)


        # Save figure
        save_dir = os.path.join(self.results_dir, "quantile_plots")
        os.makedirs(save_dir, exist_ok=True)
        plot_filename = os.path.join(save_dir, "quantile_impact.pdf")
        print("Saving quantile impact plot to:", plot_filename)
        plt.savefig(plot_filename, dpi=300, format="pdf", bbox_inches='tight')
        # plt.close()

    def plot_window_impact(self, **kwargs):
        """Plot the impact of window sizes on metrics, averaged over tasks, for the best/averaged quantiles."""
        
        # Update and unpack config
        cfg = self.cfg.copy()
        OmegaConf.set_struct(cfg, False)
        cfg.update(kwargs)
        cfg.update(cfg.window_impact)
        cfg.exclude_all = True
        OmegaConf.set_struct(cfg, True)

        if not cfg.create_plots:
            return

        # Load and filter DataFrame
        df = self._load_dataframe()
        if cfg.exclude_all:
            df = df[~df["Window"].astype(str).str.contains("all", case=False, na=False)]

        df = self._filter_dataframe(df=df, cfg=cfg)
        df = self._average_dataframe(df=df, cfg=cfg)

        # Exclude windows with ranges like "1/2"
        df = df[~df["Window"].astype(str).str.contains("/", case=False, na=False)]

        metrics = cfg.metrics_to_plot
        thresholds = cfg.thresholds_to_plot
        methods = cfg.filter_values.Method
        windows_all = sorted(
            df["Window"].unique(),
            key=lambda w: float(w.split("/")[0]) if isinstance(w, str) and "/" in w else float(w)
        )

        # Define per-metric y-limits
        ylims = [(0.4, 0.7), (0.45, 0.85), (0.0, 0.7), (0, 1)]  # Match metrics order

        colors = ['#2ca02c', '#1f77b4', '#d62728']

        threshold_mappings = {
            "tvt_cp_band": "CP band",
            "tvt_quantile": "time-varying",
            "ct_quantile": "CP constant",
        }

        # Create subplot grid
        fig, axes = plt.subplots(
            len(thresholds),
            len(metrics),
            figsize=(5 * len(metrics), 6 * len(thresholds)),
            sharex=False,
            sharey=False
        )

        if len(thresholds) == 1:
            axes = np.expand_dims(axes, axis=0)
        if len(metrics) == 1:
            axes = np.expand_dims(axes, axis=1)

        for j, threshold in enumerate(thresholds):
            threshold_df = df[df["Threshold"] == threshold]
            for i, metric in enumerate(metrics):
                ax = axes[j, i]
                ax.set_title(f"Threshold: {threshold_mappings.get(threshold, threshold)}",
                            fontsize=20)
                ax.set_xlabel("Window size $w$", fontsize=20)
                ax.set_ylabel(metric, fontsize=20)
                # Set tick font
                ax.tick_params(axis='x', labelsize=20 - 2)
                ax.tick_params(axis='y', labelsize=20 - 2)
                ax.set_ylim(ylims[i])
                ax.set_xscale('symlog', linthresh=11)  # linthresh defines the transition from linear to log
                xticks = [1, 2, 3, 4, 5, 7, 9, 11, 13, 15, 20, 25, 30, 35, 40, 45, 50]
                ax.set_xticks(xticks)
                xtick_labels = [1, 2, 3, 4, 5, 7, 9, 11, "", "", 20, "", 30, "", "", "", 50]
                ax.set_xticklabels(xtick_labels)
                ax.grid(True, linestyle="--", alpha=0.7)

                std_col = f"{metric}_std"
                for k, method in enumerate(methods):
                    method_df = threshold_df[threshold_df["Method"] == method]
                    if metric not in method_df.columns or std_col not in method_df.columns:
                        continue  # Skip if either column is missing

                    # Group by window and compute mean and std
                    grouped = method_df.groupby("Window")[[metric, std_col]].mean().reset_index()

                    # Convert window to numeric
                    grouped["Window_numeric"] = grouped["Window"].apply(
                        lambda w: (float(w.split("/")[0]) + float(w.split("/")[1])) / 2
                        if isinstance(w, str) and "/" in w else float(w)
                    )
                    grouped = grouped.sort_values("Window_numeric")

                    ax.errorbar(
                        grouped["Window_numeric"],
                        grouped[metric],
                        yerr=grouped[std_col],
                        marker='o',
                        label=cfg.method_name_mapping.get(method, method),
                        color=colors[k % len(colors)],
                        capsize=4,
                        linewidth=2
                    )

        # Shared legend below center column
        center_col = len(metrics) // 2
        axes[-1, center_col].legend(
            fontsize=20,
            loc='upper center',
            bbox_to_anchor=(-0.15, -0.2),
            ncol=len(methods)
        )

        # Tighter layout
        plt.tight_layout()
        plt.subplots_adjust(wspace=0.25, hspace=0.35)

        # Save figure
        save_dir = os.path.join(self.results_dir, "window_plots")
        os.makedirs(save_dir, exist_ok=True)
        plot_filename = os.path.join(save_dir, "window_impact_all.pdf")
        plt.savefig(plot_filename, dpi=300, format="pdf")
        # plt.close()


    def plot_threshold_impact(self, **kwargs):
        """Plot the impact of thresholds on metrics, averaged over tasks, for the best/averaged window sizes."""

        # Update and unpack config
        cfg = self.cfg.copy()
        OmegaConf.set_struct(cfg, False)
        cfg.update(kwargs)
        cfg.update(cfg.threshold_impact)
        cfg.exclude_all = True
        OmegaConf.set_struct(cfg, True)

        if not cfg.create_plots:
            return

        # Load and filter DataFrame
        df = self._load_dataframe()
        if cfg.exclude_all:
            df = df[~df["Window"].astype(str).str.contains("all", case=False, na=False)]

        df = self._filter_dataframe(df=df, cfg=cfg)
        df = self._average_dataframe(df=df, cfg=cfg)

        # Exclude windows with ranges like "1/2"
        df = df[~df["Window"].astype(str).str.contains("/", case=False, na=False)]

        metrics = cfg.metrics_to_plot
        thresholds = cfg.thresholds_to_plot
        methods = cfg.filter_values.Method
        windows_all = sorted(df["Window"].unique(), key=lambda w: float(w))

        # Define per-metric y-limits
        ylims = [(0.4, 0.7), (0.45, 0.85), (0.0, 0.65), (0.0, 1.0)]  # adjust per metric if needed

        colors = ['#2ca02c', '#1f77b4', '#d62728']  # one color per threshold
        threshold_mappings = {
            "tvt_cp_band": "CP band",
            "tvt_quantile": "time-varying",
            "ct_quantile": "CP constant",
        }

        # Create subplot grid: rows = methods, columns = metrics
        fig, axes = plt.subplots(
            nrows=len(methods),
            ncols=len(metrics),
            figsize=(5 * len(metrics), 6 * len(methods)),
            sharex=False,
            sharey=False
        )

        if len(methods) == 1:
            axes = np.expand_dims(axes, axis=0)
        if len(metrics) == 1:
            axes = np.expand_dims(axes, axis=1)

        for r, method in enumerate(methods):
            method_df = df[df["Method"] == method]
            for c, metric in enumerate(metrics):
                ax = axes[r, c]
                ax.set_title(f"{cfg.method_name_mapping.get(method, method)} - {metric}", fontsize=20)
                ax.set_xlabel("Window size $w$", fontsize=20)
                ax.set_ylabel(metric, fontsize=20)
                ax.tick_params(axis='x', labelsize=20 - 2)
                ax.tick_params(axis='y', labelsize=20 - 2)
                ax.set_ylim(ylims[c])
                ax.set_xscale('symlog', linthresh=11)
                xticks = [1, 2, 3, 4, 5, 7, 9, 11, 13, 15, 20, 25, 30, 35, 40, 45, 50]
                ax.set_xticks(xticks)
                xtick_labels = [1, 2, 3, 4, 5, 7, 9, 11, "", "", 20, "", 30, "", "", "", 50]
                ax.set_xticklabels(xtick_labels)
                ax.grid(True, linestyle="--", alpha=0.7)

                std_col = f"{metric}_std"
                for t_idx, threshold in enumerate(thresholds):
                    thresh_df = method_df[method_df["Threshold"] == threshold]
                    if metric not in thresh_df.columns or std_col not in thresh_df.columns:
                        continue

                    grouped = thresh_df.groupby("Window")[[metric, std_col]].mean().reset_index()
                    grouped["Window_numeric"] = grouped["Window"].apply(float)
                    grouped = grouped.sort_values("Window_numeric")

                    ax.errorbar(
                        grouped["Window_numeric"],
                        grouped[metric],
                        yerr=grouped[std_col],
                        marker='o',
                        label=threshold_mappings.get(threshold, threshold),
                        color=colors[t_idx % len(colors)],
                        capsize=4,
                        linewidth=2
                    )

        # Shared legend below center column
        center_col = len(metrics) // 2
        axes[-1, center_col].legend(
            fontsize=20,
            loc='upper center',
            bbox_to_anchor=(-0.15, -0.2),
            ncol=len(thresholds)
        )

        plt.tight_layout()
        plt.subplots_adjust(wspace=0.25, hspace=0.35)

        # Save figure
        plot_filename = os.path.join(self.results_dir, "threshold_impact_all.pdf")
        plt.savefig(plot_filename, dpi=300, format="pdf")
        plt.close()


    def plot_rollout_type_stats(self, **kwargs):
        """Plot the average scores by rollout type for each method."""

        # Update and unpack the config
        cfg = self.cfg.copy()
        OmegaConf.set_struct(cfg, False)
        cfg.update(kwargs)
        cfg.update(cfg.rollout_type_stats)
        OmegaConf.set_struct(cfg, True)
        # Set drop_after_best to True
        cfg.drop_after_best = False

        if not cfg.create_plots:
            return

        # Load the DataFrame
        df = self._load_dataframe()

        # Filter and process the DataFrame
        df = self._filter_dataframe(df=df, cfg=cfg)
        df = self._average_dataframe(df=df, cfg=cfg)
        df = self._sort_and_reorder_dataframe(df=df, cfg=cfg)

        # Load the results
        results = self._load_results()

        def extract_scores(scores: list, mask: np.ndarray) -> list:
            """Extract scores based on the mask and convert to numpy array."""
            if sum(mask) == 0:
                return np.array([])
            return np.hstack([score for i, score in enumerate(scores) if mask[i]])

        for method in df["Method"].unique():
            save_dir = os.path.join(self.results_dir, "rollout_type_stats", method)
            os.makedirs(save_dir, exist_ok=True)

            # Get the data for the method (first row)
            first_row = df[df["Method"] == method].iloc[0]
            results_data = results[method][first_row["HID"]]
            task_keys = [key for key in results_data.keys() if key != "hparams"]
            task_keys = (
                [key for key in task_keys if key in cfg.filter_values.Task]
                if cfg.filter_values.Task is not None and cfg.filter_values.Task
                else task_keys
            )
            # print(f"Method: {method}, Tasks: {task_keys}")
            window = first_row["Window"]
            if isinstance(window, str) and window.isdigit():
                window = int(window)

            # Initialize arrays for scores and rollout masks
            test_scores_by_threshold = []
            successful_test_rollouts = []
            ood_test_rollouts = []
            id_test_rollouts = []

            # Extract data for all tasks
            for task in task_keys:
                task_data = results_data[task]
                test_scores_by_threshold.extend(
                    task_data["test_scores_by_threshold"][first_row["Threshold"]][float(first_row["Quantile"])][window]
                )
                successful_test_rollouts.extend(task_data["successful_test_rollouts"])
                ood_test_rollouts.extend(task_data["ood_test_rollouts"])
                id_test_rollouts.extend(task_data["id_test_rollouts"])

            # Convert to numpy arrays and stack
            successful_test_rollouts = np.hstack(successful_test_rollouts)
            ood_test_rollouts = np.hstack(ood_test_rollouts)
            id_test_rollouts = np.hstack(id_test_rollouts)

            # Extract scores for each rollout type
            rollout_masks = {
                "Success ID": id_test_rollouts & successful_test_rollouts,
                "Success OOD": ood_test_rollouts & successful_test_rollouts,
                "Fail ID": id_test_rollouts & ~successful_test_rollouts,
                "Fail OOD": ood_test_rollouts & ~successful_test_rollouts,
            }
            data = {key: extract_scores(test_scores_by_threshold, mask) for key, mask in rollout_masks.items()}

            ylabel = "Score $s(O_t,A_t)$ / Threshold $\gamma_t$"

            plotname = f"violin_plot_{method}"
            # plotname = f"violin_plot_{first_row['Threshold'].replace(' ', '_')}_norm"

            # Filter outliers for each rollout type
            data = {key: filter_outliers(data[key]) for key in data.keys() if len(data[key]) > 0}
            
            # Normalize the scores by the mean of the "OOD F" scores
            ood_f_mean = np.mean(data["Fail OOD"]) if len(data["Fail OOD"]) > 0 else np.mean(data["Fail ID"])
            for key in data.keys():
                if len(data[key]) > 0:
                    data[key] = data[key] / ood_f_mean
                    data[key] = data[key][data[key] <= 3]  # Remove values above 2.5 after normalization

            ood_f_mean = np.mean(data["Fail OOD"]) if len(data["Fail OOD"]) > 0 else np.mean(data["Fail ID"])
            for key in data.keys():
                if len(data[key]) > 0:
                    data[key] = data[key] / ood_f_mean

            # Create the violin plot
            fig, ax = plt.subplots(figsize=(12, 6))
            ax.set_title(f"{cfg.method_name_mapping[method]}", fontsize=20)
            ax.set_xlabel("Rollout Type", fontsize=20)
            ax.set_ylabel(ylabel, fontsize=20)
            ax.grid(True, linestyle="--", alpha=0.7)

            # Extract plots info
            quantiles = []
            for quantile in [0.9]:
                quantiles.append(1 - quantile)
                quantiles.append(quantile)

            category_colors = {
                "Success ID": ("#1f77b4", 0.7),    # solid blue
                "Success OOD": ("#1f77b4", 0.4),    # light blue
                "Fail ID": ("#d62728", 0.7),      # solid red
                "Fail OOD": ("#d62728", 0.4),     # light red
            }
            # Create violin plots
            for i, rollout_type in enumerate(data.keys()):
                if rollout_type not in data or len(data[rollout_type]) == 0:
                    continue
                scores = data[rollout_type]
                color, alpha = category_colors[rollout_type]
                violin_parts = ax.violinplot(
                    scores,
                    positions=[i],
                    showmeans=True,
                    showmedians=False,
                    showextrema=False,
                    quantiles=quantiles,
                    points=300,
                )
                for pc in violin_parts["bodies"]:
                    pc.set_facecolor(color)
                    pc.set_edgecolor(color)
                    pc.set_alpha(alpha)

                if "cmeans" in violin_parts:
                    violin_parts["cmeans"].set_color(color)  # Mean line
                if "cquantiles" in violin_parts:
                    violin_parts["cquantiles"].set_color(color)  # Quantile lines
                if "cbars" in violin_parts:
                    violin_parts["cbars"].set_color(color)  # Vertical bars

                ax.text(
                    i,
                    np.mean(scores),
                    f"Mean: {np.mean(scores):.3f}",
                    horizontalalignment="center",
                    verticalalignment="bottom",
                    fontsize=20,
                )

                for quantile in set(quantiles):
                    q_value = np.quantile(scores, quantile)
                    ax.text(
                        i,
                        q_value,
                        f"Q{quantile:.2f}: {(q_value - scores.mean()):.3f}",
                        horizontalalignment="center",
                        verticalalignment="bottom",
                        fontsize=20,
                    )

            ax.set_xticks(range(len(data.keys())))
            ax.tick_params(axis="y", labelsize=20)
            ax.set_xticklabels(list(data.keys()), fontsize=20)

            ax.set_ylim(bottom=0.0, top=2.6)

            # Adjust layout and save the figure
            plt.tight_layout()
            plot_filename = os.path.join(save_dir, plotname + ".pdf")
            fig.savefig(plot_filename, dpi=500, format="pdf")
            plt.close(fig)

    def accumulate_seed_results(self, results: list) -> dict:
        """Compute a single result dictionary from a list of result dictionaries by averaging each metric over seeds and adding std values for each metric.
        Args:
            results (list): List of result dictionaries (results[task][method][test_metrics][threshold_style][quantile][window_size]), each containing metrics for different seeds."""
        if not results:
            return {}
        if not isinstance(results, list):
            raise ValueError("Input results should be a list of dictionaries.")
        if len(results) == 1:
            return results[0]
        # Initialize the result dictionary
        acc = results[0].copy()  # Start with the first result as the base
        for task in acc.keys():
            for method in acc[task].keys():
                for threshold_style in acc[task][method]["test_metrics"].keys():
                    for quantile in acc[task][method]["test_metrics"][threshold_style].keys():
                        for window_size in acc[task][method]["test_metrics"][threshold_style][quantile].keys():
                                # Initialize the accumulator for the current metric
                                dict_of_metric_list = {}
                                for metric in acc[task][method]["test_metrics"][threshold_style][quantile][
                                    window_size
                                ].keys():
                                    dict_of_metric_list[metric] = [
                                        results[i][task][method]["test_metrics"][threshold_style][quantile][window_size][
                                            metric
                                        ]
                                        for i in range(len(results))
                                    ]
                                    
                                # Calculate the mean and std for each important metric
                                for metric, values in dict_of_metric_list.items():
                                    if isinstance(values[0], dict):
                                        continue  # Skip non-numerical metrics
                                    acc[task][method]["test_metrics"][threshold_style][quantile][window_size][metric] = (
                                        np.mean(values)
                                    )
                                    acc[task][method]["test_metrics"][threshold_style][quantile][window_size][
                                        f"{metric}_std"
                                    ] = np.std(values)
        return acc


# Function to filter outliers based on the IQR method
def filter_outliers(data, threshold=3):
    """Remove outliers from the data using the IQR method."""
    q1 = np.percentile(data, 25)  # First quartile (25th percentile)
    q3 = np.percentile(data, 75)  # Third quartile (75th percentile)
    iqr = q3 - q1  # Interquartile range
    upper_bound = q3 + threshold * iqr
    upper_bound = max(upper_bound, 5.0)  # Set minimum upper bound to 5.0
    upper_bound = min(upper_bound, 20.0)  # Set maximum upper bound to 20.0
    return np.array([x for x in data if 0.0 <= x <= upper_bound])
