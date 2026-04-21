import os
import sys
import pathlib
import numpy as np
import pickle as pkl

from PIL import Image
from typing import Union
import scipy.stats as stats

ROOT_DIR = str(pathlib.Path(__file__).parent.parent)
sys.path.append(ROOT_DIR)
os.chdir(ROOT_DIR)


def _calculate_accuracy(TP: int, TN: int, FP: int, FN: int) -> tuple[float, float, float, float]:
    """Calculate accuracy metrics from confusion matrix values.

    Returns:
        TPR: True Positive Rate (Sensitivity)

        TNR: True Negative Rate (Specificity)

        accuracy: Overall accuracy

        balanced_accuracy: Balanced accuracy
    """
    TPR = TP / (TP + FN) if (TP + FN) > 0 else 0
    TNR = TN / (FP + TN) if (FP + TN) > 0 else 0
    accuracy = (TP + TN) / (TP + TN + FP + FN)
    balanced_accuracy = (TPR + TNR) / 2
    return TPR, TNR, accuracy, balanced_accuracy


def _calculate_confusion_matrix(
    failures_detected: Union[list, np.ndarray], successful_rollouts: Union[list, np.ndarray]
):
    """Calculate confusion matrix values from detected failures and successful rollouts.

    Args:
        failures_detected: List or numpy array of detected failures in rollouts.
        successful_rollouts: List or numpy array of successful rollouts.

    Returns:
        TP, TN, FP, FN: True Positive, True Negative, False Positive, False Negative counts.
    """
    if isinstance(failures_detected, list):
        failures_detected = np.array(failures_detected)
    if isinstance(successful_rollouts, list):
        successful_rollouts = np.array(successful_rollouts)
    if len(failures_detected) != len(successful_rollouts) or failures_detected.shape != successful_rollouts.shape:
        raise ValueError("Length and shape of failures_detected and successful_rollouts must be the same.")
    # for i in range(len(failures_detected)):
    #     if (not isinstance(failures_detected[i], bool) or not isinstance(successful_rollouts[i], bool)) and (
    #         not isinstance(successful_rollouts[i], bool) or not isinstance(successful_rollouts[i], bool)
    #     ):
    #         raise ValueError("Elements of failures_detected and successful_rollouts must be boolean values.")
    # Calculate confusion matrix values
    FP = np.sum(failures_detected & successful_rollouts)
    TN = np.sum(~failures_detected & successful_rollouts)
    TP = np.sum(failures_detected & ~successful_rollouts)
    FN = np.sum(~failures_detected & ~successful_rollouts)
    return TP, TN, FP, FN


def _calculate_twa(detected_failure_in_episode, successful_rollouts, detection_times):
    """
    Calculate the timestep-wise accuracy metrics.

    Args:
        detected_failure_in_episode: List of booleans indicating if a failure was detected in the episode.
        successful_rollouts: List of booleans indicating if the episode was successful.
        detected_times: List of indices where the failure was detected.

    Returns:
        TWA: Timestep-wise accuracy metric.
    """
    num_successful_rollouts = np.sum(successful_rollouts)
    num_failed_rollouts = len(successful_rollouts) - num_successful_rollouts
    # Calculate the timestep-wise accuracy
    TWA = 0.0
    # Counter for the detection times as they are only for failed rollouts that were detected
    count = 0
    for i in range(len(detected_failure_in_episode)):
        if detected_failure_in_episode[i] and not successful_rollouts[i]:
            TWA += (1 - detection_times[count]) / num_failed_rollouts
            count += 1
        elif not detected_failure_in_episode[i] and successful_rollouts[i]:
            TWA += 1 / num_successful_rollouts
    return TWA * 0.5


def _translate_scores(uncertainty_scores):
    """Translate uncertainty scores to a dictionary with keys as step indices."""
    scores_for_all_steps = {}
    for episode in uncertainty_scores:
        for i, step in enumerate(episode):
            if i not in scores_for_all_steps:
                scores_for_all_steps[i] = []
            scores_for_all_steps[i].append(step)
    return scores_for_all_steps


def compute_thresholds(threshold_style, quantile, uncertainty_scores):
    """Compute thresholds based on the specified style and quantile."""
    # Get the maximum uncertainty scores in each episode
    max_uncertainty_scores = [max(uncertainty_score) for uncertainty_score in uncertainty_scores]
    scores_for_steps = _translate_scores(uncertainty_scores)

    if threshold_style == "ct_quantile":
        threshold = np.quantile(max_uncertainty_scores, quantile)
    elif threshold_style == "tvt_quantile":
        threshold = []
        for t in scores_for_steps:
            threshold.append(np.quantile(scores_for_steps[t], quantile))
    elif threshold_style == "tvt_cp_band":
        threshold = _calculate_cp_band_threshold(uncertainty_scores, quantile)
    else:
        raise ValueError(f"Unknown threshold style: {threshold_style}")
    return threshold


def _calculate_cp_band_threshold(uncertainty_scores: list, quantile: float) -> list:
    """Calculate the CP band threshold based on the uncertainty scores.

    Implementation based on the paper "Can We Detect Failures Without Failure Data? Uncertainty-Aware Runtime Failure Detection for Imitation Learning Policies" by Chen Xu and Al. URL: {https://arxiv.org/abs/2503.08558}
    """
    # Get longest episode and use it in set 1 (so that the mean trajectory is as long as possible)
    longest_index = max(range(len(uncertainty_scores)), key=lambda i: len(uncertainty_scores[i]))
    longest_episode = uncertainty_scores.pop(longest_index)
    max_episode_length = len(longest_episode)

    # Split the uncertainty scores into two disjoint sets (N1 = N2 + 1)
    N1 = len(uncertainty_scores) // 2
    N2 = len(uncertainty_scores) - N1
    uncertainty_scores_1 = [longest_episode] + uncertainty_scores[:N1]
    uncertainty_scores_2 = uncertainty_scores[N1:]
    N1 += 1

    # Translate the scores for set 1
    scores_for_steps_1 = _translate_scores(uncertainty_scores_1)

    # Compute the mean trajectory on set 1
    mean_trajectory_1 = [np.mean(scores_for_steps_1[t]) for t in scores_for_steps_1.keys()]

    # Determine set H
    if (N1 + 1) * quantile > N1:
        H = list(range(N1))
    else:
        # Determine gamma
        deviations = np.array(
            [
                max(
                    [
                        abs(uncertainty_scores_1[m][t] - mean_trajectory_1[t])
                        for t in range(len(uncertainty_scores_1[m]))
                    ]
                )
                for m in range(N1)
            ]
        )
        gamma = np.quantile(deviations, quantile)
        # Determine set H
        H = np.where(deviations <= gamma)[0].tolist()

    # Compute the modulation function scalA(t)
    filtered_uncertainty_scores_1 = [uncertainty_scores_1[j] for j in H]
    filtered_scores_for_steps_1 = _translate_scores(filtered_uncertainty_scores_1)
    scalA = []
    for t in range(len(mean_trajectory_1)):
        if t not in filtered_scores_for_steps_1:
            scalA.append(1 / (t))  # t is then the episode length
        else:
            scalA.append(max(abs(np.array(filtered_scores_for_steps_1[t]) - mean_trajectory_1[t])))
    # Set zero values to 1/T where T:episode length (happens typically only if there is only one episode with the respective step)
    scalA = [1 / max_episode_length if val < 1e-7 else val for val in scalA]

    # Compute the max deviations Dj for set 2
    deviations = []
    for episode in uncertainty_scores_2:
        max_deviation = max(
            [
                (mean_trajectory_1[t] - episode[t]) / scalA[t]
                for t in range(min(len(episode), len(scalA), len(mean_trajectory_1)))
            ]
        )
        deviations.append(max_deviation)

    # Compute the band width h as the quantile of deviations
    h = np.quantile(deviations, quantile)

    # Compute the time-varying thresholds as upper bounds
    threshold = [mean_trajectory_1[t] + h * scalA[t] for t in range(min(len(mean_trajectory_1), len(scalA)))]
    return threshold


def normalize_scores_by_threshold(
    uncertainty_scores,
    thresholds,
    cfg,
):
    """Normalize the uncertainty scores by the thresholds.
    Args:
        uncertainty_scores: List of uncertainty scores for each episode.
        thresholds: Thresholds for normalization. Can be a list, numpy array, float, or int.
        extend_thresholds: How to extend the thresholds if they are shorter than the scores.
            "last": Extend the last threshold to match the length of the scores.
            "mean": Extend the mean of the thresholds to match the length of the scores.
    """
    info = dict(cfg.get("handle_zero_thresholds", {"style": "cond"}))
    max_episode_length = max([len(scores) for scores in uncertainty_scores])
    # convert constant thresholds to lists for uniformity
    if isinstance(thresholds, (float, int)):
        thresholds = [thresholds] * max_episode_length
    # convert lists to numpy arrays for uniformity
    if len(thresholds) < max_episode_length:
        if cfg.get("extend_thresholds", "mean") == "last":
            thresholds = np.concatenate([thresholds, [thresholds[-1]] * (max_episode_length - len(thresholds))])
        elif cfg.get("extend_thresholds", "mean") == "mean":
            thresholds = np.concatenate([thresholds, [np.mean(thresholds)] * (max_episode_length - len(thresholds))])
        else:
            raise ValueError("extend_thresholds must be 'last' or 'mean'.")
    thresholds = np.array(thresholds)

    if info["style"] == "cap_norm_scores":
        scores_by_threshold = normalize_scores_with_cap(
            uncertainty_scores, thresholds
        )
    elif info["style"] == "clip_threshold":
        thresholds = np.clip(thresholds, 1e-6, None)
        scores_by_threshold = [
            np.array(scores) / thresholds[: len(scores)] if len(scores) > 0 else np.array(scores)
            for scores in uncertainty_scores
        ]
    elif info["style"] == "add_small_score":
        assert min(thresholds) > 0, "Thresholds must be greater than 0."
        scores_by_threshold = [
            np.array(scores) / thresholds[: len(scores)] if len(scores) > 0 else np.array(scores)
            for scores in uncertainty_scores
        ]
    else:
        scores_by_threshold = normalize_scores_conditionally(
            uncertainty_scores, thresholds
        )

    return scores_by_threshold


def normalize_scores_with_cap(scores, thresholds, cap=5.0, clip_threshold=1e-6):
    """Normalize scores by thresholds with an upper cap on the normalized values."""
    thresholds = np.clip(thresholds, clip_threshold, None)
    # Normalize the scores by the thresholds
    scores_by_threshold = [np.array(s) / thresholds[: len(s)] if len(s) > 0 else np.array(s) for s in scores]
    # cap the normalized scores
    scores_by_threshold = [
        np.clip(scores, 0, cap) if len(scores) > 0 else np.array(scores) for scores in scores_by_threshold
    ]
    return scores_by_threshold


def normalize_scores_conditionally(scores, thresholds, small_threshold=1e-3, cap=3):
    """Normalize scores conditionally to avoid distortion from small thresholds."""
    normalized_scores = []
    for episode in scores:
        episode_norm = []
        for t in range(len(episode)):
            score = episode[t]
            threshold = thresholds[t]
            if threshold > score:
                new_score = score / threshold
            # Handle zero thresholds
            elif threshold <= 1e-8:
                new_score = 0.0 if score <= 1e-8 else cap
            # Handle small thresholds
            elif threshold <= small_threshold and score >= small_threshold * 5:
                new_score = cap
            elif score > 30 * threshold:
                new_score = cap * 2
            else:
                new_score = score / threshold
            episode_norm.append(new_score)
        normalized_scores.append(np.array(episode_norm))
    return normalized_scores


def _get_avg_episode_stats_by_type(scores_by_threshold: list, dataset_stats: dict, alt=True):
    """Calculate average episode statistics by type (ID/ood) and success/failure.
    Args:
        scores_by_threshold: List of normalized uncertainty scores for each episode.
        dataset_stats: Dictionary containing dataset statistics, including:
            - id_rollouts: List of booleans indicating ID rollouts.
            - ood_rollouts: List of booleans indicating OOD rollouts.
            - successful_rollouts: List of booleans indicating successful rollouts.
    Returns:
        avg_episode_stats_by_type: Dictionary containing average episode statistics by type.
    """
    if alt:
        return _get_avg_episode_stats_by_type_alt(scores_by_threshold, dataset_stats)
    id_rollouts = dataset_stats["id_rollouts"]
    ood_rollouts = dataset_stats["ood_rollouts"]
    successful_rollouts = dataset_stats["successful_rollouts"]
    avg_episode_stats_by_type = {}

    def mask_scores(scores, mask, norm_factor=1):
        """Helper function to mask scores based on a boolean mask."""
        if norm_factor == 0:
            norm_factor = 1
        return np.concatenate([score / norm_factor for score, m in zip(scores, mask) if m])

    # Step 1: Calculate mean scores
    if sum(id_rollouts) > 0:
        avg_episode_stats_by_type["id_success"] = {
            "mean": np.mean(mask_scores(scores_by_threshold, id_rollouts & successful_rollouts))
        }
        avg_episode_stats_by_type["id_failure"] = {
            "mean": np.mean(mask_scores(scores_by_threshold, id_rollouts & ~successful_rollouts))
        }

    if sum(ood_rollouts) > 0:
        avg_episode_stats_by_type["ood_success"] = {
            "mean": np.mean(mask_scores(scores_by_threshold, ood_rollouts & successful_rollouts))
        }
        avg_episode_stats_by_type["ood_failure"] = {
            "mean": np.mean(mask_scores(scores_by_threshold, ood_rollouts & ~successful_rollouts))
        }

    # Step 2: Normalize mean scores by the maximum mean score
    max_mean_score = max(
        [
            avg_episode_stats_by_type[key]["mean"]
            for key in avg_episode_stats_by_type.keys()
            if avg_episode_stats_by_type[key]["mean"] > 0
        ]
    )
    if max_mean_score > 0:
        for key in avg_episode_stats_by_type.keys():
            avg_episode_stats_by_type[key]["mean"] /= max_mean_score

    return avg_episode_stats_by_type


def _get_avg_episode_stats_by_type_alt(scores_by_threshold: list, dataset_stats: dict):
    """Calculate average episode statistics by type (ID/ood) and success/failure.

    Args:
        scores_by_threshold: List of normalized uncertainty scores for each episode.
        dataset_stats: Dictionary containing dataset statistics, including:
            - id_rollouts: List of booleans indicating ID rollouts.
            - ood_rollouts: List of booleans indicating OOD rollouts.
            - successful_rollouts: List of booleans indicating successful rollouts.

    Returns:
        avg_episode_stats_by_type: Dictionary containing average episode statistics by type.
    """
    id_rollouts = dataset_stats["id_rollouts"]
    ood_rollouts = dataset_stats["ood_rollouts"]
    successful_rollouts = dataset_stats["successful_rollouts"]
    avg_episode_stats_by_type = {}

    def calculate_stats(scores):
        """Helper function to calculate mean, std, and percentiles."""
        if len(scores) == 0:
            return {
                "mean": 0,
                "std": 0,
                "percentiles": {10: 0, 25: 0, 50: 0, 75: 0, 90: 0},
            }
        scores_array = np.concatenate(scores)
        return {
            "mean": np.mean(scores_array),
            "std": np.std(scores_array),
            "percentiles": {
                10: np.percentile(scores_array, 10),
                25: np.percentile(scores_array, 25),
                50: np.percentile(scores_array, 50),  # Median
                75: np.percentile(scores_array, 75),
                90: np.percentile(scores_array, 90),
            },
        }

    def extract_scores(scores_by_threshold, rollouts_mask):
        """Helper function to extract scores for a given rollout type."""
        return [score for score, mask in zip(scores_by_threshold, rollouts_mask) if mask]

    # Calculate statistics for each rollout type
    if sum(id_rollouts) > 0:
        avg_episode_stats_by_type["id_success"] = calculate_stats(
            extract_scores(scores_by_threshold, id_rollouts & successful_rollouts)
        )
        avg_episode_stats_by_type["id_failure"] = calculate_stats(
            extract_scores(scores_by_threshold, id_rollouts & ~successful_rollouts)
        )

    if sum(ood_rollouts) > 0:
        avg_episode_stats_by_type["ood_success"] = calculate_stats(
            extract_scores(scores_by_threshold, ood_rollouts & successful_rollouts)
        )
        avg_episode_stats_by_type["ood_failure"] = calculate_stats(
            extract_scores(scores_by_threshold, ood_rollouts & ~successful_rollouts)
        )

    # Normalize mean and std by the maximum mean score
    max_mean_score = max(
        [
            avg_episode_stats_by_type[key]["mean"]
            for key in avg_episode_stats_by_type.keys()
            if avg_episode_stats_by_type[key]["mean"] > 0
        ],
        default=0,
    )
    if max_mean_score > 0:
        for key in avg_episode_stats_by_type.keys():
            # Normalize mean and std
            avg_episode_stats_by_type[key]["mean"] /= max_mean_score
            avg_episode_stats_by_type[key]["std"] /= max_mean_score

            # Recalculate percentiles on normalized scores
            if key == "id_success":
                normalized_scores = [
                    np.array(score) / max_mean_score
                    for score in extract_scores(scores_by_threshold, id_rollouts & successful_rollouts)
                ]
            elif key == "id_failure":
                normalized_scores = [
                    np.array(score) / max_mean_score
                    for score in extract_scores(scores_by_threshold, id_rollouts & ~successful_rollouts)
                ]
            elif key == "ood_success":
                normalized_scores = [
                    np.array(score) / max_mean_score
                    for score in extract_scores(scores_by_threshold, ood_rollouts & successful_rollouts)
                ]
            elif key == "ood_failure":
                normalized_scores = [
                    np.array(score) / max_mean_score
                    for score in extract_scores(scores_by_threshold, ood_rollouts & ~successful_rollouts)
                ]
            else:
                continue

            if len(normalized_scores) > 0:
                scores_array = np.concatenate(normalized_scores)
                avg_episode_stats_by_type[key]["percentiles"] = {
                    10: np.percentile(scores_array, 10),
                    25: np.percentile(scores_array, 25),
                    50: np.percentile(scores_array, 50),  # Median
                    75: np.percentile(scores_array, 75),
                    90: np.percentile(scores_array, 90),
                }
            else:
                # 데이터가 없을 경우 (예: 실패한 에피소드가 하나도 없을 때)
                # NaN이나 0으로 채우거나, 해당 키를 건너뛰어야 함
                avg_episode_stats_by_type[key]["percentiles"] = {
                    10: 0.0, 25: 0.0, 50: 0.0, 75: 0.0, 90: 0.0
                }

    return avg_episode_stats_by_type


def calculate_metrics(scores_by_threshold: list, dataset_stats: dict, detection_patience=0) -> dict:
    """Calculate the evaluation metrics for a list of episode infos.

    Args:
        scores_by_threshold: List of lists of uncertainty scores for each episode normalized by thresholds.
        successful_rollouts: List of booleans indicating whether the episode was successful or not.
        dataset_stats: Dictionary containing the dataset statistics including:
            - max_episode_length: Maximum episode length.
            - id_rollouts: List of booleans indicating whether the episode is an ID test rollout.
            - ood_rollouts: List of booleans indicating whether the episode is an OOD test rollout.
            - successful_rollouts: List of booleans indicating whether the episode was successful.
    Returns:
        metrics: Dictionary containing the evaluation metrics.
    """
    # Maximum episode length and successful rollouts
    max_episode_length = dataset_stats["max_episode_length"]
    successful_rollouts = dataset_stats["successful_rollouts"]
    detection_patience = max(0, detection_patience)

    detected_failure_in_episode = []
    detection_times = []
    for scores, success in zip(scores_by_threshold, successful_rollouts):
        # Check whether a failure was detected
        scores_above_threshold = np.array(scores) > 1
        detected_failure_in_episode.append(np.sum(scores_above_threshold) > detection_patience)

        # Append detection times if a failure was correctly detected
        if detected_failure_in_episode[-1] and not success:
            # Calculate the detection time
            detection_times.append(np.where(scores_above_threshold)[0][detection_patience] / (max_episode_length - 1))

    # Calculate the number of true positives, true negatives, false positives, and false negatives
    TP, TN, FP, FN = _calculate_confusion_matrix(detected_failure_in_episode, successful_rollouts)
    # calculate metrics
    TPR, TNR, accuracy, balanced_accuracy = _calculate_accuracy(TP, TN, FP, FN)
    avg_detection_time = np.mean(detection_times) if len(detection_times) > 0 else 1.0
    std_detection_time = np.std(detection_times) if len(detection_times) > 0 else 0.0

    avg_episode_stats_by_type = _get_avg_episode_stats_by_type(scores_by_threshold, dataset_stats)
    TWA = _calculate_twa(detected_failure_in_episode, successful_rollouts, detection_times)

    metrics = {
        "TPR": TPR,
        "TNR": TNR,
        "accuracy": accuracy,
        "balanced_accuracy": balanced_accuracy,
        "avg_detection_time": avg_detection_time,
        "std_detection_time": std_detection_time,
        "TWA": TWA,
        "avg_episode_stats_by_type": avg_episode_stats_by_type,
    }
    def round_dict_values(d:dict, num:int=5, to_type=None) -> dict:
        """Recursively round float values in a dictionary."""
        for k, v in d.items():
            if isinstance(v, dict):
                round_dict_values(v, num=num, to_type=to_type)
            elif isinstance(v, float):
                d[k] = round(v, num)
                if to_type is not None:
                    d[k] = to_type(d[k])
        return d
    # Round the metrics to 6 decimal places
    metrics = round_dict_values(metrics, 6, to_type=np.float32)

    return metrics


def save_videos_with_warning(task, warning_frames, episode_lengths, base_data_path):
    path = f"{base_data_path}/{task}/rollouts/test"

    save_path = f"{base_data_path}/results/videos_with_warnings/{task}/rollouts/test_frames"
    os.makedirs(save_path, exist_ok=True)

    k = 0

    # For each episode_XX.pkl in path, load the pickle file. 
    for file in sorted(os.listdir(path)):
        if file.endswith(".pkl"):
            with open(os.path.join(path, file), "rb") as f:
                data = pkl.load(f)

            # For each rollout, save all the frames as numbered jpgs into a folder in save_path that is named after the pickle file (without .pkl)
            # Do not use imageio
            episode_name = f"episode_{k:02d}"
            episode_path = os.path.join(save_path, episode_name)
            os.makedirs(episode_path, exist_ok=True)

            warning_timesteps_for_episode = warning_frames[k]
            
            # Add a red border to each frame where the index is in warning_timesteps_for_episode
            for i, frame_data in enumerate(data['rollout']):
                image = frame_data['rgb']   # Shape: (3, 240, 320), dtype: float32, range: [0.0, 1.0]
                if image.dtype != np.uint8:
                    image = (image * 255).astype(np.uint8)  # Convert to uint8
                if image.shape[2] != 3:
                    image = np.transpose(image, (1, 2, 0))  # Change to HWC format
                # Swap channels
                if task in ['pretzel', 'sorting', 'stacking']:
                    image = image[:, :, [2, 1, 0]]  # Convert RGB to BGR
                # Otherwise, not sure. Test different channel combinations
                # else:
                #     image = image[:, :, [1, 2, 0]]  # Convert RGB to BGR

                # Red border around the image if i in warning_timesteps_for_episode
                if i in warning_timesteps_for_episode:
                    relative_border_size = 0.03
                    border_size_x = int(min(image.shape[0], image.shape[1]) * relative_border_size)
                    image[0:border_size_x, :, 0] = 255  # Blue channel
                    image[0:border_size_x, :, 1] = 0    # Green channel
                    image[0:border_size_x, :, 2] = 0    # Red channel
                    image[-border_size_x:, :, 0] = 255
                    image[-border_size_x:, :, 1] = 0
                    image[-border_size_x:, :, 2] = 0
                    image[:, 0:border_size_x, 0] = 255
                    image[:, 0:border_size_x, 1] = 0
                    image[:, 0:border_size_x, 2] = 0
                    image[:, -border_size_x:, 0] = 255
                    image[:, -border_size_x:, 1] = 0
                    image[:, -border_size_x:, 2] = 0

                image_path = os.path.join(episode_path, f"frame_{i:04d}.jpg")
                with open(image_path, "wb") as f:
                    img = Image.fromarray(image)
                    img.save(f)

            # In addition, create a single mp4 video for the episode using ffmpeg. Save in .../rollouts/
            video_path = os.path.join(f"{base_data_path}/results/videos_with_warnings/{task}/rollouts/", f"{episode_name}.mp4")
            os.system(f"ffmpeg -y -framerate 10 -i {episode_path}/frame_%04d.jpg -c:v libx264 -pix_fmt yuv420p {video_path}")

            k += 1
