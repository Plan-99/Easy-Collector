import sys
import os
import pathlib
import hydra
from omegaconf import DictConfig, OmegaConf
import torch
from collections import deque
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

from ...utils.image_parser import fetch_image_with_config

# Define paths relative to this file's location to ensure robustness.
FIPER_ROOT = "/root/backend/fiper"
BASE_CONFIG_PATH = FIPER_ROOT + "/configs"
BASE_DATA_PATH = FIPER_ROOT + "/data"
from ...configs.global_configs import DATASET_DIR as ORI_DATA_ROOT

from ...fiper.rnd import RNDTrainer
from ...fiper.evaluation import EvaluationManager, ResultsManager
from ...fiper.tasks import TaskManager

from ...fiper.shared_utils.hydra_utils import load_config
from ...fiper.shared_utils.utility_functions import get_required_tensors, set_seed
from ...fiper.shared_utils.embedding_helper import EmbeddingHelper


from ...env.env import Env
from ...env.agent import Agent

import numpy as np


def failure_detection(node, checkpoint, robots, sensors, task_obj, task_control, socketio_instance):
    if not rclpy.ok():
        rclpy.init(args=None)
    ros_node = rclpy.create_node(f'failure_detector_{checkpoint.id}')
    publisher = ros_node.create_publisher(Float64, '/failure_detection/uncertainty_score', 10)

    try:
        with hydra.initialize(config_path="../../fiper/configs", version_base="1.1", job_name="train_fiper"):
            cfg = hydra.compose(config_name=str(task_obj['id']))

            embedding_helper = EmbeddingHelper(checkpoint.id)

            tasks = cfg.get("tasks", [])
            # Methods to be evaluated
            rnd_models = cfg.get("rnd_models", [])
            methods = cfg.get("methods", [])
            methods.extend(rnd_models)

            window_size = 7
            action_embedding_window = deque(maxlen=window_size)
            obs_embedding_window = deque(maxlen=window_size)


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
            seed_list = cfg.eval.get("random_seeds", [0])  #
            all_seed_results = []
            seed = seed_list[0]
            set_seed(seed)
            task = tasks[0]
            # Load the complete config with only the task overridden
            task_cfg = load_config("task", task, return_only_subdict=False)

            task_data_path = os.path.join(BASE_DATA_PATH, task)

            dataset = None
            is_inference = True

            evaluationmanager = EvaluationManager(str(BASE_CONFIG_PATH), task_data_path, dataset, is_inference=is_inference, device=device, seed=seed)

            print(f"Starting failure detection for checkpoint {checkpoint.id}")
            
            evaluationmanager.prepare_for_inference(methods)
            socketio_instance.emit('failure_detection_progress', {'status': 'prepared_for_inference'}, room=node)


            #----------------------------------------------------------------------------------------------


            agents = []
            for robot in robots:
                agents.append(Agent(node, robot))
            env = Env(node, agents=agents, sensors=sensors)


            while not task_control['stop']:
                rollout_list = []
                # Here you would gather new rollout data for evaluation
                # For demonstration, we will assume rollout_dict is obtained

                rollout_dict = {
                    'observations': {
                        'qpos': {},
                        'images': {}
                    },
                    'qaction': {}
                }

                ts = env.record_step()
                for agent in env.agents:
                    rollout_dict['observations']['qpos'][f"robot_{agent.id}"] = np.array([ts.observation['robot_states'][agent.id]['qpos']])
                    rollout_dict['qaction'][f"robot_{agent.id}"] = None

                for sensor in env.sensors:
                    image = np.array(ts.observation['images'][f"sensor_{sensor['id']}"])


                    if image is not None:

                        image = fetch_image_with_config(image, {
                            'resize': task_obj['sensor_img_size'][str(sensor['id'])],
                            'cropped_area': task_obj['sensor_cropped_area'][str(sensor['id'])],
                            'rotate': task_obj['sensor_rotate'][str(sensor['id'])]
                        })

                        rollout_dict['observations']['images'][f"sensor_{sensor['id']}"] = np.array([image])
                    else:
                        print("error")
                # Perform inference

                rollout_list.append(rollout_dict)
                all_embeddings_batch, all_embedding_std = embedding_helper.get_embedding_batch(rollout_list)

                action_embedding_window.append(all_embeddings_batch[0][0])
                obs_embedding_window.append(all_embeddings_batch[0][0][0].flatten())

                if len(action_embedding_window) == window_size:
                    # The window is full, we can perform inference.
                    stacked_action_per_step = [np.stack(t, axis=0) for t in action_embedding_window]
                    action_preds = np.stack(stacked_action_per_step, axis=0)

                    stacked_obs_per_step = [np.stack(t, axis=0) for t in obs_embedding_window]
                    obs_embeddings = np.stack(stacked_obs_per_step, axis=0)

                    infer_input = {
                        'action_preds': action_preds,
                        'obs_embeddings': obs_embeddings,
                        'successful': True,
                    }

                    uncertainty_score_results = evaluationmanager.infer(infer_input)
                    # for i, uncertainty_scores in enumerate(uncertainty_score_results):
                        # print(f"Uncertainty Scores {i}: {uncertainty_scores}")

                    uncertainty_score = 1.5 * uncertainty_score_results[0][window_size-1] + 0.8 * uncertainty_score_results[1][window_size-1]

                    # print(f"Combined Uncertainty Score: {uncertainty_score}")
                    
                    msg = Float64()
                    msg.data = float(uncertainty_score)
                    if rclpy.ok():
                        publisher.publish(msg)

                    socketio_instance.emit('failure_detection_result', {
                        'uncertainty_score': float(uncertainty_score),
                        'failure_detected': bool(uncertainty_score > 1.2)
                    })
    finally:
        ros_node.destroy_node()
        rclpy.shutdown()
    return
    



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