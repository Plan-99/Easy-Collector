import os
import shutil
import torch
import numpy as np
from torch.utils.data import DataLoader, TensorDataset
import torch.optim as optim
from tqdm import tqdm
from rnd.rnd_models import RND_OE, RND_A, RND_AO, RNDBase
from datasets.rollout_datasets import ProcessedRolloutDataset
from omegaconf import DictConfig, OmegaConf
from shared_utils.hydra_utils import load_config
from shared_utils.utility_functions import set_seed
from typing import Union, Dict, Tuple
from datetime import datetime
import matplotlib.pyplot as plt
import hydra
import copy


class RNDTrainer:
    def __init__(
        self,
        base_config_path,
        task_data_path,
        dataset: ProcessedRolloutDataset,
        device=None,
        seed=None,
        **kwargs,
    ):
        """Initialize the RND trainer.
        Args:
            task (str): The task for which the RND models are trained.
            policy (DiffusionPolicy): The policy used to generate the rollouts.
            rnd_models (list): The list of RND models to train.
            config_path (str): The path to the configuration file.
            overwrite (bool): Whether to overwrite existing training data.
        Purpose:
            The RND trainer is used to train the RND models for a given task.
            It generates the training datasets from the raw datasets and trains the specified RND models.
            The training datasets are saved in the "/data_all/{task}/rnd_training_data/" directory.
            The trained models are saved in the "/data_all/{task}/{model}/" directory.
            The only function that should be called from outside is the train function.
        """
        self.base_config_path = base_config_path
        self.task_data_path = task_data_path
        self.training_data_dir = os.path.join(self.task_data_path, "rnd_training_data")
        if device is not None and device in ["cpu", "cuda:0"]:
            self.device = device if torch.cuda.is_available() else "cpu"
        else:
            self.device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")
        self.dataset: ProcessedRolloutDataset = dataset

        self.task_cfg = kwargs.get("task_cfg", None)
        self.seed = seed

    def train(self, rnd_models):
        """Train the specified RND models."""
        for model in rnd_models:
            print(f"Training {model}")
            self._train_model(model)

    def _train_model(self, model_name):
        """Train the specified RND model."""
        # Load config and set seed
        filename = model_name
        if model_name.startswith("nrnd"):
            filename.replace("nrnd", "rnd")
        cfg = load_config(module="eval", filename=filename, return_only_subdict=True)

        if self.seed is not None:
            cfg.hparams.model.seed = self.seed
        # Fix hparams to prevent input dict modifications in rnd class from changing them
        cfg.hparams = OmegaConf.to_container(cfg.hparams, resolve=True)

        # Define data path, load dataset, and save directory
        save_dir = os.path.join(self.task_data_path, "rnd_models", model_name)
        # If it exists, remove the directory
        if os.path.exists(save_dir) and cfg.rnd_train.get("overwrite", False):
            shutil.rmtree(save_dir)
        os.makedirs(save_dir, exist_ok=True)
        # Normaliztion variant use the same training function as the non-normalization variant
        self._training_loop(
            save_dir,
            cfg,
            model_name=filename,
            world_model_dir=os.path.join(self.task_data_path, "sys_id"),
        )

    def _get_datasets_for_model(self, model_name: str, cfg: DictConfig, **kwargs) -> Dict[str, torch.Tensor]:
        """
        Load and filter datasets based on the model's requirements.
        Args:
            model_name (str): The name of the model.
        Returns:
            dict: A dictionary of datasets.
        """
        required_datasets = cfg.required_tensors
        optional_datasets = cfg.optional_tensors

        # Returns all datasets requested but the optional ones if not available
        datasets = self.dataset.get_subset(
            subset="calibration",
            required_tensors=required_datasets,
            optional_tensors=optional_datasets,
            return_as_list=False,
            required_actions=cfg.required_actions,
            optional_actions=cfg.optional_actions,
            history=cfg.history_length,
            normalize_tensors=dict(cfg.normalize_tensors),
        )

        # Create a dictionary of datasets if as list
        if isinstance(datasets, list):
            dataset_dict = {
                name: dataset
                for name, dataset in zip(required_datasets + optional_datasets, datasets)
                if dataset is not None
            }
            return dataset_dict
        return datasets

    def _create_dataloader(self, dataset_dict: Dict[str, torch.Tensor], **kwargs) -> DataLoader:
        """
        Create a DataLoader from the dataset dictionary.
        Args:
            dataset_dict (dict): A dictionary of datasets.
            batch_size (int): The batch size.
        Returns:
            DataLoader: The DataLoader object.
        """
        # Combine datasets into a TensorDataset
        tensors = list(dataset_dict.values())
        dataset = TensorDataset(*tensors)
        dataloader = DataLoader(
            dataset,
            batch_size=kwargs.get("batch_size", 4),
            shuffle=kwargs.get("shuffle", True),
            num_workers=kwargs.get("num_workers", 4),
            pin_memory=kwargs.get("pin_memory", True),
        )
        return dataloader

    def _get_optimizers(self, model, cfg):
        if cfg.get("rnd_train", None) is not None:
            cfg = cfg.rnd_train
        if cfg.optimizer == "adamw":
            optimizer = optim.AdamW(model.parameters(), lr=cfg.lr, eps=cfg.eps, weight_decay=cfg.weight_decay)
        else:
            optimizer = optim.Adam(model.parameters(), lr=cfg.lr, eps=cfg.eps)

        # Add a cosine learning rate scheduler
        scheduler = optim.lr_scheduler.CosineAnnealingLR(optimizer, T_max=cfg.n_epochs, eta_min=cfg.get("lr_min", 1e-6))
        return optimizer, scheduler

    def _split_datasets(
        self, dataset_dict: Dict[str, torch.Tensor], train_ratio: float = 0.9
    ) -> Tuple[Dict[str, torch.Tensor], Dict[str, torch.Tensor]]:
        """
        Split the dataset dictionary into training and validation sets.

        Args:
            dataset_dict (dict): A dictionary of datasets where keys are dataset names and values are tensors.
            train_ratio (float): The ratio of the dataset to use for training. The rest is used for validation.

        Returns:
            Tuple[dict, dict]: Two dictionaries (train_dataset_dict, val_dataset_dict) containing the split datasets.
        """
        # Ensure the dataset dictionary is not empty
        if not dataset_dict:
            raise ValueError("The dataset dictionary is empty.")

        # Get the length of the datasets (assumes all datasets have the same length)
        dataset_length = dataset_dict[list(dataset_dict.keys())[0]].shape[0]

        # Generate train and validation indices
        val_indices = np.random.choice(dataset_length, int(dataset_length * (1 - train_ratio)), replace=False)
        train_indices = np.setdiff1d(np.arange(dataset_length), val_indices)

        # Split each dataset in the dictionary
        train_dataset_dict = {key: dataset[train_indices] for key, dataset in dataset_dict.items()}
        val_dataset_dict = {key: dataset[val_indices] for key, dataset in dataset_dict.items()}

        return train_dataset_dict, val_dataset_dict

    def _init_rnd_model(
        self,
        model_name: str,
        cfg: DictConfig,
        **kwargs,
    ) -> Tuple[Union[RND_OE, RND_A, RND_AO], DictConfig]:
        """Initialize the RND model based on the model name and configuration.

        Args:
            model_name (str): The name of the model to initialize.
            cfg (DictConfig): The configuration for the model.

        Returns:
            Tuple[Union[RND_OE, RND_A, RND_AO], DictConfig]: The initialized model and its configuration.
        """
        class_name = model_name.upper()
        module_name = "rnd." + class_name
        state_dim = self.dataset.get_tensor_shape("states")[-1]
        obs_embedding_dim = self.dataset.get_tensor_shape("obs_embeddings")[-1]
        action_pred_shape = self.dataset.get_tensor_shape(
            "action_preds",
            filter_actions=True,
            required_actions=cfg.required_actions,
            optional_actions=cfg.optional_actions,
        )[-3:]
        rgb_image_shape = self.dataset.get_tensor_shape("rgb_images")[1:]

        model_cfg = {
            "input_dict": {
                "input_size": None,
                "output_size": 512,
                "hyperparameters": cfg.model_hyperparameters,
                "use_states": state_dim > 0,
                # Used for calculating input sizes
                "state_dim": state_dim,
                "obs_embedding_dim": obs_embedding_dim,
                "action_pred_shape": action_pred_shape,
                "rgb_image_shape": rgb_image_shape,
                # Determines the action prediction handling
                "action_batch_handling": cfg.action_batch_handling,
                "action_execution_horizon": self.task_cfg.task.action_space.action_execution_horizon,
                "normalize_tensors": cfg.normalize_tensors,
            },
            # Used to identify the model
            "hparams": cfg.hparams.model,
            # Used to load the model
            "model_type": class_name,
            "_target_": module_name,
            "cfg": cfg,
            "kwargs": kwargs,
        }

        rnd_class = hydra.utils.get_class(module_name)

        model: RNDBase = rnd_class(input_dict=model_cfg["input_dict"], **kwargs).to(self.device)
        # Model calculates some parameters based on the input_dict
        model_cfg["input_dict"] = model.input_dict
        return model, model_cfg

    def _training_loop(
        self,
        save_dir: str,
        cfg: DictConfig,
        model_name: str,
        **kwargs,
    ):
        """Modularized function that:
            - Loads a model and model_configuration
            - Loads the model-specific datasets
            - Initializes the optimizer
            - Initializes the training parameters
            - Initializes the dataloaders (train and optimally validation)
            - Trains the model
            - Saves the model checkpoints
        Args:
            save_dir (str): The directory to save the model checkpoints.
            cfg (DictConfig): The configuration for the model.
        """
        # Load the model
        model, model_cfg = self._init_rnd_model(model_name, cfg, **kwargs)
        model_exists = model._check_for_existance(save_dir, model_cfg["hparams"])
        if model_exists:
            # If the model already exists and we are in deterministic mode, skip training
            print(f"RND Model {model_name} already exists. Skipping training.")
            return
        set_seed(cfg.hparams.model.seed)
        dataset_dict = self._get_datasets_for_model(model_cfg["model_type"], cfg, **kwargs)
        optimizer, scheduler = self._get_optimizers(model, cfg)

        # Initialize training parameters
        best_loss = float("inf")
        patience = cfg.rnd_train.get("patience", 5)  # Default patience for early stopping
        keep_checkpoints = cfg.rnd_train.get("keep_checkpoints", 2)
        checkpoint_counter = 0
        patience_counter = 0
        save_every_n_epochs = cfg.rnd_train.get("save_every_n_epochs", 0)  # 0 Means no saving
        early_stopping = cfg.rnd_train.get("early_stopping", True)
        use_validation = cfg.rnd_train.get("use_validation", False)

        if use_validation:
            train_set, val_set = self._split_datasets(dataset_dict, train_ratio=cfg.rnd_train.get("train_ratio", 0.9))
        else:
            train_set = dataset_dict

        # Create DataLoader
        print("Datasets used for training:")
        for key in dataset_dict.keys():
            print(f"{key} with shape {dataset_dict[key].shape}")

        train_dataloader = self._create_dataloader(
            train_set,
            batch_size=min(cfg.rnd_train.batch_size, train_set[list(dataset_dict.keys())[0]].shape[0]),
            shuffle=True,
        )

        if use_validation:
            val_dataloader = self._create_dataloader(
                val_set,
                batch_size=min(cfg.rnd_train.batch_size, val_set[list(dataset_dict.keys())[0]].shape[0]),
                shuffle=False,
            )

        if early_stopping:
            print(f"Early stopping with patience {patience} and saving every {save_every_n_epochs} epochs.")
        else:
            print(
                f"No early stopping. Saving every {save_every_n_epochs} epochs (Zero means never save intermediate checkpoints)."
            )

        # Training loop
        train_losses, val_losses = [], []
        progress_bar = tqdm(range(cfg.rnd_train.n_epochs), desc="Training Progress", leave=False)

        best_state_dict = None

        for epoch in progress_bar:
            model.train()
            epoch_loss = 0.0
            for batch in train_dataloader:
                batch_dict = {key: value.to(self.device) for key, value in zip(dataset_dict.keys(), batch)}
                loss = model(**model.datasets_to_model_inputs(batch_dict)).mean()
                optimizer.zero_grad()
                loss.backward()
                optimizer.step()
                epoch_loss += loss.item()

            scheduler.step()
            if torch.isnan(loss).any():
                print(f"NaN loss detected at epoch {epoch + 1}.")
                return

            # Compute average training loss
            epoch_loss /= len(train_dataloader)
            train_losses.append(epoch_loss)

            # Validation
            if use_validation:
                model.eval()
                with torch.no_grad():
                    val_loss = 0.0
                    for batch in val_dataloader:
                        batch_dict = {key: value.to(self.device) for key, value in zip(dataset_dict.keys(), batch)}
                        loss = model(**model.datasets_to_model_inputs(batch_dict)).mean()
                        val_loss += loss.item()
                    val_loss /= len(val_dataloader)
                    # val_loss = model(*val_set).mean().item()

                progress_bar.set_description(
                    f"Epoch {epoch + 1}: Train Loss: {epoch_loss:.4f}, Val Loss: {val_loss:.4f}"
                )
                # print(f"Epoch {epoch + 1}: Train Loss = {epoch_loss:.4f}, Val Loss = {val_loss:.4f}")
            else:
                val_loss = epoch_loss
                progress_bar.set_description(f"Epoch {epoch + 1}: Train Loss: {epoch_loss:.4f}")
                # print(f"Epoch {epoch + 1}: Train Loss = {epoch_loss:.4f}")
            val_losses.append(val_loss)

            # Early stopping
            if early_stopping:
                if cfg.rnd_train.get("stop_when_avg_improvement", 0) > 0 and epoch > 15:
                    improvement = val_losses[-patience] - val_losses[-1]
                    if improvement < cfg.rnd_train.stop_when_avg_improvement:
                        print(f"Early stopping triggered due to improvement over patience being {improvement}.")
                        break
                if val_loss < best_loss:
                    best_loss = val_loss
                    patience_counter = 0
                    best_state_dict = copy.deepcopy(model.state_dict())
                else:
                    patience_counter += 1
                    if patience_counter >= patience:
                        print(
                            f"Early stopping triggered due to validation loss not improving over the last {patience_counter} epochs."
                        )
                        break
                if cfg.rnd_train.get("stop_when_val_to_train_ratio", 0) > 0 and epoch > 20:
                    val_to_train_ratio = val_loss / epoch_loss
                    if val_to_train_ratio > cfg.rnd_train.stop_when_val_to_train_ratio:
                        print(
                            f"Early stopping triggered due to validation to training loss ratio being greater than {cfg.rnd_train.stop_when_val_to_train_ratio}."
                        )
                        break
            # Save model checkpoint
            if save_every_n_epochs > 0:
                if (epoch + 1) % save_every_n_epochs == 0:
                    model._save_checkpoint(
                        save_dir,
                        model_cfg=model_cfg,
                        checkpoint_name=f"ckpt_epoch_{epoch + 1}.ckpt",
                        state_dict=best_state_dict,
                        kwargs=kwargs,
                    )
                    checkpoint_counter += 1
                    if checkpoint_counter > keep_checkpoints:
                        print("Removing old checkpoints")
                        os.remove(
                            os.path.join(
                                save_dir, f"ckpt_epoch_{epoch + 1 - keep_checkpoints * save_every_n_epochs}.ckpt"
                            )
                        )
                        checkpoint_counter -= 1
        # Debug print for rnd_awm model
        if hasattr(model, "last_error_influences"):
            print(
                "last_error_influences:",
                model.last_error_influences,
                "desired_influences:",
                model.target_error_influences,
                "last_norm_factors:",
                model.last_norm_factors,
            )

        if not early_stopping:
            print("Training finished without early stopping. Saving the final model.")
            best_state_dict = copy.deepcopy(model.state_dict())
        # Save the best model
        filename = f"{datetime.now().strftime('%Y-%m-%d_%H')}_epoch_{epoch + 1}_loss_{val_loss:.2g}_seed_{model_cfg['hparams']['seed']}.ckpt"

        model._save_checkpoint(
            save_dir,
            model_cfg=model_cfg,
            checkpoint_name=filename,
            state_dict=best_state_dict,
            kwargs=kwargs,
        )
        # # Plot training and validation loss
        self._plot_training_progress(train_losses, val_losses, save_dir)

    def _save_tensors(self, save_dir: str, tensor, filename: str, overwrite: bool = True):
        """Save the tensor to a file."""
        os.makedirs(save_dir, exist_ok=True)
        if not filename.endswith(".pt"):
            filename += ".pt"
        if os.path.exists(os.path.join(save_dir, filename)) and not overwrite:
            return

        with open(os.path.join(save_dir, filename), "wb") as f:
            torch.save(tensor, f)

    def _load_tensors(self, load_dir, filename):
        if not os.path.exists(load_dir):
            raise ValueError(f"Data directory {load_dir} not found")
        if not os.path.exists(os.path.join(load_dir, filename)):
            raise ValueError(f"File {filename} not found in {load_dir}. Files in directory: {os.listdir(load_dir)}")
        with open(os.path.join(load_dir, filename), "rb") as f:
            tensor = torch.load(f, weights_only=True)
        return tensor

    def _plot_training_progress(self, train_losses, val_losses, save_dir):
        """Plot training and validation loss."""

        plt.figure(figsize=(10, 6))
        plt.plot(train_losses, label="Training Loss")
        plt.plot(val_losses, label="Validation Loss")
        plt.xlabel("Epochs")
        plt.ylabel("Loss")
        plt.title("Training Progress")
        plt.legend()
        plt.grid()
        plt.savefig(os.path.join(save_dir, "training_progress.png"))
        plt.close()
