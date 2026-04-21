from .base_eval_class import BaseEvalClass
import torch
import torch.nn as nn
from tqdm import tqdm
from shared_utils.nn_base.small_submodules import SinusoidalPosEmb
from typing import Union

class LOGPZOEval(BaseEvalClass):
    def __init__(self, cfg, method_name, device, task_data_path, dataset, **kwargs):
        super().__init__(cfg, method_name, device, task_data_path, dataset, **kwargs)

        self.device = "cuda" if torch.cuda.is_available() else "cpu"

        self.embedding_dim = self.dataset.get_tensor_shape("obs_embeddings")[-1]

        # Get the observation embeddings from the dataset
        obs_embeddings = self.dataset.get_subset(subset="calibration", required_tensors="obs_embeddings")[
            "obs_embeddings"
        ]
        self.embeddings = obs_embeddings.to(self.device)

        # Create model
        self.model = LogpZOModel(input_dim=self.embedding_dim)
        self.model.to(self.device)
        self.trainer = LogpZOTrainer(
            model=self.model,
            num_epochs=self.cfg.num_epochs,
            batch_size=self.cfg.batch_size,
            learning_rate=self.cfg.learning_rate,
        )



    def _execute_preprocessing(self):
        # Train a flow matching model to learn the distribution of the embeddings: Do it
        self.trainer.train(embeddings=self.embeddings)
        self.model.load_state_dict(self.trainer.best_model)
        self.model.eval()

    def load_model(self):
        # The embedding similarity method does not require a model
        pass

    def calculate_uncertainty_score(self, rollout_tensor_dict, **kwargs):
        obs_embeddings = rollout_tensor_dict["obs_embeddings"]
        if len(obs_embeddings.shape) > 1:
            obs_embeddings = obs_embeddings[0]
        # obs_embedding = torch.tensor(obs_embeddings[-self.embedding_dim :]).to(self.device)  # Shape (embedding_dim,)
        obs_embeddings = obs_embeddings.unsqueeze(0).to(self.device)  # Shape (1, embedding_dim)

        # Starting from embedding, integrate ODE backwards to get the noise
        timesteps = torch.tensor([0], dtype=torch.long, device=self.device).unsqueeze(1)
        noise = obs_embeddings + self.model(obs_embeddings, timesteps)  # Shape (1, embedding_dim)

        # Calculate score as ||noise||_2^2
        uncertainty_score = torch.norm(noise, p=2).item()**2  # Shape (1,)

        return uncertainty_score


class LogpZOTrainer:
    def __init__(self, model, num_epochs, batch_size, learning_rate):
        self.model = model
        self.best_model = None
        self.num_epochs = num_epochs
        self.batch_size = batch_size
        self.learning_rate = learning_rate
        self.optimizer = torch.optim.Adam(self.model.parameters(), lr=self.learning_rate)

    def train(self, embeddings):
        # Split dataset into training and validation sets
        train_size = int(0.9 * len(embeddings))
        val_size = len(embeddings) - train_size
        train_dataset, val_dataset = torch.utils.data.random_split(embeddings, [train_size, val_size])

        # Create a DataLoader for batching
        train_dataloader = torch.utils.data.DataLoader(train_dataset, batch_size=self.batch_size, shuffle=True)
        val_dataloader = torch.utils.data.DataLoader(val_dataset, batch_size=self.batch_size, shuffle=False)

        # Training loop
        best_val_loss = float("inf")
        progress_bar = tqdm(range(self.num_epochs), desc="Training Progress", leave=False)
        for epoch in progress_bar:
            # Training
            epoch_loss = 0.0
            self.model.train()
            for batch in train_dataloader:
                self.optimizer.zero_grad()

                # Construct training target for flow matching. 
                o_0 = torch.randn_like(batch)
                target = o_0 - batch

                timesteps = torch.rand((batch.shape[0], 1), device=batch.device)
                o_t = (1 - timesteps) * batch + timesteps * o_0
                output = self.model(o_t, timesteps)

                # Calculate loss
                loss = nn.MSELoss()(output, target)
                epoch_loss += loss.item()
                
                # Backpropagation
                loss.backward()
                self.optimizer.step()
            
            epoch_loss /= len(train_dataloader)

            # Validation
            val_loss = 0.0
            self.model.eval()
            with torch.no_grad():
                for batch in val_dataloader:
                    # Construct training target for flow matching. 
                    o_0 = torch.randn_like(batch)
                    target = o_0 - batch

                    timesteps = torch.rand((batch.shape[0], 1), device=batch.device)
                    o_t = (1 - timesteps) * batch + timesteps * o_0
                    output = self.model(o_t, timesteps)

                    # Calculate loss
                    loss = nn.MSELoss()(output, target)
                    val_loss += loss.item()
            val_loss /= len(val_dataloader)
            
            progress_bar.set_description(
                f"Epoch {epoch + 1}: Train Loss: {epoch_loss:.4f}, Val Loss: {val_loss:.4f}"
            )

            # Save the best model
            if val_loss < best_val_loss:
                best_val_loss = val_loss
                # Store current model as self.best_model instead of saving to disk
                self.best_model = self.model.state_dict()


# Implement backbone for flow matching model with sinusoidal positional encoding
class LogpZOModel(nn.Module):
    def __init__(self, 
        input_dim,
        diffusion_step_embed_dim=256,
        down_dims=[256, 512, 1024],
        cond_predict_scale=False
    ):
        super().__init__()
        all_dims = [input_dim] + list(down_dims)

        dsed = diffusion_step_embed_dim
        self.diffusion_step_encoder = nn.Sequential(
            SinusoidalPosEmb(dsed),
            nn.Linear(dsed, dsed * 4),
            nn.Mish(),
            nn.Linear(dsed * 4, dsed),
        )

        in_out = list(zip(all_dims[:-1], all_dims[1:]))

        mid_dim = all_dims[-1]
        self.mid_modules = nn.ModuleList([
            ConditionalResidualBlock(
                mid_dim, mid_dim, cond_dim=dsed,
                cond_predict_scale=cond_predict_scale
            ),
            ConditionalResidualBlock(
                mid_dim, mid_dim, cond_dim=dsed,
                cond_predict_scale=cond_predict_scale
            ),
        ])

        self.down_modules = nn.ModuleList([])
        for ind, (dim_in, dim_out) in enumerate(in_out):
            is_last = ind >= (len(in_out) - 1)
            self.down_modules.append(nn.ModuleList([
                ConditionalResidualBlock(
                    dim_in, dim_out, cond_dim=dsed,
                    cond_predict_scale=cond_predict_scale
                ),
                ConditionalResidualBlock(
                    dim_out, dim_out, cond_dim=dsed,
                    cond_predict_scale=cond_predict_scale
                ),
                nn.Identity() if is_last else nn.Linear(dim_out, dim_out)
            ]))

        self.up_modules = nn.ModuleList([])
        for ind, (dim_in, dim_out) in enumerate(reversed(in_out[1:])):
            is_last = ind >= (len(in_out) - 1)
            self.up_modules.append(nn.ModuleList([
                ConditionalResidualBlock(
                    dim_out * 2, dim_in, cond_dim=dsed,
                    cond_predict_scale=cond_predict_scale
                ),
                ConditionalResidualBlock(
                    dim_in, dim_in, cond_dim=dsed,
                    cond_predict_scale=cond_predict_scale
                ),
                nn.Identity() if is_last else nn.Linear(dim_in, dim_in)
            ]))

        self.final_layer = nn.Sequential(
            nn.Linear(down_dims[0], input_dim),
        )

    def forward(self, sample: torch.Tensor, timestep: Union[torch.Tensor, float, int], **kwargs):
        """
        sample: (B, input_dim)
        timestep: (B, 1)
        output: (B, input_dim)
        """
        # Encode timestep
        # if not torch.is_tensor(timestep):
        #     timestep = torch.tensor([timestep], dtype=torch.long, device=sample.device)
        if torch.is_tensor(timestep) and len(timestep.shape) == 0:
            timestep = timestep[None].to(sample.device)
        global_feature = self.diffusion_step_encoder(timestep)[:,0]

        # Downsampling path
        x = sample
        h = []
        for resnet, resnet2, downsample in self.down_modules:
            x = resnet(x, global_feature)
            x = resnet2(x, global_feature)
            h.append(x)
            x = downsample(x)

        # Middle layers
        for mid_module in self.mid_modules:
            x = mid_module(x, global_feature)

        # Upsampling path
        for resnet, resnet2, upsample in self.up_modules:
            x = torch.cat((x, h.pop()), dim=1)
            x = resnet(x, global_feature)
            x = resnet2(x, global_feature)
            x = upsample(x)

        # Final layer
        x = self.final_layer(x)
        return x


class ConditionalResidualBlock(nn.Module):
    def __init__(self, in_channels, out_channels, cond_dim, cond_predict_scale=False):
        super().__init__()
        self.blocks = nn.ModuleList([
            nn.Linear(in_channels, out_channels),
            nn.Linear(out_channels, out_channels),
        ])

        # FiLM modulation
        cond_channels = out_channels * 2 if cond_predict_scale else out_channels
        self.cond_predict_scale = cond_predict_scale
        self.out_channels = out_channels
        self.cond_encoder = nn.Sequential(
            nn.Mish(),
            nn.Linear(cond_dim, cond_channels),
        )

        # Residual connection
        self.residual_layer = nn.Linear(in_channels, out_channels) if in_channels != out_channels else nn.Identity()

    def forward(self, x, cond):
        """
        x: (B, in_channels)
        cond: (B, cond_dim)
        returns: (B, out_channels)
        """
        out = self.blocks[0](x)
        embed = self.cond_encoder(cond)
        if self.cond_predict_scale:
            scale, bias = embed.chunk(2, dim=-1)
            out = scale * out + bias
        else:
            out = out + embed
        out = self.blocks[1](out)
        out = out + self.residual_layer(x)
        return out