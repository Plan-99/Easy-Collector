import logging

import einops
import flax.nnx as nnx
import flax.nnx.bridge as nnx_bridge
import jax
import jax.numpy as jnp
from typing_extensions import override

from src.backend.lerobot.src.lerobot.policies.openpi_train.src.openpi.models import model as _model
from src.backend.lerobot.src.lerobot.policies.openpi_train.src.openpi.models import pi0_config
import src.backend.lerobot.src.lerobot.policies.openpi_train.src.openpi.models.gemma as _gemma
import src.backend.lerobot.src.lerobot.policies.openpi_train.src.openpi.models.siglip as _siglip
from src.backend.lerobot.src.lerobot.policies.openpi_train.src.openpi.shared import array_typing as at

logger = logging.getLogger("openpi")


def make_attn_mask(input_mask, mask_ar):
    """Adapted from big_vision.

    Tokens can attend to valid inputs tokens which have a cumulative mask_ar
    smaller or equal to theirs. This way `mask_ar` bool[?B, N] can be used to
    setup several types of attention, for example:

      [[1 1 1 1 1 1]]: pure causal attention.

      [[0 0 0 1 1 1]]: prefix-lm attention. The first 3 tokens can attend between
          themselves and the last 3 tokens have a causal attention. The first
          entry could also be a 1 without changing behaviour.

      [[1 0 1 0 1 0 0 1 0 0]]: causal attention between 4 blocks. Tokens of a
          block can attend all previous blocks and all tokens on the same block.

    Args:
      input_mask: bool[B, N] true if its part of the input, false if padding.
      mask_ar: bool[?B, N] mask that's true where previous tokens cannot depend on
        it and false where it shares the same attention mask as the previous token.
    """
    mask_ar = jnp.broadcast_to(mask_ar, input_mask.shape)
    cumsum = jnp.cumsum(mask_ar, axis=1)
    attn_mask = cumsum[:, None, :] <= cumsum[:, :, None]
    valid_mask = input_mask[:, None, :] * input_mask[:, :, None]
    return jnp.logical_and(attn_mask, valid_mask)


@at.typecheck
def posemb_sincos(
    pos: at.Real[at.Array, " b"], embedding_dim: int, min_period: float, max_period: float
) -> at.Float[at.Array, "b {embedding_dim}"]:
    """Computes sine-cosine positional embedding vectors for scalar positions."""
    if embedding_dim % 2 != 0:
        raise ValueError(f"embedding_dim ({embedding_dim}) must be divisible by 2")

    fraction = jnp.linspace(0.0, 1.0, embedding_dim // 2)
    period = min_period * (max_period / min_period) ** fraction
    sinusoid_input = jnp.einsum(
        "i,j->ij",
        pos,
        1.0 / period * 2 * jnp.pi,
        precision=jax.lax.Precision.HIGHEST,
    )
    return jnp.concatenate([jnp.sin(sinusoid_input), jnp.cos(sinusoid_input)], axis=-1)


class Pi0(_model.BaseModel):
    def __init__(self, config: pi0_config.Pi0Config, rngs: nnx.Rngs):
        super().__init__(config.action_dim, config.action_horizon, config.max_token_len)
        self.pi05 = config.pi05
        paligemma_config = _gemma.get_config(config.paligemma_variant)
        action_expert_config = _gemma.get_config(config.action_expert_variant)
        # TODO: rewrite gemma in NNX. For now, use bridge.
        llm = nnx_bridge.ToNNX(
            _gemma.Module(
                configs=[paligemma_config, action_expert_config],
                embed_dtype=config.dtype,
                adarms=config.pi05,
            )
        )
        llm.lazy_init(rngs=rngs, method="init", use_adarms=[False, True] if config.pi05 else [False, False])
        img = nnx_bridge.ToNNX(
            _siglip.Module(
                num_classes=paligemma_config.width,
                variant="So400m/14",
                pool_type="none",
                scan=True,
                dtype_mm=config.dtype,
            )
        )
        img.lazy_init(next(iter(config.fake_obs().images.values())), train=False, rngs=rngs)
        self.PaliGemma = nnx.Dict(llm=llm, img=img)
        self.action_in_proj = nnx.Linear(config.action_dim, action_expert_config.width, rngs=rngs)
        if config.pi05:
            self.time_mlp_in = nnx.Linear(action_expert_config.width, action_expert_config.width, rngs=rngs)
            self.time_mlp_out = nnx.Linear(action_expert_config.width, action_expert_config.width, rngs=rngs)
        else:
            self.state_proj = nnx.Linear(config.action_dim, action_expert_config.width, rngs=rngs)
            self.action_time_mlp_in = nnx.Linear(2 * action_expert_config.width, action_expert_config.width, rngs=rngs)
            self.action_time_mlp_out = nnx.Linear(action_expert_config.width, action_expert_config.width, rngs=rngs)
        self.action_out_proj = nnx.Linear(action_expert_config.width, config.action_dim, rngs=rngs)

        # This attribute gets automatically set by model.train() and model.eval().
        self.deterministic = True

    @at.typecheck
    def embed_prefix(
        self, obs: _model.Observation
    ) -> tuple[at.Float[at.Array, "b s emb"], at.Bool[at.Array, "b s"], at.Bool[at.Array, " s"]]:
        input_mask = []
        ar_mask = []
        tokens = []
        # embed images
        for name in obs.images:
            image_tokens, _ = self.PaliGemma.img(obs.images[name], train=False)
            
            tokens.append(image_tokens)
            input_mask.append(
                einops.repeat(
                    obs.image_masks[name],
                    "b -> b s",
                    s=image_tokens.shape[1],
                )
            )
            # image tokens attend to each other
            ar_mask += [False] * image_tokens.shape[1]

        # add language (aka tokenized inputs)
        if obs.tokenized_prompt is not None:
            tokenized_inputs = self.PaliGemma.llm(obs.tokenized_prompt, method="embed")
            tokens.append(tokenized_inputs)
            input_mask.append(obs.tokenized_prompt_mask)
            # full attention between image and language inputs
            ar_mask += [False] * tokenized_inputs.shape[1]
        tokens = jnp.concatenate(tokens, axis=1)
        input_mask = jnp.concatenate(input_mask, axis=1)
        ar_mask = jnp.array(ar_mask)
        return tokens, input_mask, ar_mask

    @at.typecheck
    def embed_suffix(
        self, obs: _model.Observation, noisy_actions: _model.Actions, timestep: at.Float[at.Array, " b"]
    ) -> tuple[
        at.Float[at.Array, "b s emb"],
        at.Bool[at.Array, "b s"],
        at.Bool[at.Array, " s"],
        at.Float[at.Array, "b emb"] | None,
    ]:
        input_mask = []
        ar_mask = []
        tokens = []
                
        if not self.pi05:
            # add a single state token
            state_token = self.state_proj(obs.state)[:, None, :]
            tokens.append(state_token)
            input_mask.append(jnp.ones((obs.state.shape[0], 1), dtype=jnp.bool_))
            # image/language inputs do not attend to state or actions
            ar_mask += [True]

        action_tokens = self.action_in_proj(noisy_actions)
        # embed timestep using sine-cosine positional encoding with sensitivity in the range [0, 1]
        time_emb = posemb_sincos(timestep, self.action_in_proj.out_features, min_period=4e-3, max_period=4.0)
        if self.pi05:
            # time MLP (for adaRMS)
            time_emb = self.time_mlp_in(time_emb)
            time_emb = nnx.swish(time_emb)
            time_emb = self.time_mlp_out(time_emb)
            time_emb = nnx.swish(time_emb)
            action_expert_tokens = action_tokens
            adarms_cond = time_emb
        else:
            # mix timestep + action information using an MLP (no adaRMS)
            time_tokens = einops.repeat(time_emb, "b emb -> b s emb", s=self.action_horizon)
            action_time_tokens = jnp.concatenate([action_tokens, time_tokens], axis=-1)
            action_time_tokens = self.action_time_mlp_in(action_time_tokens)
            action_time_tokens = nnx.swish(action_time_tokens)
            action_time_tokens = self.action_time_mlp_out(action_time_tokens)
            action_expert_tokens = action_time_tokens
            adarms_cond = None
        tokens.append(action_expert_tokens)
        input_mask.append(jnp.ones(action_expert_tokens.shape[:2], dtype=jnp.bool_))
        # image/language/state inputs do not attend to action tokens
        ar_mask += [True] + ([False] * (self.action_horizon - 1))
        tokens = jnp.concatenate(tokens, axis=1)
        input_mask = jnp.concatenate(input_mask, axis=1)
        ar_mask = jnp.array(ar_mask)
        return tokens, input_mask, ar_mask, adarms_cond

    @override
    def compute_loss(
        self, rng: at.KeyArrayLike, observation: _model.Observation, actions: _model.Actions, *, train: bool = False
    ) -> at.Float[at.Array, "*b ah"]:
        preprocess_rng, noise_rng, time_rng = jax.random.split(rng, 3)
        observation = _model.preprocess_observation(preprocess_rng, observation, train=train)

        batch_shape = actions.shape[:-2]
        noise = jax.random.normal(noise_rng, actions.shape)
        time = jax.random.beta(time_rng, 1.5, 1, batch_shape) * 0.999 + 0.001
        time_expanded = time[..., None, None]
        x_t = time_expanded * noise + (1 - time_expanded) * actions
        u_t = noise - actions

        # one big forward pass of prefix + suffix at once
        prefix_tokens, prefix_mask, prefix_ar_mask = self.embed_prefix(observation)
        suffix_tokens, suffix_mask, suffix_ar_mask, adarms_cond = self.embed_suffix(observation, x_t, time)
        input_mask = jnp.concatenate([prefix_mask, suffix_mask], axis=1)
        ar_mask = jnp.concatenate([prefix_ar_mask, suffix_ar_mask], axis=0)
        attn_mask = make_attn_mask(input_mask, ar_mask)
        positions = jnp.cumsum(input_mask, axis=1) - 1
        (prefix_out, suffix_out), _ = self.PaliGemma.llm(
            [prefix_tokens, suffix_tokens], mask=attn_mask, positions=positions, adarms_cond=[None, adarms_cond]
        )
        v_t = self.action_out_proj(suffix_out[:, -self.action_horizon :])

        return jnp.mean(jnp.square(v_t - u_t), axis=-1)

    @override
    def sample_actions(
        self,
        rng: at.KeyArrayLike,
        observation: _model.Observation,
        *,
        num_steps: int | at.Int[at.Array, ""] = 10,
        noise: at.Float[at.Array, "b ah ad"] | None = None,
    ) -> _model.Actions:
        observation = _model.preprocess_observation(None, observation, train=False)
        # note that we use the convention more common in diffusion literature, where t=1 is noise and t=0 is the target
        # distribution. yes, this is the opposite of the pi0 paper, and I'm sorry.
        dt = -1.0 / num_steps
        batch_size = observation.state.shape[0]
        if noise is None:
            noise = jax.random.normal(rng, (batch_size, self.action_horizon, self.action_dim))

        # first fill KV cache with a forward pass of the prefix
        prefix_tokens, prefix_mask, prefix_ar_mask = self.embed_prefix(observation)
        prefix_attn_mask = make_attn_mask(prefix_mask, prefix_ar_mask)
        positions = jnp.cumsum(prefix_mask, axis=1) - 1
        _, kv_cache = self.PaliGemma.llm([prefix_tokens, None], mask=prefix_attn_mask, positions=positions)

        def step(carry):
            x_t, time = carry
            suffix_tokens, suffix_mask, suffix_ar_mask, adarms_cond = self.embed_suffix(
                observation, x_t, jnp.broadcast_to(time, batch_size)
            )
            # `suffix_attn_mask` is shape (b, suffix_len, suffix_len) indicating how the suffix tokens can attend to each
            # other
            suffix_attn_mask = make_attn_mask(suffix_mask, suffix_ar_mask)
            # `prefix_attn_mask` is shape (b, suffix_len, prefix_len) indicating how the suffix tokens can attend to the
            # prefix tokens
            prefix_attn_mask = einops.repeat(prefix_mask, "b p -> b s p", s=suffix_tokens.shape[1])
            # `combined_mask` is shape (b, suffix_len, prefix_len + suffix_len) indicating how the suffix tokens (which
            # generate the queries) can attend to the full prefix + suffix sequence (which generates the keys and values)
            full_attn_mask = jnp.concatenate([prefix_attn_mask, suffix_attn_mask], axis=-1)
            assert full_attn_mask.shape == (
                batch_size,
                suffix_tokens.shape[1],
                prefix_tokens.shape[1] + suffix_tokens.shape[1],
            )
            # `positions` is shape (b, suffix_len) indicating the positions of the suffix tokens
            positions = jnp.sum(prefix_mask, axis=-1)[:, None] + jnp.cumsum(suffix_mask, axis=-1) - 1

            # suffix_tokens + kv_cache ---(action expert)--> suffix_out
            (prefix_out, suffix_out), _ = self.PaliGemma.llm(
                [None, suffix_tokens],
                mask=full_attn_mask,
                positions=positions,
                kv_cache=kv_cache,
                adarms_cond=[None, adarms_cond],
            )
            assert prefix_out is None
            v_t = self.action_out_proj(suffix_out[:, -self.action_horizon :])
                        
            return x_t + dt * v_t, time + dt

        def cond(carry):
            x_t, time = carry
            # robust to floating-point error
            return time >= -dt / 2

        x_0, _ = jax.lax.while_loop(cond, step, (noise, 1.0)) # 시간 반복 (T/F), 업데이트
        return x_0

    # def get_attention_maps(
    #     self,
    #     observation: _model.Observation,
    #     *,
    #     timestep: float = 0.5,
    # ) -> dict:
    #     """Extract real cross-attention maps from the transformer.

    #     Uses the last layer's Q/K projection weights to compute actual attention
    #     probabilities, showing how action tokens attend to image tokens.

    #     Args:
    #         observation: Input observation with images and prompt.
    #         timestep: Diffusion timestep to use for attention extraction (default: 0.5).

    #     Returns:
    #         dict containing:
    #             - 'camera_attentions': {cam_name: [B, 16, 16]} action→image attention per camera
    #             - 'text_camera_attentions': {cam_name: [B, 16, 16]} text→image attention per camera
    #             - 'action_to_image_attn': [B, K, G, action_len, num_image_tokens] raw cross-attention
    #             - 'attn_probs': [B, K, G, T, S] full attention from last layer
    #             - 'num_image_tokens', 'num_cameras', 'image_names', etc.
    #     """
    #     # Use dynamic image keys from the observation itself
    #     observation = _model.preprocess_observation(
    #         None, observation, train=False, image_keys=list(observation.images.keys())
    #     )
    #     batch_size = observation.state.shape[0]

    #     # Create dummy noisy actions for forward pass
    #     dummy_actions = jnp.zeros((batch_size, self.action_horizon, self.action_dim))
    #     time = jnp.full((batch_size,), timestep)

    #     # Embed prefix (images + text)
    #     prefix_tokens, prefix_mask, prefix_ar_mask = self.embed_prefix(observation)

    #     # Count image tokens per camera
    #     image_names = list(observation.images.keys())
    #     num_cameras = len(image_names)
    #     tokens_per_camera = 256  # 16x16 patches from 224x224 image with patch_size=14
    #     num_image_tokens = num_cameras * tokens_per_camera

    #     # Embed suffix (state + actions)
    #     suffix_tokens, suffix_mask, suffix_ar_mask, adarms_cond = self.embed_suffix(observation, dummy_actions, time)

    #     prefix_len = prefix_tokens.shape[1]
    #     suffix_len = suffix_tokens.shape[1]

    #     # Combine tokens
    #     input_mask = jnp.concatenate([prefix_mask, suffix_mask], axis=1)
    #     ar_mask = jnp.concatenate([prefix_ar_mask, suffix_ar_mask], axis=0)
    #     attn_mask = make_attn_mask(input_mask, ar_mask)
    #     positions = jnp.cumsum(input_mask, axis=1) - 1

    #     # Forward pass: run first N-1 layers, then compute exact last-layer attention
    #     _, _, attn_probs, q_full, k_full = self.PaliGemma.llm(
    #         [prefix_tokens, suffix_tokens],
    #         mask=attn_mask,
    #         positions=positions,
    #         adarms_cond=[None, adarms_cond],
    #         method="forward_with_exact_attention",
    #     )
    #     # attn_probs: [B, K, G, T, S] exact attention from last layer's Q/K on correct input
    #     # q_full: [B, T, K, G, H] - Q vectors from last layer (correct input)
    #     # k_full: [B, S, K, H] - K vectors from last layer (correct input)

    #     # Token layout in the full sequence:
    #     #   prefix: [img_cam1(256) | img_cam2(256) | ... | text(prompt_len)]
    #     #   suffix: [state(1) | actions(action_horizon)]  (Pi0)
    #     #       or: [actions(action_horizon)]              (Pi0.5)
    #     if self.pi05:
    #         action_start = prefix_len  # no state token
    #     else:
    #         action_start = prefix_len + 1  # skip state token
    #     action_end = prefix_len + suffix_len
    #     num_text_tokens = prefix_len - num_image_tokens

    #     # === 1) Action→Image: from exact self-attention (sliced) ===
    #     action_to_image_attn = attn_probs[:, :, :, action_start:action_end, :num_image_tokens]
    #     action_to_image_avg = action_to_image_attn.mean(axis=(1, 2, 3))

    #     # === 2) Text→Image: DEDICATED cross-attention ===
    #     # Use exact Q (text) and K (image) from the last layer's real computation
    #     # softmax over image keys only → not diluted by other tokens
    #     q_text = q_full[:, num_image_tokens:prefix_len, :, :, :]  # [B, N_text, K, G, H]
    #     k_image = k_full[:, :num_image_tokens, :, :]              # [B, N_image, K, H]

    #     text_image_logits = jnp.einsum(
    #         "BTKGH,BSKH->BKGTS", q_text, k_image, preferred_element_type=jnp.float32
    #     )  # [B, K, G, N_text, N_image]

    #     # Softmax over image keys only
    #     text_image_attn = jax.nn.softmax(text_image_logits, axis=-1)
    #     text_image_avg = text_image_attn.mean(axis=(1, 2, 3))  # [B, N_image]

    #     # Split by camera and reshape to spatial grid
    #     camera_attentions = {}
    #     text_camera_attentions = {}
    #     for i, name in enumerate(image_names):
    #         start_idx = i * tokens_per_camera
    #         end_idx = (i + 1) * tokens_per_camera

    #         # Action→Image attention per camera
    #         cam_attn = action_to_image_avg[:, start_idx:end_idx]
    #         camera_attentions[name] = cam_attn.reshape(batch_size, 16, 16)

    #         # Text→Image dedicated cross-attention per camera
    #         text_cam_attn = text_image_avg[:, start_idx:end_idx]
    #         text_camera_attentions[name] = text_cam_attn.reshape(batch_size, 16, 16)

    #     return {
    #         'camera_attentions': camera_attentions,            # action→image (exact, sliced)
    #         'text_camera_attentions': text_camera_attentions,  # text→image (exact, dedicated)
    #         'action_to_image_attn': action_to_image_attn,
    #         'text_image_attn': text_image_attn,
    #         'attn_probs': attn_probs,
    #         'num_image_tokens': num_image_tokens,
    #         'num_text_tokens': num_text_tokens,
    #         'tokens_per_camera': tokens_per_camera,
    #         'num_cameras': num_cameras,
    #         'image_names': image_names,
    #     }
