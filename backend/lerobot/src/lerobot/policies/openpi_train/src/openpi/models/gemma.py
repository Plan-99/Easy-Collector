# Copyright 2024 Big Vision Authors.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Gemma adaptation for Pi, taken from big_vision.

We follow this einsum axis naming convention:
  B: batch
  T: query length
  S: k/v length
  N: num query heads
  K: num k/v heads
  G: num query heads per k/v head
  H: head dim
  D: d_model ("features")
"""

from collections.abc import Sequence
import dataclasses
from typing import Literal, TypeAlias

import einops
import flax.linen as nn
import jax
import jax.numpy as jnp

import src.backend.lerobot.src.lerobot.policies.openpi_train.src.openpi.models.lora as lora
import src.backend.lerobot.src.lerobot.policies.openpi_train.src.openpi.shared.array_typing as at
import src.backend.lerobot.src.lerobot.policies.openpi_train.src.openpi.training.sharding as sharding

PALIGEMMA_VOCAB_SIZE = 257_152


@dataclasses.dataclass
class Config:
    width: int
    depth: int
    mlp_dim: int
    num_heads: int
    num_kv_heads: int
    head_dim: int
    lora_configs: dict[str, lora.LoRAConfig] = dataclasses.field(default_factory=dict)


Variant = Literal["dummy", "gemma_300m", "gemma_300m_lora", "gemma_2b", "gemma_2b_lora"]


def get_config(variant: Variant) -> Config:
    """Returns config for specified gemma variant."""
    if variant == "dummy":
        return Config(
            width=64,
            depth=4,
            mlp_dim=128,
            num_heads=8,
            num_kv_heads=1,
            head_dim=16,
        )
    if variant == "gemma_300m":
        # 311M params
        return Config(
            width=1024,
            depth=18,
            mlp_dim=4096,
            num_heads=8,
            num_kv_heads=1,
            head_dim=256,
        )
    if variant == "gemma_2b":
        return Config(
            width=2048,
            depth=18,
            mlp_dim=16_384,
            num_heads=8,
            num_kv_heads=1,
            head_dim=256,
        )
    if variant == "gemma_2b_lora":
        return Config(
            width=2048,
            depth=18,
            mlp_dim=16_384,
            num_heads=8,
            num_kv_heads=1,
            head_dim=256,
            lora_configs={"attn": lora.LoRAConfig(rank=16, alpha=16.0), "ffn": lora.LoRAConfig(rank=16, alpha=16.0)},
        )
    if variant == "gemma_300m_lora":
        # 311M params
        return Config(
            width=1024,
            depth=18,
            mlp_dim=4096,
            num_heads=8,
            num_kv_heads=1,
            head_dim=256,
            lora_configs={"attn": lora.LoRAConfig(rank=32, alpha=32.0), "ffn": lora.LoRAConfig(rank=32, alpha=32.0)},
        )
    raise ValueError(f"Unknown variant: {variant}")


@at.typecheck
class RMSNorm(nn.Module):
    @nn.compact
    def __call__(self, x, cond):
        dtype = x.dtype  # original dtype, could be half-precision
        var = jnp.mean(jnp.square(x.astype(jnp.float32)), axis=-1, keepdims=True)  # compute variance in float32
        normed_inputs = jnp.asarray(x * jnp.reciprocal(jnp.sqrt(var + 1e-06)))  # compute normalization in float32
        if cond is None:
            # regular RMSNorm
            scale = self.param("scale", nn.initializers.zeros_init(), (x.shape[-1]))
            normed_inputs = normed_inputs * (
                1 + scale
            )  # scale by learned parameter in float32 (matches Flax implementation)
            return normed_inputs.astype(dtype), None  # return in original dtype

        # adaptive RMSNorm
        modulation = nn.Dense(x.shape[-1] * 3, kernel_init=nn.initializers.zeros, dtype=dtype)(cond)
        scale, shift, gate = jnp.split(modulation[:, None, :], 3, axis=-1)
        normed_inputs = normed_inputs * (1 + scale) + shift  # scale and shift in float32
        return normed_inputs.astype(dtype), gate


@at.typecheck
class Embedder(nn.Module):
    """Embedder module."""

    vocab_size: int
    embed_dim: int

    def setup(self):
        self.input_embedding_table = self.param(
            "input_embedding",
            nn.initializers.normal(),
            (self.vocab_size, self.embed_dim),
        )

    def encode(self, x):
        x = self.input_embedding_table[(x,)]
        x *= jnp.sqrt(self.embed_dim).astype(x.dtype)
        return x

    def decode(self, x):
        return jnp.dot(x, self.input_embedding_table.T)


@at.typecheck
class Attention(nn.Module):
    """Attention module."""

    configs: Sequence[Config]

    @nn.compact
    def __call__(self, xs, positions, attn_mask, kv_cache, return_attention: bool = False):
        # all experts must share the same head dim, num heads, and num kv heads for self-attention to work
        assert all(config.head_dim == self.configs[0].head_dim for config in self.configs)
        assert all(config.num_heads == self.configs[0].num_heads for config in self.configs)
        assert all(config.num_kv_heads == self.configs[0].num_kv_heads for config in self.configs)

        dtype = next(x.dtype for x in xs if x is not None)  # original dtype, could be half-precision

        qkvs = []
        for i, (x, config) in enumerate(zip(xs, self.configs, strict=True)):
            if x is None:
                continue
            if config.num_kv_heads == config.num_heads:
                qkv_einsum = lora.Einsum(
                    shape=(3, config.num_heads, config.width, config.head_dim),
                    name=_name("qkv_einsum", i),
                    init_fn=nn.initializers.lecun_normal(in_axis=-2, out_axis=-1, batch_axis=(0, 1)),
                    lora_config=config.lora_configs.get("attn"),
                )
                qkvs.append(qkv_einsum("BSD,3KDH->3BSKH", x))
            else:
                q_einsum = lora.Einsum(
                    shape=(config.num_heads, config.width, config.head_dim),
                    name=_name("q_einsum", i),
                    init_fn=nn.initializers.lecun_normal(in_axis=-2, out_axis=-1, batch_axis=(0,)),
                    lora_config=config.lora_configs.get("attn"),
                )
                q = q_einsum("BTD,NDH->BTNH", x)
                kv_einsum = lora.Einsum(
                    shape=(2, config.num_kv_heads, config.width, config.head_dim),
                    name=_name("kv_einsum", i),
                    init_fn=nn.initializers.lecun_normal(in_axis=-2, out_axis=-1, batch_axis=(0, 1)),
                    lora_config=config.lora_configs.get("attn"),
                )
                k, v = kv_einsum("BSD,2KDH->2BSKH", x)
                qkvs.append((q, k, v))

        q, k, v = (jnp.concatenate(y, axis=1) for y in zip(*qkvs, strict=True))

        q = _apply_rope(q, positions=positions)
        q *= self.configs[0].head_dim ** -0.5

        k = _apply_rope(k, positions=positions)

        # should still be half-precision here (if input was half-precision)
        assert q.dtype == k.dtype == v.dtype == dtype

        if kv_cache is not None:
            cache_k, cache_v = kv_cache
            k = jnp.concatenate([cache_k, k], axis=1)
            v = jnp.concatenate([cache_v, v], axis=1)

        q = einops.rearrange(q, "B T (K G) H -> B T K G H", K=self.configs[0].num_kv_heads)
        logits = jnp.einsum("BTKGH,BSKH->BKGTS", q, k, preferred_element_type=jnp.float32)

        if attn_mask.shape != (q.shape[0], 1, q.shape[1], k.shape[1]):
            raise ValueError(
                f"Attention mask with shape {attn_mask.shape} but shapes for q and k are: {q.shape} and {k.shape}"
            )

        # big_neg = jnp.finfo(logits.dtype).min
        big_neg = -2.3819763e38  # See gemma/modules.py
        masked_logits = jnp.where(attn_mask[:, :, None, :, :], logits, big_neg)

        probs = jax.nn.softmax(masked_logits, axis=-1).astype(dtype)

        encoded = jnp.einsum("BKGTS,BSKH->BTKGH", probs, v)
        encoded = einops.rearrange(encoded, "B T K G H -> B T (K G) H")

        out = []
        start = 0
        for i, (x, config) in enumerate(zip(xs, self.configs, strict=True)):
            if x is not None:
                end = start + x.shape[1]
                out_einsum = lora.Einsum(
                    shape=(config.num_heads, config.head_dim, config.width),
                    name=_name("attn_vec_einsum", i),
                    init_fn=nn.initializers.lecun_normal(in_axis=(-3, -2), out_axis=-1),
                    lora_config=config.lora_configs.get("attn"),
                )
                out.append(out_einsum("BTNH,NHD->BTD", encoded[:, start:end]))
                start = end
            else:
                out.append(None)

        if return_attention:
            return out, (k, v), probs, q, k  # probs: [B,K,G,T,S], q: [B,T,K,G,H], k: [B,S,K,H]
        return out, (k, v)


@at.typecheck
class FeedForward(nn.Module):
    """Feed forward module."""

    features: int
    hidden_dim: int

    @nn.compact
    def __call__(self, x):
        dtype = x.dtype  # original dtype, could be half-precision
        w_gating = self.param(
            "gating_einsum",
            nn.initializers.lecun_normal(in_axis=-2, out_axis=-1, batch_axis=(0,)),
            (2, self.features, self.hidden_dim),
        ).astype(dtype)
        ff_gate = jnp.dot(x, w_gating[0])
        gate_value = nn.gelu(ff_gate)

        ff1 = jnp.dot(x, w_gating[1])
        activations = gate_value * ff1

        w_linear = self.param(
            "linear",
            nn.initializers.lecun_normal(in_axis=-2, out_axis=-1),
            (self.hidden_dim, self.features),
        ).astype(dtype)
        outputs = jnp.dot(activations, w_linear)
        assert outputs.dtype == dtype
        return outputs


@at.typecheck
class Block(nn.Module):
    """Transformer block."""

    configs: tuple[Config, ...]

    dropout: float = 0.0
    dropout_bdims: tuple[int, ...] = ()

    @nn.compact
    def __call__(self, xs, kv_cache, positions, attn_mask, adarms_cond, deterministic=True, return_attention=False):  # noqa: FBT002
        xs = sharding.activation_sharding_constraint(xs)
        drop = nn.Dropout(self.dropout, self.dropout_bdims) if self.dropout else lambda x, _: x

        attn = Attention(configs=self.configs, name="attn")

        pre_attn = []
        gates = []
        for i, x in enumerate(xs):
            if x is not None:
                x, gate = RMSNorm(name=_name("pre_attention_norm", i))(x, adarms_cond[i])  # noqa: PLW2901
            pre_attn.append(x)
            gates.append(gate if x is not None else None)

        pre_attn = sharding.activation_sharding_constraint(pre_attn)
        attn_result = attn(pre_attn, positions, attn_mask, kv_cache, return_attention=return_attention)
        if return_attention:
            post_attn, kv_cache, attn_probs, attn_q, attn_k = attn_result
        else:
            post_attn, kv_cache = attn_result
            attn_probs = None
        post_attn = jax.tree.map(lambda x: drop(x, deterministic), post_attn)
        post_attn = sharding.activation_sharding_constraint(post_attn)
        xs = [_gated_residual(x, y, gate) for x, y, gate in zip(xs, post_attn, gates, strict=True)]
        xs = sharding.activation_sharding_constraint(xs)

        out = []
        gates = []
        for i, (x, config) in enumerate(zip(xs, self.configs, strict=True)):
            if x is not None:
                x, gate = RMSNorm(name=_name("pre_ffw_norm", i))(x, adarms_cond[i])  # noqa: PLW2901
                x = lora.FeedForward(  # noqa: PLW2901
                    features=config.width,
                    hidden_dim=config.mlp_dim,
                    name=_name("mlp", i),
                    lora_config=config.lora_configs.get("ffn"),
                )(x)
            out.append(x)
            gates.append(gate if x is not None else None)

        out = sharding.activation_sharding_constraint(out)
        out = jax.tree.map(lambda x: drop(x, deterministic), out)
        xs = [_gated_residual(x, y, gate) for x, y, gate in zip(xs, out, gates, strict=True)]
        xs = sharding.activation_sharding_constraint(xs)

        if return_attention:
            return xs, kv_cache, attn_probs, attn_q, attn_k
        return xs, kv_cache


KVCache: TypeAlias = tuple[at.Float[at.Array, "l b _t _k _h"], at.Float[at.Array, "l b _t _v _h"]]


@at.typecheck
class Module(nn.Module):
    """Transformer model, supporting a mixture of different weights for different tokens."""

    configs: Sequence[Config]  # list of configs, one for each expert
    embed_dtype: str

    dropout: float = 0.0
    dropout_bdims: tuple[int, ...] = ()  # Every float is dropped independently.
    adarms: bool = False
        
    def setup(self):
        # all experts must have the same depth
        assert all(config.depth == self.configs[0].depth for config in self.configs)

        self.embedder = Embedder(
            vocab_size=PALIGEMMA_VOCAB_SIZE,
            embed_dim=self.configs[0].width,  # embedder for first expert only
            name="embedder",
        )
        block_cls = nn.remat(
            Block,
            prevent_cse=False,
            static_argnums=(5,),  # 0=self, 6=deterministic
            policy=jax.checkpoint_policies.nothing_saveable,
        )
        self.layers = nn.scan(
            block_cls,
            variable_axes={"params": 0},
            split_rngs={"params": True, "dropout": True},
            in_axes=(
                0,
                nn.broadcast,
                nn.broadcast,
                nn.broadcast,
                nn.broadcast,
            ),  # 0=kv_cache, 1=positions, 2=mask, 3=adarms_cond, 4=deterministic
            length=self.configs[0].depth,
        )(
            configs=self.configs,
            dropout=self.dropout,
            dropout_bdims=self.dropout_bdims,
        )
        self.final_norms = [RMSNorm(name=_name("final_norm", i)) for i in range(len(self.configs))]
        
    @at.typecheck
    def embed(self, tokens: at.Int[at.Array, "b t"]) -> at.Float[at.Array, "b t d"]:
        return self.embedder.encode(tokens).astype(self.embed_dtype)

    @at.typecheck
    def __call__(
        self,
        # list of token arrays, one for each expert, or None if that expert should not be run
        embedded: Sequence[at.Float[at.Array, "b _t _d"] | None],
        positions: at.Int[at.Array, "b t"],
        mask: at.Bool[at.Array, "b t s"],
        adarms_cond: Sequence[at.Float[at.Array, "b _d"] | None] | None = None,
        *,
        kv_cache: KVCache | None = None,
        deterministic: bool = True,
    ) -> tuple[Sequence[at.Float[at.Array, "b _t _d"] | None], KVCache]:
        embedded = jax.tree.map(lambda e: e.astype(self.embed_dtype), embedded)
        mask = jnp.asarray(mask)[:, None, :, :]
        if adarms_cond is None:
            adarms_cond = [None] * len(self.configs)

        embedded, kv_cache = self.layers(embedded, kv_cache, positions, mask, adarms_cond, deterministic)

        assert all(e.dtype == jnp.dtype(self.embed_dtype) for e in embedded if e is not None)

        return [
            f(e, a)[0] if e is not None else e for f, e, a in zip(self.final_norms, embedded, adarms_cond, strict=True)
        ], kv_cache

    def forward_with_attention(
        self,
        embedded: Sequence[at.Float[at.Array, "b _t _d"] | None],
        positions: at.Int[at.Array, "b t"],
        mask: at.Bool[at.Array, "b t s"],
        adarms_cond: Sequence[at.Float[at.Array, "b _d"] | None] | None = None,
        *,
        kv_cache: KVCache | None = None,
        deterministic: bool = True,
    ):
        """Forward pass that also returns approximate attention weights.

        This method computes attention weights by performing a simplified attention
        calculation on the final layer outputs. Since we can't easily extract attention
        from the scanned layers, we approximate by computing Q*K^T on the output embeddings.

        Returns:
            output: List of output tensors for each expert
            kv_cache: Updated KV cache
            attn_probs: Approximate attention probabilities [B, T, S]
        """
        embedded = jax.tree.map(lambda e: e.astype(self.embed_dtype), embedded)
        mask_4d = jnp.asarray(mask)[:, None, :, :]
        if adarms_cond is None:
            adarms_cond = [None] * len(self.configs)

        # Run all layers through scan
        embedded, kv_cache = self.layers(embedded, kv_cache, positions, mask_4d, adarms_cond, deterministic)

        # Compute approximate attention weights from the output embeddings
        # Use only the first expert (PaliGemma) embeddings which contain image+text tokens
        paligemma_embedded = embedded[0]  # [B, prefix_len, 2048]

        if paligemma_embedded is not None:
            # Simple dot-product attention approximation: softmax(Q @ K^T / sqrt(d))
            # Using the embeddings as both Q and K
            d = paligemma_embedded.shape[-1]
            attn_logits = jnp.einsum("btd,bsd->bts", paligemma_embedded, paligemma_embedded) / jnp.sqrt(d)

            # Create mask for the prefix tokens only
            prefix_len = paligemma_embedded.shape[1]
            mask_2d = mask[:, :prefix_len, :prefix_len]  # [B, prefix_len, prefix_len]
            big_neg = -2.3819763e38
            masked_logits = jnp.where(mask_2d, attn_logits, big_neg)
            attn_probs = jax.nn.softmax(masked_logits, axis=-1)
        else:
            attn_probs = None

        assert all(e.dtype == jnp.dtype(self.embed_dtype) for e in embedded if e is not None)

        output = [
            f(e, a)[0] if e is not None else e for f, e, a in zip(self.final_norms, embedded, adarms_cond, strict=True)
        ]
        return output, kv_cache, attn_probs

    def forward_with_real_attention(
        self,
        embedded: Sequence[at.Float[at.Array, "b _t _d"] | None],
        positions: at.Int[at.Array, "b t"],
        mask: at.Bool[at.Array, "b t s"],
        adarms_cond: Sequence[at.Float[at.Array, "b _d"] | None] | None = None,
        *,
        kv_cache: KVCache | None = None,
        deterministic: bool = True,
    ):
        """Forward pass extracting cross-attention using the last layer's Q/K projections.

        Runs the full forward pass through all transformer layers, then manually applies
        the last layer's pre-attention norm and Q/K projection weights to compute real
        cross-attention probabilities (including action→image attention).

        Returns:
            output: List of output tensors for each expert.
            kv_cache: Updated KV cache.
            attn_probs: Attention probabilities [B, K, G, T, S] from the last layer's projections.
        """
        embedded = jax.tree.map(lambda e: e.astype(self.embed_dtype), embedded)
        mask_4d = jnp.asarray(mask)[:, None, :, :]
        if adarms_cond is None:
            adarms_cond = [None] * len(self.configs)

        # Run all layers through scan (correctly applies LoRA, residuals, etc.)
        embedded, kv_cache = self.layers(embedded, kv_cache, positions, mask_4d, adarms_cond, deterministic)

        # --- Extract cross-attention from the last layer's Q/K projections ---
        # Access scanned params and get last layer
        scan_params = self.variables['params']['layers']
        last_layer = jax.tree.map(lambda x: x[-1], scan_params)

        # Apply pre-attention RMSNorm to each expert's output
        def _rms_norm(x, scale):
            var = jnp.mean(jnp.square(x.astype(jnp.float32)), axis=-1, keepdims=True)
            normed = x * jnp.reciprocal(jnp.sqrt(var + 1e-06))
            return (normed * (1 + scale)).astype(x.dtype)

        # Compute Q and K for each expert using the last layer's projection weights + LoRA
        def _apply_lora(result, x, params_dict, eqn_a, eqn_b, lora_config):
            """Apply LoRA delta: result += (x @ w_a) @ w_b * scaling."""
            if 'lora_a' not in params_dict or lora_config is None:
                return result
            w_a = params_dict['lora_a'].astype(x.dtype)
            w_b = params_dict['lora_b'].astype(x.dtype)
            scaling = lora_config.alpha / lora_config.rank
            lora_out = jnp.einsum(eqn_a, x, w_a)
            lora_out = jnp.einsum(eqn_b, lora_out, w_b)
            return result + lora_out * scaling

        qkvs = []
        for i, (x, config) in enumerate(zip(embedded, self.configs, strict=True)):
            if x is None:
                continue
            # Apply pre-attention norm
            norm_key = "pre_attention_norm" if i == 0 else f"pre_attention_norm_{i}"
            x_normed = _rms_norm(x, last_layer[norm_key]['scale'])

            attn_lora = config.lora_configs.get("attn")

            if config.num_kv_heads == config.num_heads:
                qkv_name = f"qkv_einsum{'_' + str(i) if i > 0 else ''}"
                qkv_params = last_layer['attn'][qkv_name]
                qkv = jnp.einsum("BSD,3KDH->3BSKH", x_normed, qkv_params['w'].astype(x_normed.dtype))
                qkv = _apply_lora(qkv, x_normed, qkv_params,
                                  "BSD,3KDL->3BSKL", "3BSKL,3KLH->3BSKH", attn_lora)
                qkvs.append(qkv)
            else:
                q_name = "q_einsum" if i == 0 else f"q_einsum_{i}"
                kv_name = "kv_einsum" if i == 0 else f"kv_einsum_{i}"
                q_params = last_layer['attn'][q_name]
                kv_params = last_layer['attn'][kv_name]

                # Q: base + LoRA
                q = jnp.einsum("BTD,NDH->BTNH", x_normed, q_params['w'].astype(x_normed.dtype))
                q = _apply_lora(q, x_normed, q_params,
                                "BTD,NDL->BTNL", "BTNL,NLH->BTNH", attn_lora)

                # KV: base + LoRA
                kv = jnp.einsum("BSD,2KDH->2BSKH", x_normed, kv_params['w'].astype(x_normed.dtype))
                kv = _apply_lora(kv, x_normed, kv_params,
                                 "BSD,2KDL->2BSKL", "2BSKL,2KLH->2BSKH", attn_lora)
                k, v = kv
                qkvs.append((q, k, v))

        # Concatenate Q, K, V from all experts along sequence dimension
        q, k, v = (jnp.concatenate(y, axis=1) for y in zip(*qkvs, strict=True))

        # Apply RoPE
        q = _apply_rope(q, positions=positions)
        q *= self.configs[0].head_dim ** -0.5
        k = _apply_rope(k, positions=positions)

        # Compute attention logits (GQA: group query heads by KV heads)
        q = einops.rearrange(q, "B T (K G) H -> B T K G H", K=self.configs[0].num_kv_heads)
        logits = jnp.einsum("BTKGH,BSKH->BKGTS", q, k, preferred_element_type=jnp.float32)

        # Apply mask and softmax
        big_neg = -2.3819763e38
        masked_logits = jnp.where(mask_4d[:, :, None, :, :], logits, big_neg)
        attn_probs = jax.nn.softmax(masked_logits, axis=-1)
        # attn_probs: [B, K, G, T, S] = [B, num_kv_heads, num_heads/num_kv_heads, total_len, total_len]

        assert all(e.dtype == jnp.dtype(self.embed_dtype) for e in embedded if e is not None)

        output = [
            f(e, a)[0] if e is not None else e
            for f, e, a in zip(self.final_norms, embedded, adarms_cond, strict=True)
        ]
        return output, kv_cache, attn_probs, q, k

    def forward_with_exact_attention(
        self,
        embedded: Sequence[at.Float[at.Array, "b _t _d"] | None],
        positions: at.Int[at.Array, "b t"],
        mask: at.Bool[at.Array, "b t s"],
        adarms_cond: Sequence[at.Float[at.Array, "b _d"] | None] | None = None,
        *,
        kv_cache: KVCache | None = None,
        deterministic: bool = True,
    ):
        """Forward pass with EXACT last-layer attention extraction.

        Runs the first N-1 layers via raw param replay to get the exact input to
        the last layer, then computes Q/K projections on that correct input.
        Unlike forward_with_real_attention (which applies last-layer weights to
        the output of ALL layers), this method applies them to the correct input.
        """
        embedded = jax.tree.map(lambda e: e.astype(self.embed_dtype), embedded)
        mask_4d = jnp.asarray(mask)[:, None, :, :]
        if adarms_cond is None:
            adarms_cond = [None] * len(self.configs)

        # --- Run first N-1 layers to get the exact input to the last layer ---
        scan_params = self.variables['params']['layers']
        n_layers = self.configs[0].depth

        embedded = _run_block_layers_pure(
            self.configs, scan_params, list(embedded), positions, mask_4d, adarms_cond,
            end_layer=n_layers - 1,
        )

        # --- Compute Q/K using last layer's weights on the CORRECT input ---
        last_layer = jax.tree.map(lambda x: x[-1], scan_params)

        def _rms_norm(x, scale):
            var = jnp.mean(jnp.square(x.astype(jnp.float32)), axis=-1, keepdims=True)
            normed = x * jnp.reciprocal(jnp.sqrt(var + 1e-06))
            return (normed * (1 + scale)).astype(x.dtype)

        def _apply_lora(result, x, params_dict, eqn_a, eqn_b, lora_config):
            if 'lora_a' not in params_dict or lora_config is None:
                return result
            w_a = params_dict['lora_a'].astype(x.dtype)
            w_b = params_dict['lora_b'].astype(x.dtype)
            scaling = lora_config.alpha / lora_config.rank
            lora_out = jnp.einsum(eqn_a, x, w_a)
            lora_out = jnp.einsum(eqn_b, lora_out, w_b)
            return result + lora_out * scaling

        qkvs = []
        for i, (x, config) in enumerate(zip(embedded, self.configs, strict=True)):
            if x is None:
                continue
            norm_key = "pre_attention_norm" if i == 0 else f"pre_attention_norm_{i}"
            x_normed = _rms_norm(x, last_layer[norm_key]['scale'])
            attn_lora = config.lora_configs.get("attn")

            if config.num_kv_heads == config.num_heads:
                qkv_name = f"qkv_einsum{'_' + str(i) if i > 0 else ''}"
                qkv_params = last_layer['attn'][qkv_name]
                qkv = jnp.einsum("BSD,3KDH->3BSKH", x_normed, qkv_params['w'].astype(x_normed.dtype))
                qkv = _apply_lora(qkv, x_normed, qkv_params,
                                  "BSD,3KDL->3BSKL", "3BSKL,3KLH->3BSKH", attn_lora)
                qkvs.append(qkv)
            else:
                q_name = "q_einsum" if i == 0 else f"q_einsum_{i}"
                kv_name = "kv_einsum" if i == 0 else f"kv_einsum_{i}"
                q_params = last_layer['attn'][q_name]
                kv_params = last_layer['attn'][kv_name]
                q = jnp.einsum("BTD,NDH->BTNH", x_normed, q_params['w'].astype(x_normed.dtype))
                q = _apply_lora(q, x_normed, q_params,
                                "BTD,NDL->BTNL", "BTNL,NLH->BTNH", attn_lora)
                kv = jnp.einsum("BSD,2KDH->2BSKH", x_normed, kv_params['w'].astype(x_normed.dtype))
                kv = _apply_lora(kv, x_normed, kv_params,
                                 "BSD,2KDL->2BSKL", "2BSKL,2KLH->2BSKH", attn_lora)
                k, v = kv
                qkvs.append((q, k, v))

        q, k, v = (jnp.concatenate(y, axis=1) for y in zip(*qkvs, strict=True))

        q = _apply_rope(q, positions=positions)
        q *= self.configs[0].head_dim ** -0.5
        k = _apply_rope(k, positions=positions)

        q = einops.rearrange(q, "B T (K G) H -> B T K G H", K=self.configs[0].num_kv_heads)
        logits = jnp.einsum("BTKGH,BSKH->BKGTS", q, k, preferred_element_type=jnp.float32)

        big_neg = -2.3819763e38
        masked_logits = jnp.where(mask_4d[:, :, None, :, :], logits, big_neg)
        attn_probs = jax.nn.softmax(masked_logits, axis=-1)

        return None, None, attn_probs, q, k

    def init(self, use_adarms: Sequence[bool]):
        """Convenience method for initializing all parameters, necessary due to the quirks of linen."""
        self.embed(jnp.zeros((1, 1), dtype=jnp.int32))
        self(
            [jnp.zeros((1, 1, c.width)) for c in self.configs],
            jnp.zeros((1, len(self.configs)), dtype=jnp.int32),
            jnp.zeros((1, len(self.configs), len(self.configs)), dtype=bool),
            adarms_cond=[jnp.zeros((1, c.width)) if u else None for u, c in zip(use_adarms, self.configs, strict=True)],
        )


def _apply_rope(x, *, positions, max_wavelength=10_000):
    """Applies RoPE positions [B, L] to x [B, L, H, D]."""
    freq_exponents = (2.0 / x.shape[-1]) * jnp.arange(x.shape[-1] // 2, dtype=jnp.float32)
    timescale = max_wavelength**freq_exponents
    radians = positions[..., None] / timescale[None, None, :]
    radians = radians[..., None, :]
    assert radians.dtype == jnp.float32
    # radians.shape = [...,L,1,d=D/2]
    sin, cos = jnp.sin(radians), jnp.cos(radians)
    x1, x2 = jnp.split(x, 2, axis=-1)
    res = jnp.concatenate([x1 * cos - x2 * sin, x2 * cos + x1 * sin], axis=-1)
    assert res.dtype == jnp.float32
    # The original bigvision impl allows RoPE to upcast to float32. It is then immediately downcast again to the cache
    # dtype when in inference mode (but not in training mode). I don't think any of this was intentional. Based on the
    # original DeepMind impl, as well as the widely-used transformers impl, it is ok to always downcast back to bfloat16
    # here.
    return res.astype(x.dtype)


def _name(name, i):
    # we name layers like this because we want the first expert's weights to have no suffix (e.g., "attn"), so that they
    # can be loaded seamlessly from the existing PaliGemma checkpoint. subsequent experts will have a suffix (e.g.,
    # "attn_1") and their weights will be initialized from scratch. in practice, we only use two experts -- PaliGemma,
    # and the action expert.
    if i == 0:
        return name
    return f"{name}_{i}"


def _gated_residual(x, y, gate):
    assert (x is None) == (y is None)
    if x is None:
        return None
    if gate is None:
        return x + y
    return x + y * gate


def _pure_block_step(xs, layer_params, configs, positions, mask_4d, adarms_cond):
    """Pure JAX implementation of one transformer Block layer (no Flax modules).

    Replicates Block.__call__ using only raw param dicts and JAX operations,
    so it can be called from any context without Flax scope restrictions.
    """

    def _rms_norm(x, norm_params, cond):
        var = jnp.mean(jnp.square(x.astype(jnp.float32)), axis=-1, keepdims=True)
        normed = x * jnp.reciprocal(jnp.sqrt(var + 1e-06))
        if cond is None:
            return (normed * (1 + norm_params['scale'])).astype(x.dtype), None
        # AdaRMS
        dp = norm_params['Dense_0']
        mod = jnp.dot(cond.astype(x.dtype), dp['kernel'].astype(x.dtype))
        if 'bias' in dp:
            mod = mod + dp['bias'].astype(x.dtype)
        mod = mod[:, None, :]
        scale, shift, gate = jnp.split(mod, 3, axis=-1)
        return (normed * (1 + scale) + shift).astype(x.dtype), gate

    def _lora(result, x, p, eqn_a, eqn_b, lora_cfg):
        if lora_cfg is None or 'lora_a' not in p:
            return result
        s = lora_cfg.alpha / lora_cfg.rank
        return result + jnp.einsum(eqn_b, jnp.einsum(eqn_a, x, p['lora_a'].astype(x.dtype)),
                                   p['lora_b'].astype(x.dtype)) * s

    # === 1. Pre-attention RMSNorm ===
    pre_attn, attn_gates = [], []
    for i, (x, cfg) in enumerate(zip(xs, configs)):
        if x is None:
            pre_attn.append(None); attn_gates.append(None); continue
        nk = "pre_attention_norm" if i == 0 else f"pre_attention_norm_{i}"
        xn, g = _rms_norm(x, layer_params[nk], adarms_cond[i])
        pre_attn.append(xn); attn_gates.append(g)

    # === 2. Multi-expert Attention ===
    dtype = next(x.dtype for x in pre_attn if x is not None)
    qkvs = []
    for i, (x, cfg) in enumerate(zip(pre_attn, configs)):
        if x is None:
            continue
        al = cfg.lora_configs.get("attn")
        if cfg.num_kv_heads == cfg.num_heads:
            n = f"qkv_einsum{'_' + str(i) if i > 0 else ''}"
            p = layer_params['attn'][n]
            qkv = jnp.einsum("BSD,3KDH->3BSKH", x, p['w'].astype(dtype))
            qkv = _lora(qkv, x, p, "BSD,3KDL->3BSKL", "3BSKL,3KLH->3BSKH", al)
            qkvs.append(qkv)
        else:
            qn = "q_einsum" if i == 0 else f"q_einsum_{i}"
            kn = "kv_einsum" if i == 0 else f"kv_einsum_{i}"
            qp, kvp = layer_params['attn'][qn], layer_params['attn'][kn]
            q = _lora(jnp.einsum("BTD,NDH->BTNH", x, qp['w'].astype(dtype)),
                      x, qp, "BTD,NDL->BTNL", "BTNL,NLH->BTNH", al)
            kv = _lora(jnp.einsum("BSD,2KDH->2BSKH", x, kvp['w'].astype(dtype)),
                       x, kvp, "BSD,2KDL->2BSKL", "2BSKL,2KLH->2BSKH", al)
            k, v = kv
            qkvs.append((q, k, v))

    q, k, v = (jnp.concatenate(y, axis=1) for y in zip(*qkvs, strict=True))
    q = _apply_rope(q, positions=positions)
    q *= configs[0].head_dim ** -0.5
    k = _apply_rope(k, positions=positions)

    # GQA attention
    q_gqa = einops.rearrange(q, "B T (K G) H -> B T K G H", K=configs[0].num_kv_heads)
    logits = jnp.einsum("BTKGH,BSKH->BKGTS", q_gqa, k, preferred_element_type=jnp.float32)
    big_neg = -2.3819763e38
    probs = jax.nn.softmax(jnp.where(mask_4d[:, :, None, :, :], logits, big_neg), axis=-1).astype(dtype)
    encoded = jnp.einsum("BKGTS,BSKH->BTKGH", probs, v)
    encoded = einops.rearrange(encoded, "B T K G H -> B T (K G) H")

    # Output projection
    post_attn, start = [], 0
    for i, (x, cfg) in enumerate(zip(pre_attn, configs)):
        if x is not None:
            end = start + x.shape[1]
            on = "attn_vec_einsum" if i == 0 else f"attn_vec_einsum_{i}"
            op = layer_params['attn'][on]
            al = cfg.lora_configs.get("attn")
            o = _lora(jnp.einsum("BTNH,NHD->BTD", encoded[:, start:end], op['w'].astype(dtype)),
                      encoded[:, start:end], op, "BTNH,NHL->BTL", "BTL,NLD->BTD", al)
            post_attn.append(o); start = end
        else:
            post_attn.append(None)

    xs = [_gated_residual(x, y, g) for x, y, g in zip(xs, post_attn, attn_gates, strict=True)]

    # === 3. Pre-FF RMSNorm + FeedForward ===
    pre_ff, ff_gates = [], []
    for i, (x, cfg) in enumerate(zip(xs, configs)):
        if x is None:
            pre_ff.append(None); ff_gates.append(None); continue
        nk = "pre_ffw_norm" if i == 0 else f"pre_ffw_norm_{i}"
        xn, g = _rms_norm(x, layer_params[nk], adarms_cond[i])
        pre_ff.append(xn); ff_gates.append(g)

    ff_out = []
    for i, (x, cfg) in enumerate(zip(pre_ff, configs)):
        if x is None:
            ff_out.append(None); continue
        mk = "mlp" if i == 0 else f"mlp_{i}"
        mp = layer_params[mk]
        fl = cfg.lora_configs.get("ffn")
        wg = mp['gating_einsum'].astype(dtype)
        wl = mp['linear'].astype(dtype)
        fg = jnp.dot(x, wg[0])
        f1 = jnp.dot(x, wg[1])
        if fl and 'gating_einsum_lora_a' in mp:
            la, lb = mp['gating_einsum_lora_a'].astype(dtype), mp['gating_einsum_lora_b'].astype(dtype)
            s = fl.alpha / fl.rank
            fg = fg + jnp.dot(jnp.dot(x, la[0]), lb[0]) * s
            f1 = f1 + jnp.dot(jnp.dot(x, la[1]), lb[1]) * s
        out = jnp.dot(jax.nn.gelu(fg) * f1, wl)
        if fl and 'linear_lora_a' in mp:
            la, lb = mp['linear_lora_a'].astype(dtype), mp['linear_lora_b'].astype(dtype)
            s = fl.alpha / fl.rank
            out = out + jnp.dot(jnp.dot(jax.nn.gelu(fg) * f1, la), lb) * s
        ff_out.append(out)

    xs = [_gated_residual(x, y, g) for x, y, g in zip(xs, ff_out, ff_gates, strict=True)]
    return xs


def _run_block_layers_pure(configs, scan_params, xs, positions, mask_4d, adarms_cond, end_layer):
    """Run transformer layers [0, end_layer) using pure JAX (no Flax modules)."""
    for i in range(end_layer):
        layer_params = jax.tree.map(lambda x, _i=i: x[_i], scan_params)
        xs = _pure_block_step(xs, layer_params, configs, positions, mask_4d, adarms_cond)
    return xs
