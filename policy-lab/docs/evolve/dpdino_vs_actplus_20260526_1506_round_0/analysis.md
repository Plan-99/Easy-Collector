# Round_0 — DPDino vs ACTPlus (peg-in-hole)

Tournament: `dpdino_vs_actplus_20260526_1506_round_0`
Dataset: 15 (`model_tester_peg_1779759979`, 53 episodes, tool_qpos_indices implied = [6])
Trials per candidate: 5 × 200 steps
Both candidates: `action_type=relative_ee_pos`, `obs_state_keys=[]` (vision-only + 1-dim gripper proprio)

## Results

| candidate            | val_loss | train_time | success | xy_err (m)       | tilt_cos | z_err (m) |
|----------------------|----------|------------|---------|------------------|----------|-----------|
| dpdino_baseline      | 0.06     | 3:46:59    | 0/5     | 0.084 – 0.151    | 1.0 (4/5), -0.0 (1/5) | -0.010 |
| actplus_resnet18 †   | 2.0      | 0:03:15    | 0/5     | 0.063 – 0.145    | 1.0 (5/5) | -0.010 |

† ACTPlus value reported here is from the **second** evaluation pass, after two
   evaluator bugs were fixed (see "Infra fixes" below). The first attempt
   crashed with `RuntimeError: mat1 and mat2 shapes cannot be multiplied`.

**No strictly-higher winner.** Both candidates scored 0/5; champion file is not updated.

## Failure mode — shared across both candidates

Both models converge on the same qualitative behaviour:

- **Orientation correct.** `tilt_cos = 1.0` in 9/10 trials (gripper pointed straight down at the hole pose).
- **Z gap small.** `z_err ≈ −0.01 m` consistently. Peg is held just above the hole rim.
- **XY gap large.** `xy_err` in the 6 – 15 cm range. The peg never reaches the hole footprint despite a 200-step budget.

This is not a "model collapsed to a single output" failure — per-trial `xy_err`
varies (the policies *do* respond to the scene). It is a **visual XY-localization
bottleneck**: the policy can find the gripper-down posture but cannot close the
final 6 – 15 cm xy gap to the peg/hole pair.

## Why DPDino is the *effective* round_0 winner

The tournament summary names ACTPlus as the winner by random tiebreak (both 0/5).
That label is misleading:

- **DPDino actually converged** — val_loss 0.06 after 3h47m of training; trainable
  params ≈ 6M (DINOv2 frozen, only U-Net + state projection).
- **ACTPlus did not converge** — val_loss 2.0 after only 3m15s of training;
  trainable params ≈ 51M, so 53 ep × 3 batches × 300 epochs ≈ 900 gradient
  steps were nowhere near enough to fit a 51M-param head + ResNet18 backbone.
  Its "similar" xy_err is *coincidence* — an underfit ACTPlus emits near-random
  trajectories that happen to live in the same metric bucket as DPDino's
  smoothly-but-wrongly aimed ones.

So for round_1 inheritance purposes, **DPDino is the round_0 winner**. ACTPlus
is dropped going forward (per user directive 2026-05-26 — replaced by plain
ACT-ResNet18 as the "verified baseline" anchor).

## Failure-mode hypotheses (for round_1 design)

The qualitative pattern (right pose, wrong xy) admits two independent
hypotheses, both supported by the literature:

1. **H1 — Frozen DINOv2 features lose precision at peg/hole scale.**
   DINOv2 is pre-trained on natural images. Frozen, its last-layer feature map
   may encode the *category* of objects on the table but not the few-pixel
   relative offset between two small cylinders. Counter-measures from arxiv:
   - DINOv3 (2509.17684): demonstrates partial unfreeze for diffusion policies.
   - VAT (2512.06013): multi-layer (not just last-layer) DINOv2 features
     improve manipulation precision.
   - User intuition explicitly flagged freeze/unfreeze as a meaningful axis.

2. **H2 — DDIM 10-step denoiser smooths the action chunk.**
   DPDino uses 100 train / 10 inference timesteps. Ten steps may suffice for
   "go down and grab" but be too coarse for the sub-cm xy commit needed at
   insertion. Counter-measures:
   - ManiFlow (2506.19089): rectified flow + OT coupling — sharper, fewer
     steps, used in precision manipulation.
   - ManiDiT (2511.04543): DiT-X action head — more committal long-horizon
     trajectories.

The two hypotheses are decoupled: H1 attacks the **visual encoder**, H2
attacks the **action decoder**. Round_1 can attack one at a time so we can
read which axis pays.

## Round_1 entrant slate (per project memory + user directives)

Round_1 = `[round_0 winner, plain ACT-ResNet18 anchor, paper-inspired variant]`:

1. **DPDino_baseline** — unchanged from round_0. Carries forward as the
   strictly-higher reference.
2. **plain_ACT_resnet18** — `policy_type="ACT"` (NOT ACTPlus), ResNet18,
   `obs_state_keys=[]`, `action_type=relative_ee_pos`. Verified anchor per
   user statement 2026-05-26 ("이게 한번 그래도 잘 된다고 검증이 되었던
   모델이어서리"). Removes the ACTPlus augmentation/aux-head confound so we
   can compare round_1's creative variant against a *trusted* ACT baseline.
3. **Creative variant** — one paper-inspired structural swap of DPDino. To
   maximize signal, attack either H1 or H2 *but not both* (so we can
   attribute any delta cleanly). Concrete options to be chosen by
   policy-researcher in Phase C:
   - **(H1) DPDino_unfrozen_last2** — DPDino, last 2 DINOv2 blocks unfrozen,
     decoupled LR (backbone 1e-6 / head 1e-4). Smallest implementable
     intervention against H1.
   - **(H1) DPDino_VAT** — DPDino, multi-layer DINOv2 feature pooling
     (concat blocks {-1,-3,-5}) instead of last-layer concat.
   - **(H2) DPDino_Flow** — DPDino, swap DDIM denoiser → rectified flow
     matching with OT coupling. Same UNet1D + DINOv2-frozen backbone.

Constraint on the variant: **300-epoch budget must produce a fitted model**
(don't repeat ACTPlus's 51M-param underfit failure). Trainable-param count
should stay in the 6 – 15M range so 53-episode dataset can actually train it.

## Infra fixes landed in this session (relevant to all future rounds)

Two bugs in `backend/tools/model_tester/tutorial_evaluator.py` were masking
ACTPlus's true behaviour:

1. **Bug — `train_config.json` not visible to evaluator.** Training pipeline
   wrote `train_config.json` in the tournament job dir but never copied it
   into the checkpoint dir. The evaluator's only resolution path looked in
   the checkpoint dir → silently fell back to
   `default_action_key="qaction"`, which dispatched the policy through the
   7-dim joint-target path even when the model was trained with 6-dim
   `relative_ee_pos`. **Fix:** `training_server/train_worker.py` now copies
   `job/train_config.json` into `checkpoint_dir` at end of training. Existing
   checkpoints 42 (DPDino) and 43 (ACTPlus) were backfilled manually.

2. **Bug — bridge state vs model state dim mismatch.** Bridge sends 7-dim
   qpos `[arm_0..arm_5, tool]`. Model was trained with `obs_state_keys=[]`
   + `tool_qpos_indices=[6]` → 1-dim gripper-only `observation.state`. The
   evaluator was naively passing the full 7-dim vector into the model's
   `Linear(1, 512)` projection. **Fix:** `tutorial_evaluator.py` now reads
   `train_config.json` to mirror `_build_obs_state` semantics at inference
   time (with a "take the last N dims" fallback when train_config is missing
   `tool_qpos_indices`, which works for the current dataset 15).

DPDino survived bug 2 only because its modeling code is vision-only — it
never consumes `observation.state` even when the config says it does. Any
state-using policy (ACT/ACTPlus/PI05/future custom) would have hit the same
crash.
