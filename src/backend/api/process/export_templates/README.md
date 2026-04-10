# Exported EasyTrainer Checkpoint

This bundle contains everything needed to run inference with a trained
EasyTrainer policy outside the EasyTrainer environment. Three usage modes:

1. **Single-step Python API** (`inference.py`) — call from your own control
   loop with a state vector + image dict, get an action vector back.
2. **Closed-loop ROS 2 inference** (`ros_inference.py`) — full main loop
   matching EasyTrainer's `checkpoint_test`. Subscribes to your robot/camera
   topics, runs inference at a fixed rate, publishes joint commands. No DB,
   no Flask, no EasyTrainer container.
3. **Service-based ROS 2 node** (`ros_inference_service.py`) — same setup as
   Mode 2 but the model only runs when an external client calls a ROS 2
   service. The node stays warm in the background. Useful when an upstream
   planner / state machine wants to step the policy on demand.

## Layout

```
.
├── inference.py                 # CheckpointInference class + 1-step CLI
├── ros_inference.py             # closed-loop ROS 2 main loop
├── ros_inference_service.py     # service-based ROS 2 node (on-demand)
├── export_meta.json             # task / sensor / robot / policy metadata
├── requirements.txt             # Python deps to install
├── README.md                    # this file
├── model/                       # the actual checkpoint
│   ├── config.json
│   ├── model.safetensors
│   ├── policy_preprocessor.json
│   ├── policy_preprocessor_*.safetensors
│   ├── policy_postprocessor.json
│   ├── policy_postprocessor_*.safetensors
│   └── dataset_stats.pkl
├── lerobot/                     # vendored lerobot package (matches training)
└── easytrainer_runtime/         # SimpleAgent + SimpleEnv + image_parser
    ├── simple_agent.py
    ├── simple_env.py
    └── image_parser.py
```

## Setup

```bash
python3 -m venv venv
source venv/bin/activate
pip install -r requirements.txt
```

A CUDA-enabled GPU is strongly recommended. CPU works for testing but is
several orders of magnitude slower.

`ros_inference.py` additionally requires **ROS 2 installed on the host**
(e.g. Humble) so that `rclpy` and `sensor_msgs` are importable. Source the
ROS 2 setup before running the venv:

```bash
source /opt/ros/humble/setup.bash
source venv/bin/activate
python ros_inference.py
```

---

## Mode 1: Single-step Python API

Use this when you want full control over how the robot is driven and where the
images come from. The class is the same one EasyTrainer uses internally.

```python
from inference import CheckpointInference
import numpy as np
from PIL import Image

inf = CheckpointInference("./model", "./export_meta.json")

# Call once at the start of every episode to clear the temporal ensembler.
inf.reset()

# state: (state_dim,) raw joint positions in the same units as training
state = np.array([0.0, 0.10, -0.60, 0.0, 1.00, 0.0, 0.05], dtype=np.float32)

# images: one entry per sensor declared in export_meta.json["sensor_ids"]
# Resolution doesn't matter — they get resized to 224x224 internally.
images = {
    "sensor_1": np.array(Image.open("cam1.png").convert("RGB")),
    "sensor_2": np.array(Image.open("cam2.png").convert("RGB")),
    "sensor_3": np.array(Image.open("cam3.png").convert("RGB")),
}

action = inf.infer(state, images)   # (action_dim,) float32 in raw units
print(action)
```

In your robot control loop, call `infer()` every step (or every N steps if
your model uses temporal ensembling). Reset between episodes.

There is also a CLI sanity check:

```bash
python inference.py \
    --state "0,0.1,-0.6,0,1,0,0.05" \
    --image sensor_1=cam1.png \
    --image sensor_2=cam2.png \
    --image sensor_3=cam3.png
```

---

## Mode 2: Closed-loop ROS 2 inference

Use this when your robot already publishes joint state and accepts joint
command messages over ROS 2 topics, and you want the bundle to drive it
autonomously the same way EasyTrainer does.

**The set of robots and sensors is fixed by the checkpoint.** The checkpoint
was trained for a specific task, which is bound to a specific assembly
(left/right arm + tools + optional mobile base). The model's output layer is
sized for exactly that assembly, so there's no "subset" flag — the script
drives all robots in `export_meta.json["robots"]` and reads all sensors in
`export_meta.json["sensors"]` every step.

```bash
# Drive the assembly at 10 Hz until Ctrl-C
python ros_inference.py

# Custom rate + step limit
python ros_inference.py --hz 15 --episode-len 300

# Dry run: compute actions but don't publish anything
python ros_inference.py --dry-run
```

What it does, step by step:

1. Reads `export_meta.json` to discover the assembly's robots, the task's
   sensors, per-sensor crop/rotate/resize, the policy type, etc.
2. Initialises `rclpy` and creates an `easytrainer_exported_inference` node.
3. Builds one `SimpleAgent` per robot in the assembly. Each agent subscribes
   to the robot's `read_topic` (joint state) and creates a publisher on the
   `write_topic` (joint command).
4. Builds a `SimpleEnv` that subscribes to every task sensor's image topic
   and decodes frames into BGR numpy arrays.
5. Loads the policy via `CheckpointInference` (same class as Mode 1).
6. Loops at the requested Hz: snapshot observation → apply per-sensor
   crop/rotate/resize → run inference → split action across agents →
   publish joint commands.
7. On Ctrl-C, stops cleanly.

The published action format is whatever the robot's `write_topic_msg`
expects (`sensor_msgs/JointState` with `position` filled, or
`trajectory_msgs/JointTrajectoryPoint` with `positions` filled).

### Limitations of ros_inference.py

This script is intentionally a slim subset of EasyTrainer's full
`checkpoint_test`. It bails out at startup if your checkpoint or robot
config hits one of these:

- **`action_key` must be `qaction`.** `ee_delta_action` and
  `relative_ee_pos` need an IK solver and per-robot URDF files. Use
  EasyTrainer for those modes.
- **`vision_backbone` must be `resnet18`.** dinov2/dinov3 require
  HuggingFace Hub access at first load.
- **Robot `write_type` must be `topic`.** Service / action goal robots
  (some industrial arms) are not supported here.
- **Robot read/write messages must be `sensor_msgs/JointState`** (or
  `trajectory_msgs/JointTrajectoryPoint`). Custom message types need
  custom parsing — see `easytrainer_runtime/simple_agent.py`.
- **No homepose movement.** Move the robot to its starting pose manually
  (or via your own teach pendant) before launching the script. EasyTrainer
  did this through robot-specific motion APIs that aren't bundled here.
- **No failure detection / OOD scoring / Grad-CAM / OTI-RL / socketio
  broadcasts.** Just inference and joint commands.
- **`has_succeed=true`** checkpoints automatically strip the trailing
  succeed bit before publishing. The succeed score is logged to stdout
  in `--dry-run` mode but not broadcast anywhere.

If your setup hits any of these limits, run inference inside EasyTrainer.

---

## Mode 3: Service-based ROS 2 node

Use this when an external program (planner, state machine, GUI button) needs
to step the policy on demand instead of running it autonomously at a fixed
rate. The node loads the model once at startup and stays warm; each service
call takes a fresh observation snapshot, runs one inference step, publishes
the resulting joint command, and returns success/message.

```bash
# Bring the node up — model loads once, then it idles waiting for calls
python ros_inference_service.py

# Custom node name (lets you run several side by side)
python ros_inference_service.py --node-name my_policy

# Dry run: compute & log actions but don't publish to the robot
python ros_inference_service.py --dry-run
```

From another terminal (or any ROS 2 client) call the service:

```bash
# Run one inference step
ros2 service call /easytrainer_inference_service/run_inference \
    std_srvs/srv/Trigger

# Reset the temporal ensembler / action queue at the start of an episode
ros2 service call /easytrainer_inference_service/reset \
    std_srvs/srv/Trigger
```

The response is a `std_srvs/srv/Trigger.Response`:
- `success`: `true` if inference completed without exception
- `message`: human-readable summary including the joint command, e.g.
  `step 0: action=[0.205, 0.840, -0.846, ..., 0.029]`

The same EasyTrainer policy / preprocessor / postprocessor is loaded as in
Modes 1 and 2 — bit-for-bit identical outputs given identical inputs.

### Service node design notes

- Joint state and image subscriptions live in a `ReentrantCallbackGroup` and
  the executor is `MultiThreadedExecutor`, so observations keep updating in
  the background even while a service call is in flight. Each call always
  sees the freshest frame.
- Concurrent service calls are serialised behind an internal lock — CUDA and
  the temporal ensembler are not thread-safe. Two simultaneous callers will
  not double-publish.
- All limitations from Mode 2 apply (qaction, resnet18, write_type=topic,
  supported message types).

---

## Input / output shapes

| field   | shape            | dtype   | units                       |
|---------|------------------|---------|-----------------------------|
| state   | `(state_dim,)`   | float32 | raw joint positions         |
| image   | `(H, W, 3)`      | uint8   | RGB, any resolution         |
| action  | `(action_dim,)`  | float32 | raw joint positions / cmds  |

`state_dim` and `action_dim` are recorded in `export_meta.json`.

If `has_succeed=true`, `action_dim` includes a trailing succeed bit.
`ros_inference.py` strips it automatically. The Python class returns it
to the caller — strip it yourself before sending to the robot.

---

## Verifying the bundle

To confirm the exported model produces the same actions as the original
EasyTrainer instance, pick a frame from your training dataset and run the
same `state + images` through both. The outputs should match within float32
rounding error (~1e-5).

For Mode 2, the simplest sanity check is:

```bash
python ros_inference.py --dry-run --episode-len 5
```

This runs five inference steps using live ROS topic data and prints the
joint commands without sending them. Make sure your robot driver and
camera nodes are running before launching.
