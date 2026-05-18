# Exported EasyTrainer Planner

This bundle runs a complete EasyTrainer **planner** outside the EasyTrainer
container — no Flask backend, no database, no gRPC bridge. Robots are driven
directly over ROS 2 topics and the planner's checkpoints run through the
bundled inference code.

## Contents

```
.
├── run_planner.py            # autonomous runner — run the planner directly
├── ros_planner_service.py    # service-triggered node — run on a ROS 2 service call
├── inference.py              # CheckpointInference single-step API
├── planner_meta.json         # planner structure: groups, workspaces, checkpoints
├── requirements.txt          # Python dependencies
├── easytrainer_runtime/      # SimpleAgent / SimpleEnv / image_parser / planner_engine / interpolation_node
├── lerobot/                  # vendored lerobot (do not pip-install a different one)
└── checkpoints/
    └── <checkpoint_id>/      # one folder per checkpoint the planner uses
        ├── config.json, model.safetensors, policy_*processor.json, ...
        └── export_meta.json
```

## Requirements

- **ROS 2** (Humble or newer) installed and sourced — `rclpy`, `std_srvs`,
  `sensor_msgs`, `trajectory_msgs` must import.
- **Python 3.10+** with the packages in `requirements.txt`:
  ```bash
  pip install -r requirements.txt
  ```
- A **CUDA GPU** is strongly recommended for checkpoint blocks (use
  `--device cpu` to force CPU — inference will be slow).
- The robot drivers and camera drivers must already be running and publishing
  on the same ROS 2 topics the planner was built with (see `planner_meta.json`
  → `workspaces.<id>.assembly.robots` and `.sensors`).

## 1. Run the planner directly

```bash
# Run the planner once:
python3 run_planner.py

# Repeat it 5 times:
python3 run_planner.py --repeat 5

# Loop forever until Ctrl-C:
python3 run_planner.py --repeat 0

# Compute checkpoint actions but don't publish to the robots (sanity check):
python3 run_planner.py --dry-run

# Force CPU inference:
python3 run_planner.py --device cpu
```

Groups run in parallel threads; blocks run sequentially within a group.
Ctrl-C signals a clean stop.

## 2. Run the planner on a ROS 2 service call

```bash
# Bring the node up (one terminal):
python3 ros_planner_service.py

# From another terminal — trigger / abort a run:
ros2 service call /easytrainer_planner_service/start std_srvs/srv/Trigger
ros2 service call /easytrainer_planner_service/stop  std_srvs/srv/Trigger
```

The node loads every checkpoint once at startup and keeps them warm, so each
`start` call begins immediately. `--repeat` controls iterations per call;
`--node-name`, `--device`, `--dry-run` work the same as `run_planner.py`.

## Joint-command smoothing (interpolation node)

Checkpoint inference runs at ~10Hz, but most robot drivers want a higher-rate
joint-command stream (200Hz) to move smoothly without jitter.

If a robot was configured with **interpolation enabled** in EasyTrainer
(`workspace → robot → interpolation: true`), the exporter records that in the
bundle's `planner_meta.json`. At startup the runner spawns a
`JointInterpolationNode` for each such robot — it subscribes to
`/ec_robot_<id>/ec_joint_cmd`, linearly interpolates between successive goals,
and publishes to the robot's actual `write_topic` at `interpolation_hz`
(default 200Hz). `move_to`-style blocks bypass the smoother via
`/ec_robot_<id>/ec_joint_cmd_direct` since they already produce smooth
trajectories themselves.

If interpolation is disabled, `SimpleAgent` publishes goals straight to the
robot's `write_topic` — your robot's driver / controller is then responsible
for any smoothing.

## Block support

| Block            | Standalone support                                              |
|------------------|-----------------------------------------------------------------|
| `joint_position` | ✅ interpolated move over `duration`                            |
| `checkpoint`     | ✅ closed-loop inference; `until_done` / `move_homepose` honored |
| `timesleep`      | ✅                                                              |
| `sync`           | ✅ cross-group barrier                                          |
| `query_pose`     | ✅ joint-position mode · ❌ end-effector mode (needs IK solver)  |

## Limitations vs in-container planner execution

- Robot `write_type` must be `topic`. Service / action-goal robots are skipped
  at startup; a block that needs one fails with a clear error.
- `query_pose` end-effector mode needs the Pinocchio IK solver, which is not
  bundled. Re-author the block in joint-position mode, or run the planner
  inside EasyTrainer.
- Checkpoint blocks require `action_key='qaction'` and
  `vision_backbone='resnet18'` — the exporter rejects other checkpoints up
  front.
- `move_homepose` moves robots to the workspace home pose **before** the
  checkpoint runs (the in-container per-episode re-homing loop is not
  replicated).
- No socketio progress broadcasts, no OOD scoring, no Grad-CAM, no OTI-RL.

If a planner needs any of the unsupported features, run it inside EasyTrainer
instead.
