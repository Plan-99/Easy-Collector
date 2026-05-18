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
├── Dockerfile                # ROS 2 Humble + deps for the test container
├── docker-compose.yml        # convenience compose file (GPU + host network)
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

## Run inside the bundled Docker container (recommended)

The bundle ships a `Dockerfile` + `docker-compose.yml` so you can run the
planner in a fresh, isolated environment without polluting the host. Host
needs Docker, Docker Compose, and (for GPU inference) the
`nvidia-container-toolkit`.

```bash
docker compose build                  # build the image once
docker compose run --rm test          # open an interactive shell inside it
# inside the container:
python3 run_planner.py --help
python3 run_planner.py --dry-run      # safe first check (no robot motion)
```

The compose file uses `network_mode: host` so the container sees the same
ROS 2 graph as the rest of your system (same `ROS_DOMAIN_ID`). The bundle
folder is mounted at `/workspace`, so you can re-export the planner from
EasyTrainer, unzip over the same folder, and the next `docker compose run`
picks up the new code immediately — **no rebuild needed unless
`requirements.txt` changes**.

To run a one-shot command without dropping into a shell:

```bash
docker compose run --rm test python3 run_planner.py --dry-run
```

If you'd rather skip Docker, the next two sections cover running directly on
the host.

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
| `query_pose`     | ✅ joint-position mode · ✅ end-effector mode (bundled Pinocchio IK) |

## End-effector query_pose (Pinocchio IK)

`query_pose` blocks in **end_effector_position** mode resolve target EE poses
to joint angles using a Pinocchio + `pink` IK solver. The exporter bundles, for
every robot with IK enabled, its URDF + mesh package directory under
`urdfs/<robot_id>/` and records the `joints_to_lock` + `ee_definitions` in
`planner_meta.json`. At startup `SimpleAgent` constructs a `Common_ArmIK`
(``easytrainer_runtime/ik_solver.py``) per robot using those paths.

The bundled Dockerfile installs Pinocchio via the `robotpkg` apt repo (same as
the in-container EasyTrainer ros2 image); `pin-pink` and a QP backend are
installed via `requirements.txt`. If you skip Docker and install on the host,
make sure Pinocchio (`pip install pin` or apt `robotpkg-py310-pinocchio`) and
`pin-pink` are reachable from your Python environment.

## Limitations vs in-container planner execution

- Robot `write_type` must be `topic`. Service / action-goal robots are skipped
  at startup; a block that needs one fails with a clear error.
- Checkpoint blocks require `action_key='qaction'` and
  `vision_backbone='resnet18'` — the exporter rejects other checkpoints up
  front.
- `move_homepose` moves robots to the workspace home pose **before** the
  checkpoint runs (the in-container per-episode re-homing loop is not
  replicated).
- EE-mode `query_pose` supports single-EE robots only (the same constraint as
  the in-container planner).
- No socketio progress broadcasts, no OOD scoring, no Grad-CAM, no OTI-RL.

If a planner needs any of the unsupported features, run it inside EasyTrainer
instead.
