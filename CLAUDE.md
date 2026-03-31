# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

EasyTrainer is a robotics training and data collection platform for robot teleoperation, imitation learning, and policy training. It uses a Python/Flask backend with ROS 2 integration and a Vue 3/Quasar frontend, all running inside Docker containers with NVIDIA GPU support.

## Development Commands

### Docker (primary development environment)
```bash
docker compose up -d service          # Start services (GPU)
docker compose -f docker-compose.cpu.yml up -d service  # CPU-only
docker compose build                  # Rebuild container
docker logs -f easy_collector_service # View logs
```

### Frontend (inside container or locally in src/ui/)
```bash
cd src/ui
npm run dev       # Quasar dev server with hot reload
npm run build     # Production build
npm run lint      # ESLint check
npm run format    # Prettier format
```

### Quick Sync (apply local changes to running container without rebuild)
```bash
bash scripts/quick_apply.sh ./ /opt/easytrainer/project
```

### Release
Tag with semver (e.g., `git tag 3.2.1`) to trigger GitHub Actions CI/CD which runs PyArmor obfuscation + PyInstaller + DEB packaging.

## Architecture

### Backend (`src/backend/`)
- **`api/app.py`** — Flask-SocketIO app on port 5000. Registers 11+ route blueprints and manages ROS 2 node lifecycle.
- **`api/routes/`** — REST API endpoints (robot, sensor, dataset, policy, task, checkpoint, teleoperator, assembly, planner, vla, leader_robot). Each is a Flask Blueprint.
- **`api/process_manager.py`** — Manages subprocess lifecycle for long-running operations.
- **`api/process/`** — Async subprocess scripts (record_episode, augment_dataset, merge_dataset, read_hdf5, test_vla).
- **`database/`** — Orator ORM with SQLite. DB stored at `${EASYTRAINER_DATA_DIR}/database/main.db` (default `/opt/easytrainer/database/main.db`). Models in `models/`, migrations in `migrations/`.
- **`policies/`** — Policy implementations and utilities. Supports ACT, Diffusion, PI0, VLAsEn.
- **`lerobot/`** — Integrated LeRobot library for imitation learning policies, datasets, and configs.
- **`scripts/train.py`** — Main training entry point.
- **`env/`** — Robot environment abstractions and agent implementations (Dynamixel, Unitree, Piper, Jaka).

### Frontend (`src/ui/`)
- **Framework:** Vue 3 + Quasar 2.16 + Vite, hash-mode routing
- **State:** Pinia stores (e.g., `processStore.js`)
- **API communication:** Axios to `http://localhost:5000/api`, Socket.IO for real-time, RosLib for ROS bridge
- **Pages:** `src/ui/src/pages/v2/` — IndexPage, SensorPage, RobotPage, AssemblePage, WorkspacePage, TrainPage, DatasetPage, PlannerPage
- **Routes:** defined in `src/ui/src/router/routes.js`

### Service Orchestration (`start_services.sh`)
Entrypoint script (~700 lines) that handles memory detection, environment setup, config persistence, and starts backend API + frontend + ROS 2 services + streaming.

### Key Environment Variables
- `EASYTRAINER_DATA_DIR` — Persistent storage root (default `/opt/easytrainer`)
- `ROS_DOMAIN_ID` — ROS 2 domain isolation
- `EC_*` flags — Feature toggles (debug, minimal API, no frontend, etc.)

## Code Style

### Frontend
- Prettier: no semicolons, single quotes, 100 char print width
- ESLint: Quasar recommended + Vue essential rules

### Backend
- Python with Orator ORM for database operations
- Commit messages are typically in Korean

## Key Conventions
- Container runs as privileged with host networking for device access
- Persistent data (DB, config, logs) lives on host at `/opt/easytrainer/`
- Docker volumes cache `node_modules` to avoid reinstalls
- `src/` is bind-mounted into the container for live editing
- The `lerobot/` directory under backend is a vendored copy, not an external dependency

## MD 파일
- 코드가 바뀔 때마다 /home/hjhj/.claude/projects/-home-hjhj-EasyTrainer-v2-3-1/memory 폴더에서 적절한 md 파일을 보고, 또는 만들어서 적절히 수정해줘.
