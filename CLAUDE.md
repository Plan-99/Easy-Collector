# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## 어느 폴더를 수정해야 하는지 판단하기

작업이 들어오면 **먼저 [FOLDERS.md](FOLDERS.md)를 참고해서 어느 폴더를 건드려야 하는지 결정한다.** 각 폴더(`backend/`, `frontend/`, `home-next/`, `isaaclab/`, `modules/`, `release/`, `ros2/`, `training_server/`)의 역할, 주요 하위 구조, "이런 요청이면 여기를 수정" 예시가 정리되어 있다. 폴더 책임이 바뀌면 FOLDERS.md도 함께 갱신할 것.

## 왜 지금 코드가 이렇게 생겼는지 알아야 하면

[docs/](docs/) — 굳어진 설계 결정이 모두 여기 있다. 특히 [docs/design-docs/](docs/design-docs/)는
"왜 이런 구조인가"의 단일 정보원이다. 작업이 도메인 결정(모듈 entitlement, 학습 분리, 카메라
처리 등)에 의존하면 먼저 design-docs를 훑을 것. 새 결정이 굳어지면 `docs/README.md`에 적힌
규칙대로 새 design-doc을 추가한다.

## Project Overview

EasyTrainer is a robotics training and data collection platform for robot teleoperation, imitation learning, and policy training. It uses a Python/Flask backend with ROS 2 integration and a Vue 3/Quasar frontend, all running inside Docker containers with NVIDIA GPU support.

## Development Commands

### Docker (primary development environment)
Compose services are `frontend`, `backend`, `ros2` (containers `easytrainer_frontend`, `easytrainer_backend`, `easytrainer_ros2`).
```bash
docker compose up -d                       # Start all services (GPU)
docker compose up -d backend               # Start a single service (frontend / backend / ros2)
docker compose -f docker-compose.cpu.yml up -d  # CPU-only
docker compose build                       # Rebuild images
docker logs -f easytrainer_backend         # View logs (or easytrainer_frontend / easytrainer_ros2)
```

### Frontend (inside container or locally in frontend/)
```bash
cd frontend
npm run dev       # Quasar dev server with hot reload
npm run build     # Production build
npm run lint      # ESLint check
npm run format    # Prettier format
```

### Quick Sync (apply local changes to running container without rebuild)
**코드 수정 후 반드시 실행하여 런타임에 반영할 것.**
```bash
bash scripts/quick_apply.sh ./ /opt/easytrainer/project
```

### Release
Tag with semver (e.g., `git tag 3.2.1`) to trigger GitHub Actions CI/CD which runs PyArmor obfuscation + PyInstaller + DEB packaging.

### Pre-commit (lint gate)
Catches broken JSON, Python syntax errors, missing module.json fields, and (on pre-push) ESLint + module.json version-bump. One-time setup on each dev machine:
```bash
pipx install pre-commit    # or: pip install --user pre-commit
pre-commit install
pre-commit install --hook-type pre-push
```
Manual sweep across the repo: `pre-commit run --all-files`. Hook definitions live in [.pre-commit-config.yaml](.pre-commit-config.yaml).

## Architecture

### Backend (`backend/`)
- **`api/app.py`** — Flask-SocketIO app on port 5000. Registers the route blueprints (currently 16) and manages ROS 2 node lifecycle.
- **`api/routes/`** — REST API endpoints (robot, robot_pose, sensor, dataset, policy, task, checkpoint, teleoperator, assembly, planner, vla, leader_robot, sim, remote_train, tutorial, module). Each is a Flask Blueprint.
- **`api/process_manager.py`** — Manages subprocess lifecycle for long-running operations.
- **`api/process/`** — Async subprocess scripts (record_episode, augment_dataset, merge_dataset, read_dataset, test_vla, etc.).
- **`database/`** — Peewee ORM with SQLite. DB stored at `${EASYTRAINER_DATA_DIR}/database/main.db` (default `/opt/easytrainer/database/main.db`). Models in `models/`; tables are created/migrated via `models.create_tables()` (run by the backend entrypoint, or `python -m backend.database.migrate`).
- **`policies/`** — Policy implementations and utilities. Supports ACT, Diffusion, PI0, VLAsEn.
- **`lerobot/`** — Integrated LeRobot library for imitation learning policies, datasets, and configs.
- **`scripts/train_fiper.py`** — Hydra-based training entry point (configs under `backend/fiper/configs`).
- **`env/`** — Robot environment helpers (currently `dxl_controller.py` for Dynamixel). Per-vendor robot drivers live in `modules/robots/` and `ros2/`.

### Frontend (`frontend/`)
- **디자인/구현 패턴은 [frontend/DESIGN.md](frontend/DESIGN.md)를 반드시 먼저 참고할 것.** 페이지·컴포넌트를 만들거나 수정하기 전에 색상 팔레트, 페이지 골격, 카드 그리드/폼/알림 패턴, 공용 컴포넌트 재사용 규칙이 정리되어 있다.
- **Framework:** Vue 3 + Quasar 2.16 + Vite, hash-mode routing
- **State:** Pinia stores (e.g., `processStore.js`)
- **API communication:** Axios to `http://localhost:5000/api`, Socket.IO for real-time, RosLib for ROS bridge
- **Pages:** `frontend/src/pages/v2/` — IndexPage, SensorPage, RobotPage, AssemblePage, WorkspacePage, TrainPage, DatasetPage, PlannerPage, TeleoperationPage
- **Routes:** defined in `frontend/src/router/routes.js`

### Service Orchestration
There is no single orchestrator script. Each Docker service ships its own entrypoint:
- **`backend/entrypoint.sh`** — runs DB migration (`create_tables`), installs module dependencies as a fallback (skipped when deps are baked into the image), then launches the Flask-SocketIO API.
- **`frontend/entrypoint.sh`** — starts the Quasar dev server.
- **`ros2/start_ros2_services.sh`** (via `ros2/entrypoint.sh`) — builds/sources the ROS 2 workspace and starts ROS 2 nodes/bridge and streaming.

### Key Environment Variables
- `EASYTRAINER_DATA_DIR` — Persistent storage root (default `/opt/easytrainer`)
- `ROS_DOMAIN_ID` — ROS 2 domain isolation
- `EC_*` flags — Feature toggles (debug, minimal API, no frontend, etc.)

## Code Style

### Frontend
- Prettier: no semicolons, single quotes, 100 char print width
- ESLint: Quasar recommended + Vue essential rules

### Backend
- Python with Peewee ORM for database operations
- Commit messages are typically in Korean

## Key Conventions
- Container runs as privileged with host networking for device access
- Persistent data (DB, config, logs) lives on host at `/opt/easytrainer/`
- Docker volumes cache `node_modules` to avoid reinstalls
- `backend/`, `frontend/`, and `ros2/` are bind-mounted into their containers for live editing
- The `lerobot/` directory under backend is a vendored copy, not an external dependency

## Modules

### 구조
```
modules/
├── robots/          # 로봇 드라이버 (ros2 pkg + sdk)
│   └── piper/
│       ├── module.json
│       ├── ros2/    → 설치 시 project/ros2/ros2_ws/src/piper/
│       └── sdk/     → 설치 시 project/ros2/robot_sdk/piper/
├── sensors/         # 센서 드라이버 (ros2 pkg)
│   └── webcam_publisher/
│       ├── module.json
│       └── ros2/    → 설치 시 project/ros2/ros2_ws/src/webcam_publisher/
└── extensions/      # 확장 모듈
    └── test_arm/
        ├── module.json
        └── ...      → 설치 시 project/backend/extensions/test_arm/
```

### 모듈 배포 규칙 (필수)
**모듈 관련 코드(module.json, ros2/, sdk/ 등)가 수정되면 반드시 다음 절차를 따른다:**
1. `module.json`의 `version`을 올린다 (같은 버전은 릴리즈 스킵됨)
2. `module_up` 또는 `main` 브랜치에 commit & push한다
3. CI(`modules-release.yml`)가 변경된 모듈만 감지하여 Plan-99/Easy-Trainer-Modules repo에 릴리즈를 생성한다
4. **배포된 tar.gz를 다운로드하여 module.json 내용이 올바른지 검증한다**
5. 검증 실패 시 버전을 다시 올려서 재배포한다

### 검증 명령어
```bash
# CI 상태 확인
gh run list -w "Package Modules" -L 5

# 배포된 tar.gz의 module.json 확인
python3 -c "
import urllib.request, json, tarfile, os
url = 'https://api.github.com/repos/Plan-99/Easy-Trainer-Modules/releases'
req = urllib.request.Request(url, headers={'Accept': 'application/vnd.github+json'})
with urllib.request.urlopen(req) as resp:
    for r in json.loads(resp.read().decode()):
        if 'robot_piper' in r['tag_name']:
            for a in r['assets']:
                urllib.request.urlretrieve(a['browser_download_url'], '/tmp/check.tar.gz')
                with tarfile.open('/tmp/check.tar.gz') as tar:
                    for m in tar.getmembers():
                        if 'module.json' in m.name:
                            print(json.dumps(json.loads(tar.extractfile(m).read()), indent=2))
                os.unlink('/tmp/check.tar.gz')
            break
"
```

<!-- ## MD 파일
- 코드가 바뀔 때마다 /home/hjhj/.claude/projects/-home-hjhj-EasyTrainer-v2-3-1/memory 폴더에서 적절한 md 파일을 보고, 또는 만들어서 적절히 수정해줘. -->
