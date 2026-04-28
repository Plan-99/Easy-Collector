# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## 어느 폴더를 수정해야 하는지 판단하기

작업이 들어오면 **먼저 [FOLDERS.md](FOLDERS.md)를 참고해서 어느 폴더를 건드려야 하는지 결정한다.** 각 폴더(`backend/`, `frontend/`, `home-next/`, `isaaclab/`, `modules/`, `release/`, `ros2/`, `training_server/`)의 역할, 주요 하위 구조, "이런 요청이면 여기를 수정" 예시가 정리되어 있다. 폴더 책임이 바뀌면 FOLDERS.md도 함께 갱신할 것.

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
**코드 수정 후 반드시 실행하여 런타임에 반영할 것.**
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
