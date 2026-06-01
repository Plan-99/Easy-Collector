# FOLDERS.md

이 문서는 작업이 들어왔을 때 **어느 폴더를 수정해야 하는지 판단**하기 위한 디렉토리 가이드입니다.
각 폴더의 역할, 주요 하위 구조, 기술 스택, "이런 요청이면 여기를 본다" 예시를 정리합니다.

> 새 폴더가 생기거나 폴더의 책임이 크게 바뀌면 이 문서도 갱신할 것.

---

## 작업 → 폴더 빠른 매핑

| 요청 키워드 | 1차로 볼 폴더 |
|---|---|
| REST API, 엔드포인트, 데이터셋/정책/체크포인트 비즈니스 로직, 학습 트리거 | [backend/](backend/) |
| 운영자 UI, 화면, 실시간 차트, 텔레오퍼레이션 페이지 | [frontend/](frontend/) |
| 홈페이지, 결제(PortOne), Google 로그인, 모듈 마켓, 어드민 백오피스 | [home-next/](home-next/) |
| Isaac Sim, USD, 합성 데이터, 모션 플래닝(curobo), 시뮬 로봇 | [isaaclab/](isaaclab/) |
| 새 로봇/센서 드라이버 추가, vendor SDK 패키징, 모듈 배포 | [modules/](modules/) |
| 데스크톱 런처, 설치 프로그램(.deb), PyQt UI, 디바이스 인증 화면 | [release/](release/) |
| ROS 2 노드/서비스, SDK 컨트롤러, IK, 토픽 브릿지 | [ros2/](ros2/) |
| 원격 학습 서버(별도 Docker, 5100 포트) | [training_server/](training_server/) |
| **"왜 지금 코드가 이렇게 생겼나"** — 굳어진 설계 결정의 단일 정보원 | [docs/](docs/) |

---

## backend/ — Flask-SocketIO API 허브 (Python)

**역할:** 로봇 제어, 센서, 데이터셋, 정책, 학습, ROS 2 노드 라이프사이클을 묶는 메인 백엔드. 5000 포트.

**주요 하위:**
- [backend/api/app.py](backend/api/app.py) — Flask-SocketIO 진입점. 라우트 블루프린트 등록 + ROS 2 노드 매니저.
- [backend/api/routes/](backend/api/routes/) — robot, sensor, dataset, policy, task, checkpoint, teleoperator, assembly, planner, vla, leader_robot 등 REST 블루프린트.
- [backend/api/process_manager.py](backend/api/process_manager.py) + [backend/api/process/](backend/api/process/) — record_episode, augment_dataset, merge_dataset 등 장시간 서브프로세스.
- [backend/database/](backend/database/) — Peewee ORM (SQLite). `models/` (테이블 생성/마이그레이션은 `models.create_tables()` 또는 `python -m backend.database.migrate`). DB는 `${EASYTRAINER_DATA_DIR}/database/main.db`.
- [backend/policies/](backend/policies/) — ACT, Diffusion, PI0, VLAsEn 정책 구현.
- [backend/lerobot/](backend/lerobot/) — LeRobot 라이브러리 vendored 사본 (외부 의존이 아님).
- [backend/env/](backend/env/) — 로봇 환경 헬퍼 (현재 `dxl_controller.py`, Dynamixel). 벤더별 로봇 드라이버는 `modules/robots/` 와 `ros2/` 에 있음.
- [backend/scripts/train_fiper.py](backend/scripts/train_fiper.py) — Hydra 기반 학습 진입점.
- [backend/sim/](backend/sim/), [backend/bridge/](backend/bridge/), [backend/fiper/](backend/fiper/), [backend/tools/](backend/tools/), [backend/utils/](backend/utils/).

**스택:** Python, Flask-SocketIO, Peewee ORM, SQLite, PyTorch (lerobot 경유).

**이런 요청이면 여기를 수정:**
- "새 REST 엔드포인트 추가" → `api/routes/`에 블루프린트 추가 + `api/app.py`에 등록.
- "데이터셋 스키마/컬럼 추가" → `database/models/` (수정 후 `create_tables()` / `python -m backend.database.migrate`로 반영).
- "학습 도중 로그/체크포인트 처리 변경" → `api/process/`, `policies/`, `scripts/train_fiper.py`.
- "기존 DB 모델 수정/조회 변경" → `database/models/` (Peewee).

**주의:** 코드 수정 후 `bash scripts/quick_apply.sh ./ /opt/easytrainer/project` 필수.

---

## frontend/ — Vue 3 / Quasar 운영자 UI

**역할:** 컨테이너 안에서 도는 운영자 대시보드. 실시간 텔레메트리, 모터 제어, 데이터셋 녹화, 학습 모니터, 정책 실행.

**주요 하위:**
- [frontend/src/pages/v2/](frontend/src/pages/v2/) — IndexPage, SensorPage, RobotPage, AssemblePage, WorkspacePage, TrainPage, DatasetPage, PlannerPage.
- [frontend/src/router/routes.js](frontend/src/router/routes.js) — 해시 모드 라우팅.
- [frontend/src/stores/](frontend/src/stores/) — Pinia (`processStore.js` 등).
- [frontend/src/components/](frontend/src/components/) — 재사용 컴포넌트.
- [frontend/quasar.config.js](frontend/quasar.config.js), [frontend/eslint.config.js](frontend/eslint.config.js).

**스택:** Vue 3 + Quasar 2.16 + Vite, Pinia, Axios (→ `http://localhost:5000/api`), Socket.IO, RosLib.

**스타일:** Prettier (세미콜론 X, 작은따옴표, 100 char), ESLint (Quasar recommended + Vue essential). **ESLint 규칙 반드시 준수.**

**이런 요청이면 여기를 수정:**
- "운영자 대시보드 화면/페이지 변경" → `src/pages/v2/`.
- "실시간 차트, 토픽 구독 추가" → 컴포넌트 + `socket.io` / `roslib` 연결.
- "전역 상태 변경" → `src/stores/`.

**주의:** 결제·로그인·랜딩 화면은 여기가 아니라 `home-next/`임.

---

## home-next/ — 외부 홈페이지 + 어드민 (Next.js 16, React 19)

**역할:** 제품 랜딩, Google OAuth **device-flow** 인증, 모듈 마켓, **PortOne v2** 결제, 사용자/디바이스/엔타이틀먼트 관리, 어드민 백오피스.

**주요 하위:**
- [home-next/src/app/](home-next/src/app/) — App Router 페이지: `/`, `/auth/signin`, `/auth/device`, `/onboarding`, `/checkout/[moduleId]`, `/dashboard`, `/admin/*`.
- [home-next/src/app/api/](home-next/src/app/api/) — `device-auth`, `devices`, `entitlements`, `me`, `modules`, `onboarding`, `payments`, `admin/*`.
- [home-next/src/lib/](home-next/src/lib/) — `launcher-auth.ts`, `payments.ts`, `portone.ts`, `legal.ts`, Prisma 클라이언트.
- [home-next/prisma/schema.prisma](home-next/prisma/schema.prisma) + [home-next/prisma/migrations/](home-next/prisma/migrations/).
- [home-next/src/components/](home-next/src/components/) — Footer, LegalModal 등.
- [home-next/CLAUDE.md](home-next/CLAUDE.md) — 이 폴더 전용 가이드 (있으면 먼저 참고).

**스택:** Next.js 16, React 19, NextAuth v5, Prisma + Postgres(Neon), PortOne v2, Tailwind v4, TypeScript.

**이런 요청이면 여기를 수정:**
- "홈페이지/마케팅 화면" → `src/app/page.tsx`, `src/app/(landing)`.
- "어드민 새 탭/사용자·결제·디바이스 관리" → `src/app/admin/*` + `src/app/api/admin/*`.
- "결제 흐름, 환불, 빌링키" → `src/lib/payments.ts`, `src/lib/portone.ts`, `src/app/api/payments/*`.
- "런처 ↔ 서버 디바이스 인증 토큰" → `src/lib/launcher-auth.ts`, `src/app/api/device-auth/*`.
- "DB 스키마 변경" → `prisma/schema.prisma` + 새 migration.

**주의:** 운영자 UI(`frontend/`)와 별개임. 결제·인증·마켓 관련 변경은 절대 `frontend/`에 넣지 말 것.

---

## isaaclab/ — Isaac Sim 기반 시뮬레이션

**역할:** NVIDIA Isaac Lab 기반 GPU 시뮬레이션, 합성 데이터 생성, 모션 플래닝, 커스텀 USD 로봇 모델.

**주요 하위:**
- [isaaclab/source/](isaaclab/source/) — Isaac Lab 코어 (물리, 로봇, 센서 시뮬).
- [isaaclab/apps/](isaaclab/apps/) — `.kit` 앱 설정 (headless, 렌더링).
- [isaaclab/ros_pkgs/](isaaclab/ros_pkgs/) — Sim ↔ ROS 2 브릿지 패키지.
- [isaaclab/curobo/](isaaclab/curobo/) — CuRobo 모션 플래닝.
- [isaaclab/isaac_control_core/](isaaclab/isaac_control_core/) — 제어 미들웨어.
- [isaaclab/custom_usds/](isaaclab/custom_usds/) — 커스텀 로봇 USD.
- [isaaclab/scripts/](isaaclab/scripts/), [isaaclab/tools/](isaaclab/tools/), [isaaclab/moveit/](isaaclab/moveit/), [isaaclab/docker/](isaaclab/docker/).

**스택:** Python, Isaac Lab, USD, ROS 2, CuRobo.

**이런 요청이면 여기를 수정:**
- "시뮬에서 새 로봇/그리퍼 추가" → `custom_usds/` + `source/`에 등록.
- "Sim에서 ROS 토픽으로 상태 publish" → `ros_pkgs/`.
- "모션 플래너 파라미터 튜닝" → `curobo/`.

**주의:** 실제 하드웨어 코드는 `modules/robots/` 또는 `ros2/`임. 시뮬 전용임을 잊지 말 것.

---

## modules/ — 하드웨어 드라이버 마켓플레이스

**역할:** 로봇/센서/시뮬/확장 모듈을 카테고리별로 모은 곳. CI(`modules-release.yml`)가 변경분만 감지하여 `Plan-99/Easy-Trainer-Modules` repo에 tar.gz를 릴리즈. 사용자는 런처에서 다운로드해서 설치.

**구조:**
```
modules/
├── robots/      # 로봇 드라이버 (kinova, unitree, jaka, piper, techman, fairino, robotiq, onrobot, rbpodo …)
│   └── <name>/
│       ├── module.json
│       ├── ros2/   → 설치 시 project/ros2/ros2_ws/src/<name>/
│       └── sdk/    → 설치 시 project/ros2/robot_sdk/<name>/
├── sensors/     # 센서 (webcam_publisher 등) → project/ros2/ros2_ws/src/<name>/
├── sim/         # 시뮬 월드/설정
└── extensions/  # 확장 (test_arm 등) → project/backend/extensions/<name>/
```

**스택:** Python, ROS 2 (CMake/setup.py), vendor SDK (C++/Python).

**이런 요청이면 여기를 수정:**
- "새 로봇 팔 드라이버 추가" → `robots/<vendor>/{ros2,sdk}/` + `module.json`.
- "센서(라이다, 카메라) 드라이버 추가" → `sensors/<name>/ros2/`.
- "기존 모듈 버그 수정" → 해당 모듈 코드 + `module.json`의 `version` **반드시 올림**.

**필수 배포 절차:** `module.json` 버전 ↑ → `module_up` 또는 `main`에 push → CI 확인 → 배포된 tar.gz 검증 (자세한 절차/검증 명령어는 [CLAUDE.md](CLAUDE.md) "Modules" 섹션 참고).

---

## release/ — 데스크톱 런처 & 패키징

**역할:** 최종 사용자에게 배포되는 PyQt 런처 앱과 시스템 서비스, DEB 패키징.

**주요 하위:**
- [release/ui/launcher.py](release/ui/launcher.py) — PyQt 메인 런처.
- [release/ui/main.py](release/ui/main.py) — 진입점.
- [release/ui/modules.py](release/ui/modules.py) — 모듈 다운로드/설치 UI.
- [release/ui/device_auth.py](release/ui/device_auth.py) — Bearer 토큰 기반 디바이스 인증 (홈페이지의 `/api/device-auth/*`와 연동).
- [release/ui/service.py](release/ui/service.py) — systemd/서비스 관리.
- [release/build.sh](release/build.sh) — DEB 빌드 스크립트.
- `easytrainer_<ver>_amd64.deb` — 빌드 결과물.

**스택:** Python, PyQt, Bash, DEB.

**이런 요청이면 여기를 수정:**
- "런처 UI 변경, 새 탭/버튼" → `release/ui/`.
- "디바이스 인증 흐름 수정" → `release/ui/device_auth.py` + (서버 측은 `home-next/src/app/api/device-auth/`).
- "설치/업데이트 절차 변경" → `release/ui/modules.py`, `release/build.sh`.

**주의:** `license_validator.py`는 삭제됨. 라이선스 검증이 아니라 Bearer 토큰 기반임.

---

## ros2/ — ROS 2 워크스페이스 & SDK 브릿지

**역할:** 백엔드와 실제 로봇 하드웨어 사이의 ROS 2 미들웨어. 토픽/서비스 핸들러, vendor SDK 어댑터, IK 등.

**주요 하위:**
- [ros2/ros2_bridge/](ros2/ros2_bridge/) — 메인 브릿지. `sdk_controllers/`, `services/`, `proto/`, `configs/`.
- [ros2/robot_sdk/](ros2/robot_sdk/) — SDK 래퍼. (모듈 설치 시 `modules/robots/<x>/sdk/`가 여기에 들어감)
- [ros2/ros2_ws/src/](ros2/ros2_ws/src/) — 표준 ROS 2 패키지 트리. (모듈 설치 시 `modules/robots/<x>/ros2/`, `modules/sensors/<x>/ros2/`가 여기에 들어감)
- [ros2/start_ros2_services.sh](ros2/start_ros2_services.sh) — ROS 2 서비스 시작 스크립트.
- [ros2/Dockerfile](ros2/Dockerfile), [ros2/Dockerfile.ros2](ros2/Dockerfile.ros2).

**스택:** Python (rclpy), ROS 2, Protocol Buffers/gRPC.

**이런 요청이면 여기를 수정:**
- "새 ROS 2 서비스 노드 추가" → `ros2_bridge/services/`.
- "vendor SDK 어댑터 추가/수정" → `ros2_bridge/sdk_controllers/`.
- "IK, 텔레메트리 스트림 변경" → `ros2_bridge/services/` + `configs/`.

**주의:** 모듈로 배포되는 코드(`modules/robots/<x>/ros2/`, `modules/sensors/<x>/ros2/`)는 여기 직접 두지 않고 `modules/`에 둘 것. `ros2_ws/src/`에 직접 둔 것은 모듈 시스템 밖의 코어 패키지뿐.

---

## training_server/ — 분리된 학습 서버 (참고)

> 사용자가 처음 나열한 7개 폴더에는 없지만, `backend/`에서 분리된 별도 Docker 서비스라 함께 기록.

**역할:** 5100 포트에서 도는 독립 Docker 학습 서버. 데이터셋 업로드 → 정책 학습 → 진행률 스트리밍.

**주요 하위:**
- [training_server/app.py](training_server/app.py) — Flask API.
- [training_server/train_worker.py](training_server/train_worker.py) — 학습 워커.
- [training_server/lerobot/](training_server/lerobot/) — `setup_deps.sh`가 `backend/lerobot/`에서 복사.
- [training_server/policies/](training_server/policies/), [training_server/Dockerfile](training_server/Dockerfile), [training_server/docker-compose.yml](training_server/docker-compose.yml).

**스택:** Python, Flask, LeRobot, PyTorch, Docker.

**이런 요청이면 여기를 수정:**
- "학습 워커 로직, 새 손실 함수" → `train_worker.py`.
- "원격 학습 API/업로드 한도" → `app.py`, `docker-compose.yml`.

**주의:** `lerobot/` 동기화는 `setup_deps.sh`로 — 직접 편집하면 다음 셋업에서 덮어써짐. 원본은 `backend/lerobot/`.
