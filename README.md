## Easy Trainer

로봇 티칭부터 AI 학습, 추론까지 하나의 웹 UI에서 완결하는 로봇 모방학습 플랫폼입니다.

---

### 시스템 요구사항

| 항목 | 최소 | 권장 |
|------|------|------|
| OS | Ubuntu 22.04 LTS | Ubuntu 22.04/24.04 |
| 저장 공간 | 20GB | 50GB+ |
| RAM | 8GB | 16GB+ |
| GPU | - | NVIDIA (VRAM 8GB+) |

### 사전 준비

deb 패키지 설치 시 대부분의 시스템 의존성은 **자동으로 설치**됩니다.
사전에 준비해야 하는 것은 **Docker**뿐입니다.

```bash
# Docker & Docker Compose v2 설치
sudo apt install -y curl
sudo install -m 0755 -d /etc/apt/keyrings
curl -fsSL https://download.docker.com/linux/ubuntu/gpg | \
  sudo gpg --dearmor -o /etc/apt/keyrings/docker.gpg
echo \
  "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.gpg] \
  https://download.docker.com/linux/ubuntu $(. /etc/os-release && echo "$UBUNTU_CODENAME") stable" | \
  sudo tee /etc/apt/sources.list.d/docker.list > /dev/null
sudo apt-get update
sudo apt-get install -y docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin
sudo usermod -aG docker "$USER" && newgrp docker
```

### GPU 사용 시 (선택)

```bash
# NVIDIA Container Toolkit
sudo wget -qO /etc/apt/keyrings/nvidia-container-toolkit.asc https://nvidia.github.io/libnvidia-container/gpgkey
echo "deb [signed-by=/etc/apt/keyrings/nvidia-container-toolkit.asc] \
https://nvidia.github.io/libnvidia-container/stable/deb/amd64 /" | \
  sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list
sudo apt-get update
sudo apt-get install -y nvidia-container-toolkit
sudo nvidia-ctk runtime configure --runtime=docker
sudo systemctl restart docker
```

### GPU 컨테이너 실행 시 libnvidia-egl-wayland 오류 해결

`libnvidia-egl-wayland.so.1.1.x: no such file or directory` 에러 발생 시:

```bash
# 호스트에 설치된 실제 버전 확인
ls /usr/lib/x86_64-linux-gnu/libnvidia-egl-wayland.so.1.*

# 에러 메시지에 나온 버전으로 심볼릭 링크 생성 (예: 1.1.9를 찾는 경우)
ACTUAL=$(ls /usr/lib/x86_64-linux-gnu/libnvidia-egl-wayland.so.1.*.* | head -1)
sudo ln -sf "$ACTUAL" /usr/lib/x86_64-linux-gnu/libnvidia-egl-wayland.so.1.1.9
sudo systemctl restart docker
```

---

### 설치

```bash
# deb 패키지 설치
sudo apt install ./easytrainer_<version>_amd64.deb

# 런처 실행
easytrainer-launcher
```

### 제거

```bash
sudo dpkg -r easytrainer
# 데이터까지 완전 삭제:
bash scripts/clean_easytrainer.sh
```

---

### 프로젝트 구조

```
Easy-Collector/
├── backend/              Flask API + PyTorch (Docker)
│   ├── Dockerfile
│   ├── entrypoint.sh
│   ├── api/              REST API + SocketIO
│   ├── database/         Orator ORM + SQLite
│   ├── lerobot/          모방학습 라이브러리 (vendored)
│   ├── policies/         정책 구현
│   └── modules/          확장 모듈 설치 위치
├── frontend/             Quasar Vue 3 UI (Docker)
│   ├── Dockerfile
│   ├── entrypoint.sh
│   └── src/
├── ros2/                 ROS 2 Humble (Docker)
│   ├── Dockerfile
│   ├── entrypoint.sh
│   └── ros2_ws/src/      로봇/센서 모듈 설치 위치
├── training_server/      학습 서버 (선택)
├── release/
│   ├── build.sh          deb 패키지 빌드
│   └── ui/               런처 (PySide6)
├── docker-compose.yml    3-service (GPU)
├── docker-compose.cpu.yml 3-service (CPU)
└── home-next/            랜딩 페이지 (Next.js)
```

### Docker 서비스

| 서비스 | 포트 | 설명 |
|--------|------|------|
| frontend | 5173 | Quasar Vue 3 UI (dev server) |
| backend | 5000 | Flask API + SocketIO |
| ros2 | - | ROS 2 (host network, DDS) |

### 개발 명령어

```bash
# 전체 빌드 + 실행
docker compose build
docker compose up -d

# CPU 전용
docker compose -f docker-compose.cpu.yml up -d

# 개별 서비스
docker compose up -d backend
docker compose logs -f backend

# 로컬 변경사항을 실행 중인 컨테이너에 동기화 (재빌드 없이)
bash scripts/quick_apply.sh ./ /opt/easytrainer/project

# 중지
docker compose stop

# 완전 초기화
docker compose down --remove-orphans --volumes

# deb 빌드
bash release/build.sh

# 런처 UI 테스트 (로컬)
cd release/ui && python3 main.py
```

### 모듈 시스템

로봇/센서/확장 모듈은 GitHub에서 선택 설치됩니다.
- 모듈 저장소: https://github.com/Plan-99/Easy-Trainer-Modules
- 런처의 🧩 버튼으로 설치/제거/업데이트 가능

---

### 매뉴얼

https://vertic-ai.com/manual/html/index.html
