## Easy Trainer

- 기능
  1. 인스톨러로 초기 설치 → 컨테이너, ROS 2, 프론트엔드 의존성이 모두 포함된 환경을 그대로 재사용
  2. Ubuntu 배포용 `.deb` 자동 생성 → 현재 프로젝트 버전 그대로 설치 가능
  3. 런처에서 원본 프로젝트 경로 지정 후 “빠른 동기화(⇆)”로 `src/backend`, `src/ui`, compose 파일을 바로 적용
  4. 로컬 앱처럼 실행하면 켜지고 종료하면 꺼짐, 종료 시 `docker compose down --remove-orphans`로 컨테이너만 정리하고 볼륨은 유지

- 예정 기능
  1. 데이터 내보내기 기능, 삭제 기능
  2. 코드 정리
  3. 코드 업데이트 기능 → 전체 재설치 대신 필요한 코드 또는 암호화된 코드만 교체하여 적용
  4. 다양한 OS 배포 기능 (Windows/macOS)

---

### 초기 세팅

- 실행 환경 구성
  - UI & 배포 환경: 
    ```bash
    sudo apt-get install -y dpkg-dev rsync python3-venv
    python3 -m pip install --user PySide6
    ```
  - Docker & Docker Compose v2
    ```bash
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
  - NVIDIA Container Toolkit
    ```bash
    sudo wget -qO /etc/apt/keyrings/nvidia-container-toolkit.asc https://nvidia.github.io/libnvidia-container/gpgkey
    echo "deb [signed-by=/etc/apt/keyrings/nvidia-container-toolkit.asc] \
    https://nvidia.github.io/libnvidia-container/stable/deb/amd64 /" | \
      sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list

    sudo apt-get update
    sudo apt-get install -y nvidia-container-toolkit
    sudo nvidia-ctk runtime configure --runtime=docker
    sudo systemctl restart docker
    ```

---

### Easy Trainer 설치

- 설치/제거
  - 설치: deb 파일을 더블클릭해서 설치 (또는 우클릭 -> 속성 -> 프로그램 설치로 실행) 
  - 위치: `opt/easytrainer`에 ui, 전체 코드 생성됨.
  - 제거: `bash scripts/clean_easytrainer.sh` *주의 : DB 데이터, 학습데이터 삭제 됨.

- 배포 deb 생성
  - 기본: `bash release/build.sh`
  - 버전 지정: `VERSION=1.0.1 bash release/build.sh`
  - 결과물: `release/easytrainer_amd64.deb`

---

### 프로그램 가이드

- 인스톨러
  - 조건: 최소 20GB 이상의 공간 필요
  - 옵션: cpu, gpu 버전

- 기능
  1. **경로 표시줄**: 한 번 클릭하면 개발 프로젝트 폴더를 선택. 선택 값은 `/opt/easytrainer/config.json`에 저장되며 `EASYTRAINER_DATA_DIR`로 변경 가능.  
  2. **뒤로/앞으로 버튼**: WebView 히스토리 이동.  
  3. **빠른 동기화(⇆)**: `scripts/quick_apply.sh <원본> <런타임>`을 호출해 `src/backend`, `src/ui`, `docker-compose*.yml`, `start_services.sh`를 복사하고 WebView를 새로고침.
  4. **Log 버튼**: 런처/프론트/백 로그 창이 열리고, 실시간 tail 동기화를 유지 (로그는 기본 `/tmp/easytrainer/logs`에 저장) *런처와 ui는 pc에, 백엔드와 프론트엔드는 컨테이너에 저장
  5. **서비스 시작/종료**: 창을 닫으면 자동으로 `docker compose down --remove-orphans` 실행.

---

### 디버깅

- 터미널 명령어
  - 빌드: `docker compose build`
  - 실행: `docker compose up -d service`
  - 동기화: `bash scripts/quick_apply.sh ./ /opt/easytrainer/project`
  - 중지/정리: `docker compose down --remove-orphans` (완전 초기화 시 `--volumes` 추가)
  - 로그: `docker logs -f easy_collector_service`

---
