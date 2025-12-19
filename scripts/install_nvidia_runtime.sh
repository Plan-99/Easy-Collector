#!/usr/bin/env bash
#
# Install and configure the NVIDIA Container Toolkit so Docker can start
# GPU-enabled containers (required by Easy Trainer). Designed to be idempotent.

set -euo pipefail

if [[ "${EUID:-$(id -u)}" -ne 0 ]]; then
  exec sudo -E bash "$0" "$@"
fi

log() {
  echo "[nvidia-setup] $*" >&2
}

need_cmd() {
  if ! command -v "$1" >/dev/null 2>&1; then
    log "필요한 명령을 찾을 수 없습니다: $1"
    exit 1
  fi
}

need_cmd wget
need_cmd tee
need_cmd apt-get

ARCH="$(dpkg --print-architecture 2>/dev/null || echo amd64)"
KEYRING="/etc/apt/keyrings/nvidia-container-toolkit.asc"
LIST="/etc/apt/sources.list.d/nvidia-container-toolkit.list"
REPO_URL="https://nvidia.github.io/libnvidia-container"

log "APT keyring과 저장소를 구성합니다..."
install -m 0755 -d /etc/apt/keyrings
wget -qO "$KEYRING" "$REPO_URL/gpgkey"
echo "deb [signed-by=${KEYRING}] ${REPO_URL}/stable/deb/${ARCH} /" | tee "$LIST" >/dev/null

log "패키지 인덱스를 갱신합니다..."
apt-get update -y

log "nvidia-container-toolkit 패키지를 설치합니다..."
DEBIAN_FRONTEND=noninteractive apt-get install -y nvidia-container-toolkit

if command -v nvidia-ctk >/dev/null 2>&1; then
  log "Docker 런타임을 NVIDIA로 구성합니다..."
  nvidia-ctk runtime configure --runtime=docker
fi

if command -v systemctl >/dev/null 2>&1; then
  log "Docker 데몬을 재시작합니다..."
  systemctl restart docker
else
  log "systemctl이 없어 docker 재시작을 건너뜁니다. 필요 시 수동으로 재시작하세요."
fi

if docker info --format '{{json .Runtimes}}' 2>/dev/null | grep -qi nvidia; then
  log "NVIDIA Docker 런타임이 감지되었습니다."
else
  log "경고: docker info에서 NVIDIA 런타임을 감지하지 못했습니다."
fi

log "설치가 완료되었습니다."
