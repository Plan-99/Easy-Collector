#!/usr/bin/env bash
set -euo pipefail

PROJECT_DIR="/opt/easytrainer/project"
USER_CONFIG_DIR="${XDG_DATA_HOME:-$HOME/.local/share}/EasyTrainer"

if [ -f "$PROJECT_DIR/docker-compose.yml" ]; then
  echo "[clean] docker compose down ..."
  (cd "$PROJECT_DIR" && docker compose down --remove-orphans --volumes) || true
fi

echo "[clean] Removing easy-collector docker artifacts ..."
docker rm -f easy_collector_service 2>/dev/null || true
docker image rm easy-collector:latest 2>/dev/null || true

echo "[clean] Removing easytrainer apt package (if installed)..."
if dpkg -s easytrainer >/dev/null 2>&1; then
  sudo apt remove --purge -y easytrainer
else
  echo "[clean] easytrainer package not found."
fi
sudo apt autoremove -y || true

echo "[clean] Removing /opt/easytrainer ..."
sudo rm -rf /opt/easytrainer || true

echo "[clean] Removing user config $USER_CONFIG_DIR ..."
if ! rm -rf "$USER_CONFIG_DIR" 2>/dev/null; then
  echo "[clean] Permission denied; retrying with sudo ..."
  sudo rm -rf "$USER_CONFIG_DIR" || true
fi

echo "[clean] Pruning dangling Docker volumes/builder cache ..."
docker volume prune -f || true
docker builder prune -f || true

echo "[clean] Done."
