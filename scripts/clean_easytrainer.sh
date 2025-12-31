#!/usr/bin/env bash
set -euo pipefail

INSTALL_ROOT="/opt/easytrainer"
PROJECT_DIR="$INSTALL_ROOT/project"
USER_CONFIG_DIR="${XDG_DATA_HOME:-$HOME/.local/share}/EasyTrainer"

# Remove only within known roots to avoid wiping unrelated paths.
safe_remove_dir() {
  local target=$1
  local label=$2
  local allowed_root=$3

  if [[ -z "$target" ]]; then
    echo "[clean] Skipping ${label}: empty path detected."
    return
  fi

  if [[ "$target" == "/" || "$target" == "$HOME" ]]; then
    echo "[clean] Refusing to remove ${label}: unsafe target '$target'."
    return
  fi

  if [[ "$target" != "$allowed_root" && "$target" != "$allowed_root"/* ]]; then
    echo "[clean] Refusing to remove ${label}: target '$target' not under expected root '$allowed_root'."
    return
  fi

  if [[ ! -e "$target" ]]; then
    echo "[clean] ${label} not found at $target; skipping."
    return
  fi

  echo "[clean] Removing ${label} at $target ..."
  if ! rm -rf "$target" 2>/dev/null; then
    echo "[clean] Permission denied; retrying with sudo ..."
    sudo rm -rf "$target" || true
  fi
}

if [[ "$PROJECT_DIR" == "$INSTALL_ROOT"/project && -f "$PROJECT_DIR/docker-compose.yml" ]]; then
  echo "[clean] docker compose down ..."
  (cd "$PROJECT_DIR" && docker compose down --remove-orphans --volumes) || true
elif [[ "$PROJECT_DIR" != "$INSTALL_ROOT"/project ]]; then
  echo "[clean] Refusing to run docker compose: unexpected PROJECT_DIR '$PROJECT_DIR'."
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
safe_remove_dir "$INSTALL_ROOT" "easytrainer install root" "$INSTALL_ROOT"

echo "[clean] Removing user config $USER_CONFIG_DIR ..."
safe_remove_dir "$USER_CONFIG_DIR" "user config" "$USER_CONFIG_DIR"

echo "[clean] Pruning dangling Docker volumes/builder cache ..."
docker volume prune -f || true
docker builder prune -f || true

echo "[clean] Done."
