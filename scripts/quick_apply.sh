#!/usr/bin/env bash
#
# Synchronize backend/ui sources and compose helpers from a development
# checkout into the active runtime project directory without rebuilding
# Docker images. Designed to be fast enough for "quick apply" workflows.

set -euo pipefail

usage() {
  cat <<'USAGE' >&2
Usage: quick_apply.sh <DEV_SRC_ROOT> <RUNTIME_PROJECT_ROOT>

Copies src/backend, src/ui, ros2/, docker-compose files, and helper
scripts from the development checkout into the runtime project directory used
by services.
This script avoids rebuilding Docker images; it simply updates files in-place.
USAGE
}

if [ "$#" -ne 2 ]; then
  usage
  exit 1
fi

resolve_path() {
  local input="$1"
  if [ -z "$input" ]; then
    return 1
  fi
  # Use Python for cross-platform realpath resolution.
  python3 -c 'import os, sys; print(os.path.realpath(sys.argv[1]))' "$input"
}

DEV_SRC="$(resolve_path "$1")"
PROJECT_ROOT="$(resolve_path "$2")"

if [ ! -d "$DEV_SRC" ]; then
  echo "[quick-apply] 개발 소스 경로가 존재하지 않습니다: $DEV_SRC" >&2
  exit 2
fi
if [ ! -d "$PROJECT_ROOT" ]; then
  echo "[quick-apply] 런타임 프로젝트 경로가 존재하지 않습니다: $PROJECT_ROOT" >&2
  exit 3
fi

RSYNC_BIN=""
if command -v rsync >/dev/null 2>&1; then
  RSYNC_BIN="$(command -v rsync)"
fi

copy_tree() {
  local rel="$1"
  local no_so_exclude="${2:-}"  # "no_so" 전달 시 *.so 제외 안 함
  local src="$DEV_SRC/$rel"
  local dst="$PROJECT_ROOT/$rel"
  if [ ! -d "$src" ]; then
    echo "[quick-apply] 경고: 디렉터리가 없어 건너뜀: $src" >&2
    return 0
  fi
  mkdir -p "$dst"
  if [ -n "$RSYNC_BIN" ]; then
    local -a rsync_args=(
      -rlptD --no-owner --no-group
      --exclude='__pycache__/'
      --exclude='node_modules/'
      --exclude='database/*.db'
      --exclude='/datasets/'
      --exclude='.git/'
    )
    if [ "$no_so_exclude" != "keep_so" ]; then
      rsync_args+=(--exclude='*.so' --exclude='*.so.*')
    fi
    "$RSYNC_BIN" "${rsync_args[@]}" "$src/" "$dst/"
  else
    # Portable fallback using tar over a pipe.
    (
      cd "$src"
      tar -cf - .
    ) | (
      cd "$dst"
      tar -xf -
    )
  fi
  echo "[quick-apply] 디렉터리 동기화 완료: $rel"
}

sync_deb_ui() {
  local src="$DEV_SRC/release/ui"
  if [ ! -d "$src" ]; then
    echo "[quick-apply] 경고: DEB UI 경로가 없어 건너뜀: $src" >&2
    return 0
  fi
  local install_root=""
  if [ -n "$PROJECT_ROOT" ]; then
    install_root="$(dirname "$PROJECT_ROOT")"
  fi
  local dst=""
  if [ -n "$install_root" ] && [ -d "$install_root/ui" ]; then
    dst="$install_root/ui"
  elif [ -d "/opt/easytrainer/ui" ]; then
    dst="/opt/easytrainer/ui"
  else
    return 0
  fi
  if ! mkdir -p "$dst"; then
    echo "[quick-apply][WARN] DEB UI 경로 생성 실패: $dst" >&2
    return 0
  fi
  if [ -n "$RSYNC_BIN" ]; then
    if "$RSYNC_BIN" -a \
      --exclude='__pycache__/' \
      --exclude='*.pyc' \
      "$src/" "$dst/"
    then
      echo "[quick-apply] DEB UI 동기화 완료: $dst"
    else
      echo "[quick-apply][WARN] DEB UI 동기화 실패: $dst" >&2
    fi
  else
    if (
      cd "$src"
      tar -cf - .
    ) | (
      cd "$dst"
      tar -xf -
    ); then
      echo "[quick-apply] DEB UI 동기화 완료: $dst"
    else
      echo "[quick-apply][WARN] DEB UI 동기화 실패: $dst" >&2
    fi
  fi
}

sync_deb_scripts() {
  local src="$DEV_SRC/scripts"
  if [ ! -d "$src" ]; then
    echo "[quick-apply] 경고: DEB scripts 경로가 없어 건너뜀: $src" >&2
    return 0
  fi
  local install_root=""
  if [ -n "$PROJECT_ROOT" ]; then
    install_root="$(dirname "$PROJECT_ROOT")"
  fi
  local dst=""
  if [ -n "$install_root" ]; then
    dst="$install_root/scripts"
  elif [ -d "/opt/easytrainer" ]; then
    dst="/opt/easytrainer/scripts"
  else
    return 0
  fi
  if ! mkdir -p "$dst"; then
    echo "[quick-apply][WARN] DEB scripts 경로 생성 실패: $dst" >&2
    return 0
  fi
  if [ -n "$RSYNC_BIN" ]; then
    if "$RSYNC_BIN" -a \
      --exclude='__pycache__/' \
      --exclude='*.pyc' \
      "$src/" "$dst/"
    then
      echo "[quick-apply] DEB scripts 동기화 완료: $dst"
    else
      echo "[quick-apply][WARN] DEB scripts 동기화 실패: $dst" >&2
    fi
  else
    if (
      cd "$src"
      tar -cf - .
    ) | (
      cd "$dst"
      tar -xf -
    ); then
      echo "[quick-apply] DEB scripts 동기화 완료: $dst"
    else
      echo "[quick-apply][WARN] DEB scripts 동기화 실패: $dst" >&2
    fi
  fi
}

copy_file() {
  local rel="$1"
  local mode="${2:-644}"
  local src="$DEV_SRC/$rel"
  local dst="$PROJECT_ROOT/$rel"
  if [ ! -f "$src" ]; then
    echo "[quick-apply] 경고: 파일이 없어 건너뜀: $src" >&2
    return 0
  fi
  mkdir -p "$(dirname "$dst")"
  install -m "$mode" "$src" "$dst"
  echo "[quick-apply] 파일 복사 완료: $rel"
}

copy_tree "src/backend"
copy_tree "src/ui"
copy_tree "ros2/ros2_bridge"
copy_tree "ros2/robot_sdk"
copy_tree "ros2/ros2_ws/src" "keep_so"

copy_file "docker-compose.yml"
copy_file "docker-compose.dev.yml"
copy_file "docker-compose.cpu.yml"
copy_file "docker-compose.gpu.yml"
copy_file "start_services.sh" 755
copy_file "ros2/start_ros2_services.sh" 755
copy_file "src/kill.sh" 755
copy_file "Dockerfile"
copy_file "Dockerfile.main"
copy_file "ros2/Dockerfile.ros2"
copy_file ".dockerignore"
copy_file "requirements.txt"
copy_file "requirements.min.txt"
copy_file "ros2/requirements.ros2.txt"
copy_file "scripts/quick_apply.sh" 755
copy_file "scripts/install_nvidia_runtime.sh" 755

sync_deb_ui
sync_deb_scripts

echo "[quick-apply] 동기화가 완료되었습니다."
