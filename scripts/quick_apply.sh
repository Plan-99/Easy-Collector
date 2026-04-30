#!/usr/bin/env bash
#
# Synchronize backend/ui sources and compose helpers from a development
# checkout into the active runtime project directory without rebuilding
# Docker images. Designed to be fast enough for "quick apply" workflows.

set -euo pipefail

usage() {
  cat <<'USAGE' >&2
Usage: quick_apply.sh <DEV_SRC_ROOT> <RUNTIME_PROJECT_ROOT>

Copies src/backend, src/ui, ros2_ws/src, docker-compose files, and helper
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
  local src="$DEV_SRC/$rel"
  local dst="$PROJECT_ROOT/$rel"
  if [ ! -d "$src" ]; then
    echo "[quick-apply] 경고: 디렉터리가 없어 건너뜀: $src" >&2
    return 0
  fi
  mkdir -p "$dst"
  if [ -n "$RSYNC_BIN" ]; then
    "$RSYNC_BIN" -a \
      --exclude='__pycache__/' \
      --exclude='node_modules/' \
      --exclude='database/*.db' \
      --exclude='/datasets/' \
      --exclude='.git/' \
      "$src/" "$dst/" || true
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

copy_tree "backend"
copy_tree "frontend"
copy_tree "ros2/ros2_bridge"
# In the repo `training_server/` lives at the root (so it can ship as its own
# CI tar.gz), but at runtime it must appear under <project>/backend/training_server/
# (matching where the launcher's installer downloads to).
copy_tree_to() {
  local rel_src="$1" rel_dst="$2"
  local src="$DEV_SRC/$rel_src"
  local dst="$PROJECT_ROOT/$rel_dst"
  if [ ! -d "$src" ]; then
    echo "[quick-apply] 경고: 디렉터리가 없어 건너뜀: $src" >&2
    return 0
  fi
  mkdir -p "$dst"
  if [ -n "$RSYNC_BIN" ]; then
    "$RSYNC_BIN" -a \
      --exclude='__pycache__/' \
      --exclude='.git/' \
      "$src/" "$dst/" || true
  else
    ( cd "$src" && tar -cf - . ) | ( cd "$dst" && tar -xf - )
  fi
  echo "[quick-apply] 디렉터리 동기화 완료: $rel_src → $rel_dst"
}
copy_tree_to "training_server" "backend/training_server"
# ros2/ros2_ws/src and ros2/robot_sdk are managed by the module installer — do not
# sync them wholesale. Exception: anything sitting in ros2/ros2_ws/src/ in the
# source tree is a bundled (non-module) ROS2 package shipped with the app
# (e.g. mujoco_world for sim tutorial). 자동 탐지하여 모두 동기화한다.
if [ -d "$DEV_SRC/ros2/ros2_ws/src" ]; then
  while IFS= read -r -d '' pkg_xml; do
    pkg_dir=$(dirname "$pkg_xml")
    pkg_name=$(basename "$pkg_dir")
    rel="ros2/ros2_ws/src/$pkg_name"
    copy_tree_to "$rel" "$rel"
  done < <(find "$DEV_SRC/ros2/ros2_ws/src" -mindepth 2 -maxdepth 2 -name package.xml -print0)
fi
# modules/, home-next/ are not part of the runtime project

copy_file "docker-compose.yml"
copy_file "docker-compose.cpu.yml"
copy_file "requirements.txt"
# modules/ is NOT synced — project/modules/ only holds installed module manifests (*.json)
# Source module definitions (robots/, sensors/, extensions/) are read-only and mounted separately
copy_file "backend/entrypoint.sh" 755
copy_file "ros2/start_ros2_services.sh" 755
copy_file "ros2/entrypoint.sh" 755
copy_file "scripts/quick_apply.sh" 755
copy_file "scripts/clean_easytrainer.sh" 755

sync_deb_ui
sync_deb_scripts

echo "[quick-apply] 동기화가 완료되었습니다."
