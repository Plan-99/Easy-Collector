#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$ROOT_DIR"

if [ -n "${RELEASE_VERSION:-}" ]; then
  echo "$RELEASE_VERSION" > "$ROOT_DIR/VERSION"
fi

if [ -f "$ROOT_DIR/dist/EasyLauncher" ]; then
  export USE_PYINSTALLER=1
  export LAUNCHER_BIN_NAME=EasyLauncher
  export LAUNCHER_BIN_SRC="$ROOT_DIR/dist/EasyLauncher"
fi

bash "$ROOT_DIR/release/build.sh"

ARCH="${ARCH:-amd64}"
if [ -f "$ROOT_DIR/VERSION" ]; then
  VERSION_VAL="$(tr -d ' \t\r\n' < "$ROOT_DIR/VERSION")"
else
  VERSION_VAL=""
fi

EXPECTED_DEB="$ROOT_DIR/release/easytrainer_${VERSION_VAL}_${ARCH}.deb"
if [ -n "$VERSION_VAL" ] && [ -f "$EXPECTED_DEB" ]; then
  echo "[DEB] Built: $EXPECTED_DEB"
else
  echo "[DEB][WARN] Expected deb not found at $EXPECTED_DEB"
  find "$ROOT_DIR/release" -maxdepth 1 -type f -name "*.deb" -print
fi
