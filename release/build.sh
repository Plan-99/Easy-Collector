#!/usr/bin/env bash
set -euo pipefail

PKG=easytrainer
ARCH=${ARCH:-amd64}
INSTALL_ROOT=/opt/easytrainer
PAYLOAD_DIR=/usr/share/easytrainer-project

# Paths
RELEASE_DIR=$(cd "$(dirname "$0")" && pwd)
ROOT_DIR=$(cd "$RELEASE_DIR/.." && pwd)
VERSION_FILE="$ROOT_DIR/VERSION"
USE_PYINSTALLER=${USE_PYINSTALLER:-0}
LAUNCHER_BIN_NAME=${LAUNCHER_BIN_NAME:-EasyLauncher}
LAUNCHER_BIN_SRC=${LAUNCHER_BIN_SRC:-$ROOT_DIR/dist/$LAUNCHER_BIN_NAME}

# Version handling: control file always needs a Version, but filename can omit it
VERSION_ENV="${VERSION:-}"
if [ -z "$VERSION_ENV" ] && [ -f "$VERSION_FILE" ]; then
  VERSION_ENV="$(tr -d ' \t\r\n' <"$VERSION_FILE")"
fi
if [ -n "$VERSION_ENV" ]; then
  VERSION_VAL="$VERSION_ENV"
else
  VERSION_VAL="1.0.0"  # default for control metadata only
fi
STAGE="$RELEASE_DIR/${PKG}_${VERSION_VAL}_${ARCH}"
DEB_NAME="$STAGE.deb"

echo "[deb] Building $PKG ${VERSION_ENV:-no-version} ($ARCH)"
rm -rf "$STAGE" && mkdir -p "$STAGE/DEBIAN"

# Install layout
mkdir -p \
  "$STAGE/usr/bin" \
  "$STAGE/usr/share/applications" \
  "$STAGE${INSTALL_ROOT}" \
  "$STAGE${PAYLOAD_DIR}"

if [ "$USE_PYINSTALLER" = "1" ]; then
  echo "[deb] Copying launcher binary (${LAUNCHER_BIN_SRC} -> ${INSTALL_ROOT}/${LAUNCHER_BIN_NAME})..."
  if [ ! -f "$LAUNCHER_BIN_SRC" ]; then
    echo "[deb][ERROR] Launcher binary not found: $LAUNCHER_BIN_SRC" >&2
    exit 1
  fi
  cp "$LAUNCHER_BIN_SRC" "$STAGE${INSTALL_ROOT}/${LAUNCHER_BIN_NAME}"
  chmod 0755 "$STAGE${INSTALL_ROOT}/${LAUNCHER_BIN_NAME}"
else
  mkdir -p "$STAGE${INSTALL_ROOT}/ui"
  echo "[deb] Copying launcher UI (release/ui -> ${INSTALL_ROOT}/ui)..."
  rsync -a \
    --exclude='__pycache__/' \
    --exclude='*.pyc' \
    "$RELEASE_DIR/ui/" "$STAGE${INSTALL_ROOT}/ui/"
  required_ui_files=(
    "main.py"
    "launcher.py"
    "installer.py"
    "service.py"
    "app_context.py"
    "tools.py"
  )
  for f in "${required_ui_files[@]}"; do
    if [ ! -f "$STAGE${INSTALL_ROOT}/ui/$f" ]; then
      echo "[deb][ERROR] Missing UI file: $f" >&2
      exit 1
    fi
  done
fi
if [ -f "$ROOT_DIR/app_icon.png" ]; then
  cp "$ROOT_DIR/app_icon.png" "$STAGE${INSTALL_ROOT}/app_icon.png"
fi
# Keep version file alongside the embedded UI so runtime can read it
if [ -f "$VERSION_FILE" ]; then
  cp "$VERSION_FILE" "$STAGE${INSTALL_ROOT}/VERSION"
fi

PY_VER=$(python3 - <<'PY'
import sys
print(f"{sys.version_info.major}.{sys.version_info.minor}")
PY
)
PY_MAJOR=${PY_VER%%.*}
PY_MINOR=${PY_VER#*.}
PY_NEXT_MINOR=$((PY_MINOR + 1))
echo "[deb] Using host Python $PY_VER"
if [ "$USE_PYINSTALLER" != "1" ]; then
  if ! python3 -c 'import ensurepip' 2>/dev/null; then
    echo "[deb][ERROR] python3-venv (ensurepip) is not available on this build machine."
    echo "Install it and retry: sudo apt-get update && sudo apt-get install -y python3-venv"
    exit 1
  fi
  echo "[deb] Creating embedded venv with Qt..."
  VENV_DIR="$STAGE${INSTALL_ROOT}/venv"
  python3 -m venv "$VENV_DIR"

  # Keep default pip inside venv to avoid network upgrades
  echo "[deb] Copying existing PySide6 install into venv..."
  DST_SITE="$VENV_DIR/lib/python${PY_VER}/site-packages"
  mkdir -p "$DST_SITE"

  # --- [FIXED SECTION START] PySide6 & Shiboken6 Detection ---
  # 1. Find PySide6 Path dynamically
  PYSIDE_SRC=$(python3 -c "import os, PySide6; print(os.path.dirname(PySide6.__file__))" 2>/dev/null || true)

  if [ -z "$PYSIDE_SRC" ] || [ ! -d "$PYSIDE_SRC" ]; then
    echo "[deb][ERROR] Host Python is missing PySide6. Install it first (e.g. python3 -m pip install PySide6) and retry." >&2
    exit 1
  fi

  # 2. Identify the actual site-packages directory where PySide6 lives
  ACTUAL_SITE_PACKAGES=$(dirname "$PYSIDE_SRC")

  # 3. Copy PySide6
  rsync -a "$PYSIDE_SRC" "$DST_SITE/"

  # 4. Find and Copy shiboken6 (Try same folder first, then dynamic check)
  if [ -d "$ACTUAL_SITE_PACKAGES/shiboken6" ]; then
    rsync -a "$ACTUAL_SITE_PACKAGES/shiboken6" "$DST_SITE/"
  else
    # Fallback: check if shiboken6 is installed elsewhere
    SHIBOKEN_SRC=$(python3 -c "import os, shiboken6; print(os.path.dirname(shiboken6.__file__))" 2>/dev/null || true)
    if [ -n "$SHIBOKEN_SRC" ] && [ -d "$SHIBOKEN_SRC" ]; then
      rsync -a "$SHIBOKEN_SRC" "$DST_SITE/"
    else
      echo "[deb][WARN] shiboken6 module not found via path check."
    fi
  fi

  # 5. Copy .dist-info for PySide/Shiboken from the discovered source dir
  DIST_PACKAGES=(PySide6 shiboken6 PySide6_Addons PySide6_Essentials)
  for name in "${DIST_PACKAGES[@]}"; do
    src=$(find "$ACTUAL_SITE_PACKAGES" -maxdepth 1 -type d -iname "${name}*.dist-info" | head -n1 || true)
    if [ -n "$src" ] && [ -d "$src" ]; then
      rsync -a "$src" "$DST_SITE/"
    else
      echo "[deb][WARN] dist-info for ${name} not found under $ACTUAL_SITE_PACKAGES; continuing."
    fi
  done
  # --- [FIXED SECTION END] ---

  # licensing package no longer needed — validation uses home-next API
  echo "[deb] Embedded venv ready at $VENV_DIR (copied from host PySide6)"
fi

echo "[deb] Embedding project payload for HOME deployment..."
# Exclude large/ephemeral build artifacts to speed up packaging
RSYNC_EXCLUDES=(
  '.git'
  '.github'
  'release'
  '.vscode'
  '.idea'
  '/datasets'
  '/datasets/**'
  'node_modules'
  'frontend/node_modules'
  'frontend/.quasar'
  'logs'
  'log'
  'build'
  'dist'
  'install'
  # ros2: 모듈은 런처에서 설치, 빌드 산출물 제외
  'ros2/ros2_ws/src'
  'ros2/ros2_ws/build'
  'ros2/ros2_ws/install'
  'ros2/ros2_ws/log'
  'ros2/ros2_ws/logs'
  'ros2/robot_sdk'
  'backend/modules'
  # training_server is downloaded on-demand by the launcher when the user opts
  # into local training (CI publishes it as a separate Easy-Trainer-Modules tar.gz).
  'training_server'
  # 런타임 프로젝트에 불필요한 폴더
  'home-next'
  'modules'
  'python_pkgs'
  'cmake_pkgs'
  'scripts'
  'CLAUDE.md'
  '**/__pycache__'
  '*.pyc'
)
RSYNC_EXCLUDE_ARGS=()
for pat in "${RSYNC_EXCLUDES[@]}"; do
  RSYNC_EXCLUDE_ARGS+=("--exclude" "$pat")
done
rsync -a "${RSYNC_EXCLUDE_ARGS[@]}" "$ROOT_DIR/" "$STAGE${PAYLOAD_DIR}/"

# Guard against dpkg-ar 10GB member limit by detecting oversized payload early
MAX_PAYLOAD_BYTES=${MAX_PAYLOAD_BYTES:-9000000000} # ~9GB safety margin
PAYLOAD_SIZE=$(du -sb "$STAGE${PAYLOAD_DIR}" | awk '{print $1}')
if [ "${SKIP_PAYLOAD_SIZE_CHECK:-0}" != "1" ] && [ "$PAYLOAD_SIZE" -ge "$MAX_PAYLOAD_BYTES" ]; then
  echo "[deb][ERROR] Payload size $PAYLOAD_SIZE bytes exceeds safety limit $MAX_PAYLOAD_BYTES (ar member limit ~10GB)."
  echo "[deb][ERROR] Remove or exclude large files (datasets/checkpoints/build artifacts) or raise MAX_PAYLOAD_BYTES, then retry."
  echo "[deb][INFO] Top 15 largest files in payload:"
  find "$STAGE${PAYLOAD_DIR}" -type f -printf '%s\t%p\n' | sort -rn | head -n 15 | numfmt --to=iec --field=1 || true
  exit 1
fi

# Payload sanity check removed by request.
# Ensure critical code paths exist in the payload (avoid shipping partial copies)
REQUIRED_PATHS=(
  "$STAGE${PAYLOAD_DIR}/docker-compose.yml"
  "$STAGE${PAYLOAD_DIR}/backend/Dockerfile"
  "$STAGE${PAYLOAD_DIR}/backend/entrypoint.sh"
  "$STAGE${PAYLOAD_DIR}/backend/api/app.py"
  "$STAGE${PAYLOAD_DIR}/frontend/Dockerfile"
  "$STAGE${PAYLOAD_DIR}/ros2/Dockerfile"
)
for req in "${REQUIRED_PATHS[@]}"; do
  if [ ! -f "$req" ]; then
    echo "[deb][ERROR] Missing required payload file: $req" >&2
    exit 1
  fi
done

# Launcher wrapper
cat > "$STAGE/usr/bin/easytrainer-launcher" <<'EOF'
#!/usr/bin/env bash
set -e
export QT_QPA_PLATFORMTHEME=
export QT_STYLE_OVERRIDE=Fusion
# Improve compatibility on Wayland and avoid sandbox issues
export QTWEBENGINE_DISABLE_SANDBOX=1
if [ "${XDG_SESSION_TYPE:-}" = "wayland" ]; then
  export QT_QPA_PLATFORM=xcb
fi

# Log to /tmp to avoid host permission issues
DATA_ROOT="${EASYTRAINER_DATA_DIR:-/opt/easytrainer}"
export EASYTRAINER_DATA_DIR="${DATA_ROOT}"
LOG_DIR="${EASYTRAINER_LOG_DIR:-/tmp/easytrainer/logs}"
export EASYTRAINER_LOG_DIR="${LOG_DIR}"
export EASYTRAINER_CONFIG_PATH="${EASYTRAINER_CONFIG_PATH:-${DATA_ROOT}/config.json}"
if ! mkdir -p "$LOG_DIR"; then
  sudo mkdir -p "$LOG_DIR"
fi
LOG_FILE="$LOG_DIR/launcher.log"
if ! touch "$LOG_FILE" 2>/dev/null; then
  sudo touch "$LOG_FILE" 2>/dev/null || true
fi
if ! chmod 666 "$LOG_FILE" 2>/dev/null; then
  sudo chmod 666 "$LOG_FILE" 2>/dev/null || true
fi
# Fallback to /tmp if primary log path is still unwritable
if ! touch "$LOG_FILE" 2>/dev/null; then
  FALLBACK_LOG="/tmp/easytrainer-launcher.log"
  mkdir -p "$(dirname "$FALLBACK_LOG")" 2>/dev/null || true
  touch "$FALLBACK_LOG" 2>/dev/null || true
  chmod 666 "$FALLBACK_LOG" 2>/dev/null || true
  LOG_FILE="$FALLBACK_LOG"
fi
{
LAUNCHER_BIN="/opt/easytrainer/EasyLauncher"
VENV_PY="/opt/easytrainer/venv/bin/python"
if [ -x "$LAUNCHER_BIN" ]; then
  exec "$LAUNCHER_BIN" "$@"
elif [ -x "$VENV_PY" ] && [ -f "/opt/easytrainer/ui/main.py" ]; then
  exec "$VENV_PY" /opt/easytrainer/ui/main.py "$@"
else
  echo "Launcher runtime not found. Please reinstall the package." >&2
  exit 1
fi
} >>"$LOG_FILE" 2>&1
EOF
chmod 0755 "$STAGE/usr/bin/easytrainer-launcher"

# Prepare utility to copy payload into /opt (writable project for user)
cat > "$STAGE/usr/bin/easytrainer-prepare" <<'EOF'
#!/usr/bin/env bash
set -e

DEST="/opt/easytrainer/project"
SRC="/usr/share/easytrainer-project"

usage() {
  echo "Usage: easytrainer-prepare [--force]" >&2
}

FORCE=0
if [ "${1:-}" = "--force" ]; then
  FORCE=1
elif [ -n "${1:-}" ]; then
  usage; exit 1
fi

if [ "$FORCE" = "1" ] && [ -d "$DEST" ]; then
  rm -rf "$DEST"
fi

if [ ! -d "$DEST" ]; then
  sudo mkdir -p "$DEST"
  sudo cp -a "$SRC"/. "$DEST"/
  echo "Project copied to $DEST"
else
  echo "Project already exists at $DEST (use --force to overwrite)"
fi
EOF
chmod 0755 "$STAGE/usr/bin/easytrainer-prepare"

# Desktop entry
ICON_PATH="/opt/easytrainer/app_icon.png"
cat > "$STAGE/usr/share/applications/EasyTrainer.desktop" <<EOF
[Desktop Entry]
Name=EasyTrainer
Exec=/usr/bin/easytrainer-launcher
Icon=$ICON_PATH
Terminal=false
Type=Application
Categories=Utility;Development;
EOF
chmod 0644 "$STAGE/usr/share/applications/EasyTrainer.desktop"

# Control file
# Runtime GUI libs needed by Qt (ensure auto-install by apt)
RUNTIME_LIBS="libxcb-cursor0, libxkbcommon-x11-0, libxcb-icccm4, libxcb-image0, libxcb-keysyms1, libxcb-render-util0, libxcb-xinerama0, libegl1, libgl1-mesa-dri, libopengl0, libnss3, libasound2, libnvidia-egl-wayland1"
if [ "$USE_PYINSTALLER" = "1" ]; then
  DEPENDS_LINE="Depends: xdg-utils, ${RUNTIME_LIBS}"
else
  DEPENDS_LINE="Depends: python3 (>= ${PY_VER}), python3 (<< ${PY_MAJOR}.${PY_NEXT_MINOR}), xdg-utils, ${RUNTIME_LIBS}"
fi

cat > "$STAGE/DEBIAN/control" <<EOF
Package: ${PKG}
Version: ${VERSION_VAL}
Section: utils
Priority: optional
Architecture: ${ARCH}
Maintainer: EasyTrainer <noreply@example.com>
${DEPENDS_LINE}
Recommends: docker.io, docker-compose-plugin
Description: Easy Trainer launcher for the containerized runtime
 Provides a Qt launcher to build/start the Easy Trainer service,
 bundles the current project under /opt/easytrainer/project,
 embeds a local PySide6 runtime, and installs a desktop entry.
 Docker/Compose is recommended; NVIDIA runtime is used for GPU builds.
EOF

# postinst
cat > "$STAGE/DEBIAN/postinst" <<'EOF'
#!/bin/sh
set -e

umask 022

if command -v update-desktop-database >/dev/null 2>&1; then
  update-desktop-database || true
fi

# Resolve target user (fallback to first non-system user, else root)
TARGET_USER=""
if [ -n "$SUDO_USER" ]; then
  TARGET_USER="$SUDO_USER"
elif TARGET_USER="$(logname 2>/dev/null || true)"; then
  :
fi
if [ -z "$TARGET_USER" ] || ! id "$TARGET_USER" >/dev/null 2>&1; then
  TARGET_USER="$(getent passwd | awk -F: '$3>=1000 && $3<65534 {print $1; exit}')"
fi
if [ -z "$TARGET_USER" ] || ! id "$TARGET_USER" >/dev/null 2>&1; then
  TARGET_USER="root"
fi
TARGET_GROUP="$(id -gn "$TARGET_USER" 2>/dev/null || echo "$TARGET_USER")"

# Add user to docker group (best-effort)
if id "$TARGET_USER" >/dev/null 2>&1; then
  if ! id -nG "$TARGET_USER" | grep -qw docker; then
    adduser "$TARGET_USER" docker || true
    echo "User $TARGET_USER added to docker group. Please log out/in to apply."
  fi
fi

# Copy project payload into /opt and make it writable by the target user
DEST="/opt/easytrainer/project"
SRC="/usr/share/easytrainer-project"

# Ensure destination exists with correct ownership/permissions
install -d -m 775 -o "$TARGET_USER" -g "$TARGET_GROUP" /opt/easytrainer
install -d -m 775 -o "$TARGET_USER" -g "$TARGET_GROUP" "$DEST"
# Pre-create training_server data dir (used when EASYTRAINER_LOCAL_TRAINING=1
# spawns training_server inside the backend container).
install -d -m 775 -o "$TARGET_USER" -g "$TARGET_GROUP" /opt/easytrainer/training_data
install -d -m 775 -o "$TARGET_USER" -g "$TARGET_GROUP" /opt/easytrainer/logs
# Logs live in /tmp for runtime; keep this dir for legacy safety only
install -d -m 775 -o "$TARGET_USER" -g "$TARGET_GROUP" /opt/easytrainer/logs
touch /opt/easytrainer/logs/.keep >/dev/null 2>&1 || true
chown "$TARGET_USER":"$TARGET_GROUP" /opt/easytrainer/logs/.keep 2>/dev/null || true
# Initialize config file location for the launcher
CONFIG_PATH="/opt/easytrainer/config.json"
if [ ! -f "$CONFIG_PATH" ]; then
  echo "{}" > "$CONFIG_PATH" || true
fi
chown "$TARGET_USER":"$TARGET_GROUP" "$CONFIG_PATH" 2>/dev/null || true
chmod 664 "$CONFIG_PATH" 2>/dev/null || true
# Pre-create runtime log location under /tmp
install -d -m 777 /tmp/easytrainer/logs || true
install -m 666 /dev/null /tmp/easytrainer/logs/launcher.log || true

# Normalize ownership/permissions for everything under /opt/easytrainer
chown -R "$TARGET_USER":"$TARGET_GROUP" /opt/easytrainer || true
chmod 777 /opt/easytrainer /opt/easytrainer/logs || true
chmod 666 /opt/easytrainer/config.json /opt/easytrainer/logs/launcher.log 2>/dev/null || true

if [ -d "$SRC" ]; then
  # Clean old project files (preserve user data: modules, databases)
  if [ -d "$DEST" ]; then
    find "$DEST" -maxdepth 1 -type f -delete 2>/dev/null || true
    for d in backend frontend ros2 release scripts cmake_pkgs python_pkgs; do
      rm -rf "$DEST/$d" 2>/dev/null || true
    done
  fi
  cp -a "$SRC"/. "$DEST"/ || true
  # Pre-create runtime directories excluded from the deb payload
  install -d -m 775 -o "$TARGET_USER" -g "$TARGET_GROUP" "$DEST/ros2/robot_sdk" 2>/dev/null || true
  install -d -m 775 -o "$TARGET_USER" -g "$TARGET_GROUP" "$DEST/modules" 2>/dev/null || true
  chown -R "$TARGET_USER":"$TARGET_GROUP" "$DEST" || true
  chmod -R u+rwX "$DEST" || true
  echo "Project placed at $DEST (owner: $TARGET_USER)"
else
  echo "Warning: payload not found at $SRC; skipping copy" >&2
fi

exit 0
EOF
chmod 0755 "$STAGE/DEBIAN/postinst"

# Use faster compression to reduce build time (override with DPKG_COMP_OPTS)
# Default: try zstd (multithread), fall back to gzip if unavailable/overridden.
if [ -z "${DPKG_COMP_OPTS:-}" ]; then
  if dpkg-deb --help 2>&1 | grep -q -- "--uniform-compression" && dpkg-deb --help 2>&1 | grep -qi zstd; then
    DPKG_COMP_OPTS="--threads=0 --uniform-compression -Zzstd -z3"
  else
    DPKG_COMP_OPTS="--uniform-compression -Zgzip -z1"
  fi
fi
echo "[deb] Using dpkg-deb compression options: $DPKG_COMP_OPTS"
dpkg-deb --build --root-owner-group $DPKG_COMP_OPTS "$STAGE" "$DEB_NAME"
echo "[deb] Built: $DEB_NAME"

# Clean up staging unless KEEP_STAGE=1
if [ "${KEEP_STAGE:-0}" != "1" ]; then
  rm -rf "$STAGE"
  echo "[deb] Cleaned staging folder (set KEEP_STAGE=1 to keep it)."
else
  echo "[deb] Keeping staging folder per KEEP_STAGE=1."
fi
