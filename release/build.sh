#!/usr/bin/env bash
set -euo pipefail

PKG=easytrainer
ARCH=${ARCH:-amd64}
INSTALL_ROOT=/opt/easytrainer
PAYLOAD_DIR=/usr/share/easytrainer-project

# Paths
RELEASE_DIR=$(cd "$(dirname "$0")" && pwd)
ROOT_DIR=$(cd "$RELEASE_DIR/.." && pwd)

# Version handling: control file always needs a Version, but filename can omit it
VERSION_ENV="${VERSION:-}"
if [ -n "$VERSION_ENV" ]; then
  VERSION_VAL="$VERSION_ENV"
  STAGE="$RELEASE_DIR/${PKG}_${VERSION_VAL}_${ARCH}"
  DEB_NAME="$STAGE.deb"
else
  VERSION_VAL="1.0.0"  # default for control metadata only
  STAGE="$RELEASE_DIR/${PKG}_${ARCH}"
  DEB_NAME="$STAGE.deb"
fi

echo "[deb] Building $PKG ${VERSION_ENV:-no-version} ($ARCH)"
rm -rf "$STAGE" && mkdir -p "$STAGE/DEBIAN"

# Install layout
mkdir -p \
  "$STAGE/usr/bin" \
  "$STAGE/usr/share/applications" \
  "$STAGE${INSTALL_ROOT}/ui" \
  "$STAGE${PAYLOAD_DIR}"

echo "[deb] Copying launcher UI (release/ui -> ${INSTALL_ROOT}/ui)..."
rsync -a "$RELEASE_DIR/ui/" "$STAGE${INSTALL_ROOT}/ui/"
if [ -f "$ROOT_DIR/app_icon.png" ]; then
  cp "$ROOT_DIR/app_icon.png" "$STAGE${INSTALL_ROOT}/app_icon.png"
fi

# Always embed a Python virtualenv with PySide6 to avoid apt GUI deps
VENV_EMBEDDED=1
if ! python3 -c 'import ensurepip' 2>/dev/null; then
  echo "[deb][ERROR] python3-venv (ensurepip) is not available on this build machine."
  echo "Install it and retry: sudo apt-get update && sudo apt-get install -y python3-venv"
  exit 1
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
echo "[deb] Creating embedded venv with Qt..."
VENV_DIR="$STAGE${INSTALL_ROOT}/venv"
python3 -m venv "$VENV_DIR"
# Keep default pip inside venv to avoid network upgrades
echo "[deb] Copying existing PySide6 install into venv..."
HOST_SITE=$(python3 - <<'PY'
import site
print(site.getusersitepackages())
PY
)
DST_SITE="$VENV_DIR/lib/python${PY_VER}/site-packages"
mkdir -p "$DST_SITE"
if [ ! -d "$HOST_SITE/PySide6" ] || [ ! -d "$HOST_SITE/shiboken6" ]; then
  echo "[deb][ERROR] Host Python is missing PySide6/shiboken6. Install them first (e.g. python3 -m pip install --user PySide6) and retry." >&2
  exit 1
fi
for pkg in PySide6 shiboken6; do
  rsync -a "$HOST_SITE/$pkg" "$DST_SITE/"
done
DIST_PACKAGES=(PySide6 shiboken6 PySide6_Addons PySide6_Essentials)
for name in "${DIST_PACKAGES[@]}"; do
  src=$(find "$HOST_SITE" -maxdepth 1 -type d -iname "${name}*.dist-info" | head -n1 || true)
  if [ -n "$src" ] && [ -d "$src" ]; then
    rsync -a "$src" "$DST_SITE/"
  else
    echo "[deb][WARN] dist-info for ${name} not found under $HOST_SITE; continuing."
  fi
done
echo "[deb] Embedded venv ready at $VENV_DIR (copied from host PySide6)"

echo "[deb] Embedding project payload for HOME deployment..."
# Exclude large/ephemeral build artifacts to speed up packaging
RSYNC_EXCLUDES=(
  '.git'
  'release'
  '.vscode'
  '.idea'
  'data'
  'dataset'
  'datasets'
  'training_data'
  'train_data'
  'checkpoints'
  'weights'
  'models'
  'runs'
  'outputs'
  'artifacts'
  'node_modules'
  'src/ui/node_modules'
  'src/ui/.quasar'
  'logs'
  'log'
  'build'
  'install'
  'ros2_ws/build'
  'ros2_ws/install'
  'ros2_ws/log'
  'ros2_ws/logs'
  '**/__pycache__'
  '*.pyc'
  'python_pkgs/**/build'
  'python_pkgs/**/dist'
  'python_pkgs/**/*.egg-info'
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

# Log to user data dir for diagnostics
LOG_DIR="${XDG_DATA_HOME:-$HOME/.local/share}/EasyTrainer"
mkdir -p "$LOG_DIR"
LOG_FILE="$LOG_DIR/launcher.log"
{
VENV_PY="/opt/easytrainer/venv/bin/python"
if [ -x "$VENV_PY" ]; then
  exec "$VENV_PY" /opt/easytrainer/ui/main.py "$@"
else
  echo "Embedded runtime not found. Please reinstall the package." >&2
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
RUNTIME_LIBS="libxcb-cursor0, libxkbcommon-x11-0, libxcb-icccm4, libxcb-image0, libxcb-keysyms1, libxcb-render-util0, libxcb-xinerama0, libegl1, libgl1-mesa-dri, libopengl0, libnss3, libasound2"
DEPENDS_LINE="Depends: python3 (>= ${PY_VER}), python3 (<< ${PY_MAJOR}.${PY_NEXT_MINOR}), xdg-utils, ${RUNTIME_LIBS}"

cat > "$STAGE/DEBIAN/control" <<EOF
Package: ${PKG}
Version: ${VERSION_VAL}
Section: utils
Priority: optional
Architecture: ${ARCH}
Maintainer: EasyTrainer <noreply@example.com>
${DEPENDS_LINE}
Recommends: docker.io, docker-compose-plugin
Description: EasyTrainer Launcher (+ project payload under /opt)
 Installs a Qt launcher and embeds the project payload.
 On install, the project is placed under /opt/easytrainer/project.
EOF

# postinst
cat > "$STAGE/DEBIAN/postinst" <<'EOF'
#!/bin/sh
set -e

if command -v update-desktop-database >/dev/null 2>&1; then
  update-desktop-database || true
fi

TARGET_USER=""
if [ -n "$SUDO_USER" ]; then
  TARGET_USER="$SUDO_USER"
else
  TARGET_USER="$(logname 2>/dev/null || true)"
fi

if [ -n "$TARGET_USER" ] && id "$TARGET_USER" >/dev/null 2>&1; then
  if ! id -nG "$TARGET_USER" | grep -qw docker; then
    adduser "$TARGET_USER" docker || true
    echo "User $TARGET_USER added to docker group. Please log out/in to apply."
  fi
  # Copy project into /opt if missing and make it writable by the target user
  DEST="/opt/easytrainer/project"
  SRC="/usr/share/easytrainer-project"
  if [ ! -d "$DEST" ]; then
    mkdir -p "$DEST"
    cp -a "$SRC"/. "$DEST"/
    chown -R "$TARGET_USER":"$TARGET_USER" "$DEST"
    echo "Project placed at $DEST"
  fi
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
