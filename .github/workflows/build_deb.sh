#!/usr/bin/env bash
set -e

# ==========================================
# 1. 설정 변수
# ==========================================
PKG_NAME="easytrainer"
BINARY_NAME="EasyLauncher"
ARCH="amd64"

# 버전 파일 읽기
if [ -f "VERSION" ]; then
    VERSION=$(cat VERSION | tr -d ' \t\r\n')
else
    VERSION="1.0.0"
fi

STAGE_DIR="deb_stage"
INSTALL_DIR="/opt/$PKG_NAME"
PROJECT_ROOT=$(pwd)

echo "[DEB] 패키징 시작: $PKG_NAME ($VERSION)"

# ==========================================
# 2. 청소 및 디렉토리 구조 생성
# ==========================================
rm -rf "$STAGE_DIR"
mkdir -p "$STAGE_DIR/DEBIAN"
mkdir -p "$STAGE_DIR$INSTALL_DIR"             # /opt/easytrainer
mkdir -p "$STAGE_DIR$INSTALL_DIR/project"     # 프로젝트 원본이 들어갈 곳 (중요 복구!)
mkdir -p "$STAGE_DIR/usr/bin"
mkdir -p "$STAGE_DIR/usr/share/applications"

# ==========================================
# 3. [핵심] 프로젝트 전체 파일(Payload) 복사
# ==========================================
# 기존 스크립트의 로직을 복원했습니다.
# 실행 파일 외에 Docker Compose, Backend 소스, Config 파일 등을 통째로 복사합니다.
echo "[DEB] 프로젝트 전체 파일(Payload) 복사 중..."

# 제외할 것들만 쏙 빼고 나머지는 다 넣습니다.
rsync -a \
    --exclude='.git' \
    --exclude='.github' \
    --exclude='.gitignore' \
    --exclude='.vscode' \
    --exclude='.idea' \
    --exclude='__pycache__' \
    --exclude='*.pyc' \
    --exclude='dist' \
    --exclude='build' \
    --exclude='venv' \
    --exclude='deb_stage' \
    --exclude='*.deb' \
    --exclude='*.spec' \
    --exclude='temp_obf' \
    "$PROJECT_ROOT/" "$STAGE_DIR$INSTALL_DIR/project/"

echo " - 프로젝트 파일들이 $STAGE_DIR$INSTALL_DIR/project/ 에 복사되었습니다."

# ==========================================
# 4. PyInstaller 실행 파일(Binary) 심기
# ==========================================
if [ -f "dist/$BINARY_NAME" ]; then
    echo "[DEB] PyInstaller 바이너리 복사 중..."
    # 바이너리는 프로젝트 폴더 안에 넣거나, 상위 폴더에 둡니다.
    # 여기서는 /opt/easytrainer/EasyLauncher 위치에 둡니다.
    cp "dist/$BINARY_NAME" "$STAGE_DIR$INSTALL_DIR/$PKG_NAME"
    chmod 755 "$STAGE_DIR$INSTALL_DIR/$PKG_NAME"
else
    echo "[ERROR] dist/$BINARY_NAME 파일이 없습니다."
    exit 1
fi

# 아이콘 복사
if [ -f "app_icon.png" ]; then
    cp "app_icon.png" "$STAGE_DIR$INSTALL_DIR/"
fi

# ==========================================
# 5. 실행 래퍼 스크립트 (경로 연결)
# ==========================================
echo "[DEB] 실행 래퍼 스크립트 생성..."

# 런처가 실행될 때 "내 프로젝트 파일들이 어디 있는지" 알 수 있게 환경변수를 꽂아줍니다.
cat > "$STAGE_DIR/usr/bin/$PKG_NAME" <<EOF
#!/bin/bash
# 런처가 참조할 프로젝트 루트 경로
export EASYTRAINER_ROOT="$INSTALL_DIR"
export EASYTRAINER_DATA_DIR="$INSTALL_DIR"

# 런처 실행 시 작업 디렉토리(CWD)를 프로젝트 폴더로 이동 (Docker Compose 실행 등을 위해)
cd "$INSTALL_DIR/project"

# Wayland 호환성
if [ "\${XDG_SESSION_TYPE}" = "wayland" ]; then
    export QT_QPA_PLATFORM=xcb
fi

# 실행 (로그는 /tmp 등에 남기거나 런처 내부 로직 따름)
exec "$INSTALL_DIR/$PKG_NAME" "\$@"
EOF
chmod 755 "$STAGE_DIR/usr/bin/$PKG_NAME"

# ==========================================
# 6. .desktop 파일 및 Control 파일
# ==========================================
cat > "$STAGE_DIR/usr/share/applications/$PKG_NAME.desktop" <<EOF
[Desktop Entry]
Name=EasyTrainer
Exec=/usr/bin/$PKG_NAME
Icon=$INSTALL_DIR/app_icon.png
Terminal=false
Type=Application
Categories=Development;Utility;
EOF

cat > "$STAGE_DIR/DEBIAN/control" <<EOF
Package: $PKG_NAME
Version: $VERSION
Section: utils
Priority: optional
Architecture: $ARCH
Maintainer: EasyTrainer <noreply@example.com>
Recommends: docker.io, docker-compose-plugin
Description: Easy Trainer Launcher
 Full project payload included.
EOF

# ==========================================
# 7. Postinst (권한 설정 - 중요!)
# ==========================================
cat > "$STAGE_DIR/DEBIAN/postinst" <<EOF
#!/bin/sh
set -e
if command -v update-desktop-database >/dev/null 2>&1; then
  update-desktop-database || true
fi

# 프로젝트 파일들에 사용자가 쓰기 권한이 있어야 Docker 실행/로그 생성이 가능함
# 보안상 777은 좋지 않으나 로컬 툴 특성상 편의를 위해 허용하거나,
# 혹은 chown으로 현재 사용자에게 소유권을 넘기는 로직이 필요함.
chmod -R 777 "$INSTALL_DIR"

echo "EasyTrainer installed with full project payload."
EOF
chmod 755 "$STAGE_DIR/DEBIAN/postinst"

# ==========================================
# 8. 빌드
# ==========================================
dpkg-deb --build "$STAGE_DIR" "${PKG_NAME}_${VERSION}_${ARCH}.deb"
echo "✅ [SUCCESS] 빌드 완료 (Payload 포함)"