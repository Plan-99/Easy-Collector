#!/usr/bin/env bash
set -e  # 에러 발생 시 즉시 종료

# ==========================================
# 1. 설정 변수
# ==========================================
PKG_NAME="easytrainer"
BINARY_NAME="EasyLauncher"  # PyInstaller로 빌드된 실행 파일명
ARCH="amd64"

# 버전 자동 감지 (VERSION 파일이 있으면 읽고, 없으면 기본값)
if [ -f "VERSION" ]; then
    VERSION=$(cat VERSION | tr -d ' \t\r\n')
else
    VERSION="1.0.0"
fi

# 경로 설정
STAGE_DIR="deb_stage"
INSTALL_DIR="/opt/$PKG_NAME"
PROJECT_ROOT=$(pwd)

echo "[DEB] 패키징 시작: $PKG_NAME ($VERSION) - Architecture: $ARCH"

# ==========================================
# 2. 청소 및 디렉토리 구조 생성
# ==========================================
echo "[DEB] 디렉토리 초기화 중..."
rm -rf "$STAGE_DIR"
mkdir -p "$STAGE_DIR/DEBIAN"
mkdir -p "$STAGE_DIR$INSTALL_DIR"             # /opt/easytrainer
mkdir -p "$STAGE_DIR$INSTALL_DIR/logs"        # 로그 폴더 미리 생성
mkdir -p "$STAGE_DIR/usr/bin"                 # 실행 래퍼용
mkdir -p "$STAGE_DIR/usr/share/applications"  # 아이콘용

# ==========================================
# 3. 메인 실행 파일(Binary) 복사
# ==========================================
if [ -f "dist/$BINARY_NAME" ]; then
    echo "[DEB] PyInstaller 바이너리 복사 중..."
    cp "dist/$BINARY_NAME" "$STAGE_DIR$INSTALL_DIR/$PKG_NAME"
    chmod 755 "$STAGE_DIR$INSTALL_DIR/$PKG_NAME"
else
    echo "[ERROR] dist/$BINARY_NAME 파일이 없습니다. PyInstaller 빌드를 먼저 수행하세요."
    exit 1
fi

# ==========================================
# 4. 리소스 및 페이로드 복사 (Legacy 로직 통합)
# ==========================================
# 실행 파일 외에 Docker Compose, 백엔드 소스 등이 필요하므로 복사합니다.
# 불필요한 파일(.git, venv 등)은 제외합니다.
echo "[DEB] 프로젝트 리소스(Payload) 복사 중..."

rsync -a \
    --exclude='.git' \
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
    "$PROJECT_ROOT/" "$STAGE_DIR$INSTALL_DIR/project/"

# 아이콘 복사
if [ -f "app_icon.png" ]; then
    cp "app_icon.png" "$STAGE_DIR$INSTALL_DIR/"
fi

# ==========================================
# 5. 실행 래퍼 스크립트 생성 (중요!)
# ==========================================
# 단순히 심볼릭 링크를 거는 대신, 환경변수를 설정하는 래퍼를 만듭니다.
# (Wayland 호환성 및 로그 경로 설정 등을 위해 필요)
echo "[DEB] 실행 래퍼 스크립트 생성 중..."

cat > "$STAGE_DIR/usr/bin/$PKG_NAME" <<EOF
#!/bin/bash
export EASYTRAINER_ROOT="$INSTALL_DIR"
export EASYTRAINER_DATA_DIR="$INSTALL_DIR/project"

# Qt Wayland 호환성 처리 (Ubuntu 22.04+ 등에서 화면 안 뜨는 문제 방지)
if [ "\${XDG_SESSION_TYPE}" = "wayland" ]; then
    export QT_QPA_PLATFORM=xcb
fi

# 실행
exec "$INSTALL_DIR/$PKG_NAME" "\$@"
EOF
chmod 755 "$STAGE_DIR/usr/bin/$PKG_NAME"

# ==========================================
# 6. .desktop 파일 생성
# ==========================================
echo "[DEB] .desktop 파일 생성 중..."
cat > "$STAGE_DIR/usr/share/applications/$PKG_NAME.desktop" <<EOF
[Desktop Entry]
Name=EasyTrainer
Comment=Easy Trainer Launcher
Exec=/usr/bin/$PKG_NAME
Icon=$INSTALL_DIR/app_icon.png
Terminal=false
Type=Application
Categories=Development;Utility;
EOF

# ==========================================
# 7. DEBIAN/control 파일 생성
# ==========================================
# PyInstaller를 쓰므로 python 의존성은 뺍니다. 
# 하지만 Docker를 사용한다면 추천 패키지(Recommends)에 넣습니다.
echo "[DEB] Control 파일 생성 중..."
cat > "$STAGE_DIR/DEBIAN/control" <<EOF
Package: $PKG_NAME
Version: $VERSION
Section: utils
Priority: optional
Architecture: $ARCH
Maintainer: EasyTrainer <noreply@example.com>
Recommends: docker.io, docker-compose-plugin
Description: Easy Trainer Launcher (Binary Build)
 This package installs the PyInstaller-built launcher and necessary
 project files (docker-compose, scripts) to run Easy Trainer.
EOF

# ==========================================
# 8. DEBIAN/postinst 스크립트 생성 (권한 설정)
# ==========================================
# 설치 후 /opt/easytrainer 폴더 권한을 풀어주어 사용자가 로그/설정을 쓸 수 있게 합니다.
echo "[DEB] postinst 스크립트 생성 중..."
cat > "$STAGE_DIR/DEBIAN/postinst" <<EOF
#!/bin/sh
set -e

# 아이콘 캐시 갱신
if command -v update-desktop-database >/dev/null 2>&1; then
  update-desktop-database || true
fi

# 권한 설정: 설치된 디렉토리에 사용자가 쓰기 권한이 있어야 Docker나 로그 생성이 원활함
# 보안상 완벽하진 않으나, 로컬 개발 도구 특성상 777 또는 사용자 소유로 변경
chmod -R 777 "$INSTALL_DIR"

echo "EasyTrainer installed successfully."
EOF
chmod 755 "$STAGE_DIR/DEBIAN/postinst"

# ==========================================
# 9. 최종 빌드 (.deb 생성)
# ==========================================
echo "[DEB] .deb 패키지 빌드 중..."
dpkg-deb --build "$STAGE_DIR" "${PKG_NAME}_${VERSION}_${ARCH}.deb"

echo ""
echo "✅ [SUCCESS] 빌드 완료: ${PKG_NAME}_${VERSION}_${ARCH}.deb"
echo "👉 설치 방법: sudo dpkg -i ${PKG_NAME}_${VERSION}_${ARCH}.deb"