#!/bin/bash

# ===================================================================
# 스크립트 설정 (필요에 따라 수정)
# ===================================================================
# 스크립트가 오류 발생 시 즉시 중단되도록 설정
set -e

# Docker 이미지 및 컨테이너 이름 설정
DOCKER_IMAGE_NAME="easy-collector:latest"
CONTAINER_NAME="easy_collector_container"

# AppImage 결과물 이름 설정
APP_NAME="EasyTrainer"

# 작업 디렉토리 설정
BUILD_DIR="easy_trainer_build"
APPDIR_PATH="$BUILD_DIR/$APP_NAME.AppDir"

# ===================================================================
# 1단계: 사전 준비 및 환경 정리
# ===================================================================
echo "▶ (1/7) 이전 빌드 파일 정리..."
# 기존 빌드 디렉토리가 있다면 삭제
if [ -d "$BUILD_DIR" ]; then
    rm -rf "$BUILD_DIR"
fi
mkdir -p "$APPDIR_PATH"
echo "✔ 빌드 디렉토리 준비 완료: $APPDIR_PATH"

# appimagetool 다운로드 (없을 경우)
if [ ! -f "appimagetool" ]; then
    echo "▶ appimagetool 다운로드 중..."
    wget "https://github.com/AppImage/AppImageKit/releases/download/continuous/appimagetool-x86_64.AppImage" -O appimagetool
    chmod +x appimagetool
fi

# ===================================================================
# 2단계: 최신 코드로 Docker 이미지 빌드
# ===================================================================
echo "▶ (2/7) 최신 코드로 Docker 이미지 빌드 시작..."
docker build -t "$DOCKER_IMAGE_NAME" .
echo "✔ Docker 이미지 빌드 완료: $DOCKER_IMAGE_NAME"

# ===================================================================
# 3단계: Docker 컨테이너에서 빌드된 파일 추출
# ===================================================================
echo "▶ (3/7) 파일 추출을 위한 임시 컨테이너 생성..."
# 기존에 같은 이름의 컨테이너가 있다면 삭제
docker rm -f "$CONTAINER_NAME" 2>/dev/null || true
# 컨테이너 생성
docker create --name "$CONTAINER_NAME" "$DOCKER_IMAGE_NAME"
echo "✔ 임시 컨테이너 생성 완료: $CONTAINER_NAME"

echo "▶ (4/7) 컨테이너에서 AppDir로 파일 복사 시작..."
# 이 과정은 시스템에 따라 수 분 이상 소요될 수 있습니다.

# ROS2 시스템 복사
echo "  - ROS2 시스템 복사 중... (시간이 오래 걸립니다)"
docker cp "$CONTAINER_NAME:/opt/ros/humble" "$APPDIR_PATH/opt/ros"

# Python 실행 파일 및 라이브러리 복사
echo "  - Python 및 의존성 라이브러리 복사 중..."
# site-packages 경로를 자동으로 찾기 (더 견고한 방법)
PY_SITE_PACKAGES=$(docker run --rm "$DOCKER_IMAGE_NAME" python3 -c "import site; print(site.getsitepackages()[0])")
mkdir -p "$APPDIR_PATH/usr/lib/python"
docker cp "$CONTAINER_NAME:$PY_SITE_PACKAGES" "$APPDIR_PATH/usr/lib/python/site-packages"
mkdir -p "$APPDIR_PATH/usr/bin"
docker cp "$CONTAINER_NAME:/usr/bin/python3" "$APPDIR_PATH/usr/bin/python3"

# 애플리케이션 소스코드 및 빌드 결과물 복사
echo "  - 애플리케이션 파일 복사 중..."
docker cp "$CONTAINER_NAME:/root/ros2_ws" "$APPDIR_PATH/ros2_ws"
docker cp "$CONTAINER_NAME:/root/src/backend" "$APPDIR_PATH/backend"
docker cp "$CONTAINER_NAME:/root/src/ui/dist" "$APPDIR_PATH/dist"

echo "✔ 모든 파일 복사 완료!"

# ===================================================================
# 4단계: AppRun 스크립트 및 메타데이터 생성
# ===================================================================
echo "▶ (5/7) AppRun 스크립트 및 .desktop 파일 생성..."

# AppRun 스크립트 생성
cat << 'EOF' > "$APPDIR_PATH/AppRun"
#!/bin/bash
HERE=$(dirname $(readlink -f "${0}"))
export PATH=$HERE/usr/bin:$PATH
export LD_LIBRARY_PATH=$HERE/usr/lib:$LD_LIBRARY_PATH
# site-packages 경로를 동적으로 설정
export PYTHONPATH=$HERE/usr/lib/python/site-packages:$PYTHONPATH

# ROS 환경 설정
source $HERE/opt/ros/humble/setup.bash
source $HERE/ros2_ws/install/setup.bash

echo "Starting ROS2 AppImage..."

# ROS Launch 파일 실행 (백그라운드) - 실제 프로젝트에 맞게 수정하세요!
ros2 launch your_ros_pkg your_launch_file.py &
ROS_PID=$!

echo "ROS2 nodes launched. Starting backend server..."

# Python 웹서버 실행 (포어그라운드)
python3 -m backend.api.app

# 앱 종료 시 ROS 노드도 함께 종료
kill $ROS_PID
wait $ROS_PID
echo "Exiting."
EOF

# AppRun 스크립트에 실행 권한 부여
chmod +x "$APPDIR_PATH/AppRun"

# .desktop 파일 생성
cat << EOF > "$APPDIR_PATH/$APP_NAME.desktop"
[Desktop Entry]
Name=$APP_NAME
Exec=AppRun
Icon=app_icon
Type=Application
Categories=Utility;
EOF

# 아이콘 파일 복사 (프로젝트에 app_icon.png 파일이 있다고 가정)
# 아이콘 파일이 없다면 이 줄을 주석 처리하거나, 실제 아이콘 경로로 수정하세요.
if [ -f "app_icon.png" ]; then
    cp "app_icon.png" "$APPDIR_PATH/app_icon.png"
else
    echo "⚠️  경고: 아이콘 파일(app_icon.png)을 찾을 수 없습니다."
fi

echo "✔ 메타데이터 생성 완료!"

# ===================================================================
# 5단계: AppImage 빌드
# ===================================================================
echo "▶ (6/7) appimagetool을 사용하여 최종 AppImage 빌드..."
./appimagetool "$APPDIR_PATH"

echo "✔ AppImage 빌드 성공!"
echo "  결과물: ${APP_NAME}-x86_64.AppImage"

# ===================================================================
# 6단계: 최종 정리
# ===================================================================
echo "▶ (7/7) 임시 컨테이너 및 빌드 디렉토리 정리..."
docker rm -f "$CONTAINER_NAME"
# AppDir은 디버깅을 위해 남겨둘 수 있습니다. 원치 않으면 아래 줄의 주석을 해제하세요.
# rm -rf "$BUILD_DIR"

echo "🎉 모든 작업이 완료되었습니다!"