#!/bin/bash

# 스크립트 실행 중 에러 발생 시 즉시 중단 (안전 장치)
set -e

PACKAGES=("rbpodo")

# sudo 권한 캐시 갱신
sudo -v

# 현재 스크립트 실행 위치 저장
START_DIR=$(pwd)

for pkg in "${PACKAGES[@]}"; do
    echo -e "\n\033[1;32m>>> Building Package: $pkg \033[0m" # 녹색 텍스트로 강조

    # 소스 디렉토리 확인
    SOURCE_DIR="$START_DIR/$BASE_DIR/$pkg"
    if [ ! -d "$SOURCE_DIR" ]; then
        echo "Error: $pkg 폴더를 찾을 수 없습니다: $SOURCE_DIR"
        exit 1
    fi

    # pushd: 현재 위치 기억하고 이동
    mkdir -p "$SOURCE_DIR/build"
    pushd "$SOURCE_DIR/build" > /dev/null

    # CMake & Make & Install
    cmake -DCMAKE_BUILD_TYPE=Release ..
    make -j$(nproc)  # CPU 코어 수만큼 병렬 빌드
    sudo make install

    # popd: 아까 기억한 위치(스크립트 시작 위치)로 복귀
    popd > /dev/null
done

echo -e "\n\033[1;32m>>> All packages built and installed successfully! \033[0m"