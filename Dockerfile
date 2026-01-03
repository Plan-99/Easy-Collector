# ===================================================================
# ROS 2 Humble (Desktop-Full) on Ubuntu 22.04 Dockerfile
# Optimized for RTX 5090 & Easy-Trainer Project (Fix: Colcon Build Error)
# ===================================================================

# 1. 베이스 이미지 설정
FROM nvidia/cuda:12.4.1-devel-ubuntu22.04

# 환경 변수 설정
ENV DEBIAN_FRONTEND=noninteractive
ENV TZ=Asia/Seoul
ENV PIP_NO_CACHE_DIR=1
ENV PIP_DEFAULT_TIMEOUT=300

# RTX 5090 호환성 및 자동 다운그레이드 방지
ENV EC_SKIP_TORCHVISION_COMPAT=1
ENV NVIDIA_VISIBLE_DEVICES=all
ENV NVIDIA_DRIVER_CAPABILITIES=compute,utility,video

# Regional PyPI mirror (Kakao)
ARG PIP_INDEX_URL=https://mirror.kakao.com/pypi/simple
ARG PIP_EXTRA_INDEX_URL=https://pypi.org/simple
ENV PIP_INDEX_URL=${PIP_INDEX_URL}
ENV PIP_EXTRA_INDEX_URL=${PIP_EXTRA_INDEX_URL}
RUN printf "[global]\nindex-url = %s\nextra-index-url = %s\n" "$PIP_INDEX_URL" "$PIP_EXTRA_INDEX_URL" > /etc/pip.conf

# 권한 문제 해결을 위한 사전 설정
RUN mkdir -p /opt/easytrainer/logs /tmp/easytrainer/logs && chmod -R 777 /opt/easytrainer /tmp/easytrainer

# 2. 필수 시스템 패키지 설치
RUN sed -i 's/archive.ubuntu.com/mirror.kakao.com/g' /etc/apt/sources.list && \
    apt-get update && apt-get install -y --no-install-recommends \
    apt-utils curl gnupg lsb-release ca-certificates software-properties-common apt-transport-https \
    && rm -rf /var/lib/apt/lists/*

# 3. ROS 2 및 RealSense 저장소 설정
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -sc) main" | tee /etc/apt/sources.list.d/ros2-latest.list > /dev/null

RUN curl -sSL "http://keyserver.ubuntu.com/pks/lookup?op=get&search=0xF6E65AC044F831AC80A06380C8B3A55A6F3EFCDE" | gpg --dearmor -o /usr/share/keyrings/intel-librealsense.gpg && \
    echo "deb [signed-by=/usr/share/keyrings/intel-librealsense.gpg] https://librealsense.intel.com/Debian/apt-repo $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/librealsense.list > /dev/null

# 4. ROS 2 Humble 및 관련 종속성 설치
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-desktop-full \
    ros-humble-realsense2-camera \
    ros-humble-realsense2-description \
    ros-humble-usb-cam \
    ros-humble-image-transport-plugins \
    v4l-utils ethtool can-utils iproute2 libyaml-cpp-dev \
    ros-humble-moveit \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    python3-pip python3-rosdep python3-colcon-common-extensions \
    build-essential cmake git tmux nodejs \
    && apt-get autoremove -y && apt-get clean -y && rm -rf /var/lib/apt/lists/*

# Node.js 최신 LTS 버전 설치
RUN curl -fsSL https://deb.nodesource.com/setup_20.x | bash - && \
    apt-get install -y nodejs

# 5. RobotPkg (Pinocchio) 설치
RUN mkdir -p /etc/apt/keyrings && \
    curl -sSL http://robotpkg.openrobots.org/packages/debian/robotpkg.asc | tee /etc/apt/keyrings/robotpkg.asc > /dev/null && \
    echo "deb [arch=amd64 signed-by=/etc/apt/keyrings/robotpkg.asc] http://robotpkg.openrobots.org/packages/debian/pub $(lsb_release -cs) robotpkg" | tee /etc/apt/sources.list.d/robotpkg.list && \
    apt-get update && apt-get install -y --no-install-recommends robotpkg-py310-pinocchio && \
    rm -rf /var/lib/apt/lists/*

ENV PATH="/opt/openrobots/bin:${PATH}"
ENV PYTHONPATH="/opt/openrobots/lib/python3.10/site-packages:${PYTHONPATH}"

# --- Python 패키지 설치 섹션 ---
WORKDIR /root
# [1] 기본 툴 업데이트 및 setuptools 고정
RUN python3 -m pip install --no-cache-dir --upgrade pip setuptools==58.2.0

# [2] 일반 requirements 및 커스텀 패키지를 먼저 설치 (이때 torch가 꼬여도 상관없음)
COPY requirements.txt .
RUN python3 -m pip install --no-cache-dir --ignore-installed -r requirements.txt

COPY python_pkgs /root/python_pkgs
# 커스텀 패키지들 설치 (각 디렉토리에서 -e . 실행)
RUN python3 -m pip install --no-cache-dir -e /root/python_pkgs/xr_teleoperate/teleop/robot_control/dex-retargeting && \
    python3 -m pip install --no-cache-dir -e /root/python_pkgs/xr_teleoperate/teleop/televuer && \
    python3 -m pip install --no-cache-dir -e /root/python_pkgs/unitree_sdk2_python

# # 3. ★ 최종 해결책: 모든 패키지 설치 완료 후 '마지막'에 Torch 환경 정화 ★
# # 기존에 꼬인 torch 계열을 완전히 지우고, Blackwell(cu124)용으로 강제 고정합니다.
# RUN python3 -m pip uninstall -y torch torchvision torchaudio && \
#     python3 -m pip install --no-cache-dir --ignore-installed \
#     --index-url https://download.pytorch.org/whl/cu124 \
#     torch==2.5.1 torchvision==0.20.1 torchaudio==2.5.1

# 4. Matplotlib 및 툴 버전 최종 보정 (Axes3D 에러 방지)
RUN python3 -m pip install --no-cache-dir --ignore-installed \
    matplotlib==3.8.2 \
    pyyaml==6.0.2 \
    setuptools==58.2.0


# --- 워크스페이스 및 빌드 설정 ---
WORKDIR /root
COPY src /root/src
COPY cmake_pkgs /root/cmake_pkgs

# C++ 종속성 빌드 (rbpodo)
WORKDIR /root/cmake_pkgs/rbpodo
RUN mkdir -p build && cd build && cmake -DCMAKE_BUILD_TYPE=Release .. && make -j"$(nproc)" && make install

# # ROS 2 워크스페이스 빌드
# RUN python3 -m pip install --no-cache-dir --ignore-installed setuptools==58.2.0

COPY ros2_ws /root/ros2_ws
WORKDIR /root/ros2_ws
# setuptools==58.2.0 덕분에 --symlink-install 에러가 해결됩니다.
RUN python3 -m pip uninstall -y setuptools && \
    python3 -m pip install --no-cache-dir setuptools==58.2.0

# [추가] colcon 빌드 시 setuptools 버전을 체크하는 로직을 우회하기 위한 환경 변수
ENV PYTHONWARNINGS="ignore:setup.py install is deprecated"

RUN bash -c "source /opt/ros/humble/setup.bash && \
    colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release"

RUN python3 -m pip install --no-cache-dir --ignore-installed \
    --index-url https://download.pytorch.org/whl/nightly/cu124 \
    torch torchvision torchaudio
    
# 최종 환경 설정
WORKDIR /root
RUN echo "source /opt/ros/humble/setup.bash" >> /etc/bash.bashrc && \
    echo "source /root/ros2_ws/install/setup.bash" >> /etc/bash.bashrc

# COPY start_services.sh /usr/local/bin/start_services.sh
# RUN chmod +x /usr/local/bin/start_services.sh
COPY --chmod=755 start_services.sh /usr/local/bin/start_services.sh
RUN chmod -R 777 /opt/easytrainer /tmp/easytrainer

ENTRYPOINT ["/bin/bash", "/usr/local/bin/start_services.sh"]