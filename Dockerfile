# ===================================================================
# ROS 2 Humble (Desktop-Full) on Ubuntu 22.04 Dockerfile
# with tmux for multi-terminal support
# ===================================================================

# 베이스 이미지 설정 (GPU 지원을 위해 NVIDIA CUDA 런타임 사용)
FROM nvidia/cuda:12.4.1-devel-ubuntu22.04

# 빌드 중 대화형 프롬프트 방지 및 환경 변수 설정
ENV DEBIAN_FRONTEND=noninteractive
ENV TZ=Asia/Seoul
ENV PIP_NO_CACHE_DIR=1
# Make pip more resilient to slow networks
ENV PIP_DEFAULT_TIMEOUT=300
ENV PIP_PROGRESS_BAR=off
# NVIDIA Container Toolkit와 함께 사용할 때 GPU를 항상 노출
ENV NVIDIA_VISIBLE_DEVICES=all
ENV NVIDIA_DRIVER_CAPABILITIES=compute,utility,video

# Regional PyPI mirror (defaults to Kakao; can be overridden at build time)
ARG PIP_INDEX_URL=https://mirror.kakao.com/pypi/simple
ARG PIP_EXTRA_INDEX_URL=https://pypi.org/simple
ENV PIP_INDEX_URL=${PIP_INDEX_URL}
ENV PIP_EXTRA_INDEX_URL=${PIP_EXTRA_INDEX_URL}
RUN printf "[global]\nindex-url = %s\nextra-index-url = %s\n" "$PIP_INDEX_URL" "$PIP_EXTRA_INDEX_URL" > /etc/pip.conf


# ROS 설치를 위한 준비 작업 및 필수 패키지 설치
RUN \
    # 1. 패키지 저장소를 archive.ubuntu.com에서 mirror.kakao.com으로 변경
    sed -i 's/archive.ubuntu.com/mirror.kakao.com/g' /etc/apt/sources.list && \
    \
    apt-get update && apt-get install -y --no-install-recommends \
    apt-utils \
    curl \
    gnupg \
    lsb-release \
    ca-certificates \
    software-properties-common \
    apt-transport-https \
    && rm -rf /var/lib/apt/lists/*

# ROS 2 저장소(Repository) 추가 및 GPG 키 설정
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -sc) main" | tee /etc/apt/sources.list.d/ros2-latest.list > /dev/null

# Intel RealSense 저장소 및 GPG 키 등록 (수정된 방식)
RUN curl -sSL "http://keyserver.ubuntu.com/pks/lookup?op=get&search=0xF6E65AC044F831AC80A06380C8B3A55A6F3EFCDE" | gpg --dearmor -o /usr/share/keyrings/intel-librealsense.gpg && \
    echo "deb [signed-by=/usr/share/keyrings/intel-librealsense.gpg] https://librealsense.intel.com/Debian/apt-repo $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/librealsense.list > /dev/null

RUN apt-get update && apt-get install -y --no-install-recommends \
    libyaml-cpp-dev \
    && rm -rf /var/lib/apt/lists/*

# 네트워크 관련 도구 설치
RUN apt-get update && apt-get install -y --no-install-recommends ethtool can-utils iproute2 && rm -rf /var/lib/apt/lists/*

# ROS 2 Humble (Desktop-Full) 및 관련 패키지 설치
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-desktop-full \
    ros-humble-realsense2-camera \
    ros-humble-realsense2-description \
    ros-humble-usb-cam \
    ros-humble-image-transport-plugins \
    v4l-utils \
    ros-humble-ruckig \
    ros-humble-eigen-stl-containers \
    ros-humble-geometric-shapes \
    ros-humble-pybind11-vendor \
    ros-humble-moveit-resources-panda-moveit-config \
    ros-humble-ompl \
    ros-humble-warehouse-ros \
    ros-humble-eigenpy \
    ros-humble-moveit-msgs \
    ros-humble-srdfdom \
    ros-humble-rosbridge-server \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    ros-humble-controller-manager \
    ros-humble-rmw-cyclonedds-cpp \
    ros-humble-rosidl-generator-dds-idl \
    ros-humble-ament-cmake \
    ros-humble-joint-state-publisher \
    ros-humble-moveit \
    ros-humble-pluginlib \
    ros-humble-robot-state-publisher \
    ros-humble-urdf-launch \
    ros-humble-xacro \
    python3-pip \
    python3-rosdep \
    python3-colcon-common-extensions \
    python3-pykdl \
    build-essential \
    cmake \
    && apt-get autoremove -y && apt-get clean -y && rm -rf /var/lib/apt/lists/*

# ROS 2 환경 설정
RUN echo "source /opt/ros/humble/setup.bash" >> /etc/bash.bashrc
# Note: The following line is for interactive shells. The entrypoint script will source the workspace setup manually.
RUN echo "source /root/ros2_ws/install/setup.bash" >> /etc/bash.bashrc

# NodeSource 저장소 설정 (Node.js 20 설치용)
RUN mkdir -p /etc/apt/keyrings && \
    curl -fsSL https://deb.nodesource.com/gpgkey/nodesource-repo.gpg.key | gpg --dearmor -o /etc/apt/keyrings/nodesource.gpg && \
    echo "deb [signed-by=/etc/apt/keyrings/nodesource.gpg] https://deb.nodesource.com/node_20.x nodistro main" | tee /etc/apt/sources.list.d/nodesource.list

RUN apt-get update && \
    # 2. 인증서 등록
    mkdir -p /etc/apt/keyrings && \
    curl -sSL http://robotpkg.openrobots.org/packages/debian/robotpkg.asc \
        | tee /etc/apt/keyrings/robotpkg.asc > /dev/null && \
    \
    # 3. 소스 리포지토리 추가
    echo "deb [arch=amd64 signed-by=/etc/apt/keyrings/robotpkg.asc] http://robotpkg.openrobots.org/packages/debian/pub $(lsb_release -cs) robotpkg" \
        | tee /etc/apt/sources.list.d/robotpkg.list && \
    \
    apt-get update && \
    apt-get install -y --no-install-recommends robotpkg-py3*-pinocchio && \
    \
    rm -rf /var/lib/apt/lists/* && \
    apt-get clean


ENV PATH="/opt/openrobots/bin:${PATH}"
ENV PKG_CONFIG_PATH="/opt/openrobots/lib/pkgconfig:${PKG_CONFIG_PATH}"
ENV LD_LIBRARY_PATH="/opt/openrobots/lib:${LD_LIBRARY_PATH}"
ENV PYTHONPATH="/opt/openrobots/lib/python3.10/site-packages:${PYTHONPATH}"
ENV CMAKE_PREFIX_PATH="/opt/openrobots:${CMAKE_PREFIX_PATH}"


# pip 패키지를 먼저 설치하여 충돌 시 빠르게 실패하도록 배치
COPY requirements.txt .
# RTX 5090(sm_100) 및 하위 호환성을 위해 CUDA 12.4용 최신 PyTorch 설치
RUN python3 -m pip install --no-cache-dir --timeout 300 --prefer-binary \
      --extra-index-url https://download.pytorch.org/whl/cu124 \
      torch torchvision torchaudio \
    && python3 -m pip install --no-cache-dir --timeout 300 --prefer-binary -r requirements.txt \
    && python3 -m pip install --no-cache-dir --timeout 300 --prefer-binary "pyyaml==6.0.2" \
    && rm -rf /root/.cache/pip

# Node.js 설치 (npm도 함께 설치됨)
RUN apt-get update && apt-get install -y nodejs && rm -rf /var/lib/apt/lists/*

# GUI 관련 라이브러리 및 tmux 설치
RUN apt-get update && apt-get install -y --no-install-recommends \
    libgl1-mesa-glx \
    libglib2.0-0 \
    libxext6 \
    libx11-6 \
    tmux \
    && rm -rf /var/lib/apt/lists/*

# Git 설치
RUN apt-get update && apt-get install -y --no-install-recommends git && rm -rf /var/lib/apt/lists/*

# --- ROS2 Workspace & UI Setup ---
WORKDIR /root
# src 디렉토리 전체를 복사하여 ui와 backend를 모두 포함하도록 수정
COPY src /root/src
# Defer UI dependency install to runtime for resilience; start_services.sh handles npm ci/install


# ===================================================================
# (추가됨) Custom Python Packages Setup
# python_pkgs/install.sh 스크립트의 로직을 Docker RUN 명령어로 변환
# ===================================================================

# python_pkgs 폴더 복사!
COPY python_pkgs /root/python_pkgs

# 0. pip 업그레이드
# NOTE: setuptools는 ROS2 Humble의 ament/colcon (setup.py develop, script_dir 등)과
#       호환성 문제가 생길 수 있어 여기서 업그레이드하지 않습니다.
RUN python3 -m pip install --no-cache-dir --timeout 300 --upgrade pip \
    && rm -rf /root/.cache/pip

# 1. Dex Retargeting 설치
WORKDIR /root/python_pkgs/xr_teleoperate/teleop/robot_control/dex-retargeting
RUN python3 -m pip install --no-cache-dir --timeout 300 -e . \
    && rm -rf /root/.cache/pip

# 2. Televuer 설치 및 SSL 인증서 생성 
WORKDIR /root/python_pkgs/xr_teleoperate/teleop/televuer
RUN python3 -m pip install --no-cache-dir --timeout 300 -e . && \
    openssl req -x509 -nodes -days 365 -newkey rsa:2048 \
    -keyout key.pem -out cert.pem \
    -subj "/C=KR/ST=Seoul/L=Seoul/O=Robot/CN=localhost" \
    -addext "subjectAltName=DNS:localhost" && rm -rf /root/.cache/pip

# 3. Unitree SDK 설치
WORKDIR /root/python_pkgs/unitree_sdk2_python
RUN python3 -m pip install --no-cache-dir --timeout 300 -e . \
    && rm -rf /root/.cache/pip

# Build C++ dependencies shipped in cmake_pkgs (e.g., rbpodo)
COPY cmake_pkgs /root/cmake_pkgs
WORKDIR /root/cmake_pkgs/rbpodo
RUN mkdir -p build && cd build && cmake -DCMAKE_BUILD_TYPE=Release .. && make -j"$(nproc)" && make install

# Build ROS 2 workspace
COPY ros2_ws /root/ros2_ws
WORKDIR /root/ros2_ws
RUN bash -c "source /opt/ros/humble/setup.bash && colcon build --symlink-install"


# 작업 디렉토리 원상 복구
WORKDIR /root

# ===================================================================

# Helper to launch both backend and frontend in one container
COPY start_services.sh /usr/local/bin/start_services.sh
RUN chmod +x /usr/local/bin/start_services.sh
