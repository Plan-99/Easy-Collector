# cuRobo + ROS2 Humble Docker
#
# 베이스: NVIDIA CUDA (시스템 Python 3.10만 사용, conda 없음)
# 추가: PyTorch + ROS2 Humble + cuRobo
#
# 빌드:
#   cd curobo && docker build -t curobo-ros2:latest .
#
# 실행:
#   ./docker_start.sh

FROM nvidia/cuda:12.4.0-devel-ubuntu22.04

ENV DEBIAN_FRONTEND=noninteractive
ENV LANG=C.UTF-8
ENV LC_ALL=C.UTF-8

# 미러 설정
RUN sed -i 's/archive.ubuntu.com/mirror.kakao.com/g' /etc/apt/sources.list && \
    sed -i 's/security.ubuntu.com/mirror.kakao.com/g' /etc/apt/sources.list

# ── 기본 패키지 + Python 3.10 ────────────────────────────
RUN apt-get update && apt-get install -y \
      python3 python3-pip python3-dev \
      software-properties-common curl gnupg lsb-release \
      git build-essential && \
    ln -sf /usr/bin/python3 /usr/bin/python && \
    pip3 install --no-cache-dir --upgrade pip && \
    rm -rf /var/lib/apt/lists/*

# ── PyTorch 설치 (CUDA 12.4 호환 버전 고정) ──────────────
RUN pip install --no-cache-dir \
    torch==2.5.1 torchvision==0.20.1 --index-url https://download.pytorch.org/whl/cu124

# ── ROS2 Humble 설치 ────────────────────────────────────
RUN apt-get update && \
    add-apt-repository universe && \
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc \
      | gpg --batch --dearmor -o /usr/share/keyrings/ros2-latest-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros2-latest-archive-keyring.gpg] \
      http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
      > /etc/apt/sources.list.d/ros2.list && \
    apt-get update && \
    apt-get install -y \
      ros-humble-ros-base \
      ros-humble-rmw-cyclonedds-cpp \
      ros-humble-sensor-msgs \
      ros-humble-geometry-msgs \
      ros-humble-std-srvs \
      ros-humble-trajectory-msgs \
      ros-humble-control-msgs \
      ros-humble-tf2-ros \
      ros-humble-visualization-msgs \
      python3-colcon-common-extensions \
      python3-rosdep && \
    rm -rf /var/lib/apt/lists/*

# rosdep 초기화
RUN rosdep init 2>/dev/null || true && rosdep update

# ── cuRobo 설치 (GitHub 소스에서 빌드) ────────────────────
# --no-build-isolation: 이미 설치된 PyTorch를 사용 (재설치 방지)
# torch를 제외한 의존성만 먼저 설치
RUN pip install --no-cache-dir \
    numpy-quaternion trimesh networkx warp-lang yourdfpy scipy

ENV TORCH_CUDA_ARCH_LIST="8.6"
RUN git clone https://github.com/NVlabs/curobo.git /tmp/curobo && \
    cd /tmp/curobo && \
    pip install --no-cache-dir --no-build-isolation --no-deps . && \
    rm -rf /tmp/curobo

# ── 환경 설정 ────────────────────────────────────────────
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc && \
    echo "source /root/ws/install/setup.bash 2>/dev/null" >> /root/.bashrc

WORKDIR /root/ws
