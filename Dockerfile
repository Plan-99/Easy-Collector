# ===================================================================
# ROS 2 Humble (Desktop-Full) on Ubuntu 22.04 Dockerfile
# ===================================================================

# 베이스 이미지 설정
FROM ubuntu:22.04

# 빌드 중 대화형 프롬프트 방지 및 환경 변수 설정
ENV DEBIAN_FRONTEND=noninteractive
ENV TZ=Asia/Seoul

# ROS 설치를 위한 준비 작업 및 필수 패키지 설치
RUN apt-get update && apt-get install -y --no-install-recommends \
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
# 기존 키 파일 URL이 더 이상 유효하지 않으므로, 키 서버에서 ID로 직접 키를 받아옵니다.
RUN curl -sSL "http://keyserver.ubuntu.com/pks/lookup?op=get&search=0xF6E65AC044F831AC80A06380C8B3A55A6F3EFCDE" | gpg --dearmor -o /usr/share/keyrings/intel-librealsense.gpg && \
    echo "deb [signed-by=/usr/share/keyrings/intel-librealsense.gpg] https://librealsense.intel.com/Debian/apt-repo $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/librealsense.list > /dev/null

# librealsense2-sdk 와 개발용 헤더 파일 설치
RUN apt-get update && apt-get install -y --no-install-recommends \
    librealsense2-utils \
    librealsense2-dev \
    librealsense2-dbg \
    && rm -rf /var/lib/apt/lists/*

# 네트워크 관련 도구 설치
RUN apt-get update && apt-get install -y --no-install-recommends ethtool can-utils iproute2 && rm -rf /var/lib/apt/lists/*

# ROS 2 Humble (Desktop-Full) 및 관련 패키지 설치
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-desktop-full \
    ros-humble-realsense2-camera \
    ros-humble-realsense2-description \
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
    python3-pip \
    python3-rosdep \
    python3-colcon-common-extensions \
    build-essential \
    && apt-get autoremove -y && apt-get clean -y && rm -rf /var/lib/apt/lists/*

# ROS 2 환경 설정
RUN echo "source /opt/ros/humble/setup.bash" >> /etc/bash.bashrc
RUN echo "source /root/ros2_ws/install/setup.bash" >> /etc/bash.bashrc

# NodeSource 저장소 설정 (Node.js 20 설치용)
RUN mkdir -p /etc/apt/keyrings && \
    curl -fsSL https://deb.nodesource.com/gpgkey/nodesource-repo.gpg.key | gpg --dearmor -o /etc/apt/keyrings/nodesource.gpg && \
    echo "deb [signed-by=/etc/apt/keyrings/nodesource.gpg] https://deb.nodesource.com/node_20.x nodistro main" | tee /etc/apt/sources.list.d/nodesource.list

# Node.js 설치 (npm도 함께 설치됨)
RUN apt-get update && apt-get install -y nodejs && rm -rf /var/lib/apt/lists/*

# GUI 관련 라이브러리
RUN apt-get update && apt-get install -y --no-install-recommends \
    libgl1-mesa-glx \
    libglib2.0-0 \
    libxext6 \
    libx11-6 \
    && rm -rf /var/lib/apt/lists/*

# pip 패키지 설치
COPY requirements.txt .
RUN pip install --no-cache-dir -r requirements.txt

# 컨테이너 시작 시 실행할 기본 명령어 설정
CMD ["/bin/bash"]