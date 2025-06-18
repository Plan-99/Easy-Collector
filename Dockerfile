# ===================================================================
# ROS Noetic (Desktop-Full) on Ubuntu 20.04 Dockerfile
# ===================================================================

# 1. 베이스 이미지 설정
# ROS Noetic은 Ubuntu 20.04 (Focal)을 공식 지원합니다.
FROM ubuntu:20.04

# 2. 빌드 중 대화형 프롬프트 방지 및 환경 변수 설정
# apt-get 설치 시 timezone 이나 키보드 레이아웃 등을 묻는 것을 방지합니다.
ENV DEBIAN_FRONTEND=noninteractive
ENV TZ=Asia/Seoul

# 3. ROS 설치를 위한 준비 작업
RUN apt-get update && apt-get install -y --no-install-recommends \
    curl \
    gnupg2 \
    lsb-release \
    # ROS 설치 후 정리할 때 필요한 패키지
    && apt-get autoremove -y && apt-get clean -y && rm -rf /var/lib/apt/lists/*

# 4. ROS 저장소(Repository) 추가 및 GPG 키 설정 (인증서 문제 해결)
RUN apt-get update && apt-get install -y --no-install-recommends \
    ca-certificates \
    curl \
    gnupg \
    lsb-release && \
    update-ca-certificates && \
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" | tee /etc/apt/sources.list.d/ros-latest.list > /dev/null

    # 5. ROS Noetic (Desktop-Full) 설치
#   - ros-noetic-desktop-full: 가장 일반적인 버전 (시뮬레이터, GUI 도구 포함)
#   - ros-noetic-ros-base: 최소 버전 (빌드, 통신 라이브러리만 포함)
#   - ros-noetic-ros-core: 가장 핵심적인 패키지만 포함
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-noetic-desktop-full \
    # 파이썬3 관련 필수 도구 설치
    python3-pip \
    python3-rosdep \
    python3-rosinstall \
    python3-rosinstall-generator \
    python3-wstool \
    build-essential \
    && apt-get autoremove -y && apt-get clean -y && rm -rf /var/lib/apt/lists/*

# 6. rosdep 초기화
# ROS 패키지의 시스템 의존성을 관리하는 도구입니다.
RUN rosdep init && \
    rosdep update

# 7. ROS 환경 설정
# 컨테이너의 bash 쉘이 시작될 때마다 ROS 환경을 자동으로 source 합니다.
RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc

# NodeSource 저장소 설정 (Node.js 20 설치용)
RUN mkdir -p /etc/apt/keyrings && \
    curl -fsSL https://deb.nodesource.com/gpgkey/nodesource-repo.gpg.key | gpg --dearmor -o /etc/apt/keyrings/nodesource.gpg && \
    echo "deb [signed-by=/etc/apt/keyrings/nodesource.gpg] https://deb.nodesource.com/node_20.x nodistro main" | tee /etc/apt/sources.list.d/nodesource.list

# Node.js 설치 (npm도 함께 설치됨)
RUN apt-get update && apt-get install nodejs -y

# # 8. (선택) Catkin 작업 공간(Workspace) 생성
# WORKDIR /root/catkin_ws
# RUN mkdir src
# RUN /bin/bash -c "source /opt/ros/noetic/setup.bash; catkin_init_workspace"


COPY requirements.txt .
RUN pip install -r requirements.txt

# 9. 컨테이너 시작 시 실행할 기본 명령어 설정
# 컨테이너가 실행되면 bash 쉘을 시작합니다.
CMD ["/bin/bash"]