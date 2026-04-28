FROM moveit/moveit2:main-humble-tutorial-source

RUN sed -i 's/archive.ubuntu.com/mirror.kakao.com/g' /etc/apt/sources.list && \
    sed -i 's/security.ubuntu.com/mirror.kakao.com/g' /etc/apt/sources.list

# ROS2 GPG 키 갱신 + CycloneDDS 설치
RUN rm -f /usr/share/keyrings/ros2-latest-archive-keyring.gpg && \
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc \
      | gpg --batch --dearmor -o /usr/share/keyrings/ros2-latest-archive-keyring.gpg && \
    apt-get update && \
    apt-get install -y ros-humble-rmw-cyclonedds-cpp && \
    rm -rf /var/lib/apt/lists/*

# ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp


# bash 시작 시 자동으로 ROS2 환경 로드
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc && \
    echo "source /root/ws_moveit/install/setup.bash 2>/dev/null" >> /root/.bashrc && \
    echo "source /root/ws/install/setup.bash 2>/dev/null" >> /root/.bashrc
