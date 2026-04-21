# ROS 2 Humble: Robot/Sensor drivers only (no GPU)
FROM ros:humble

ENV DEBIAN_FRONTEND=noninteractive
ENV TZ=Asia/Seoul
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# Regional mirror
RUN sed -i 's/archive.ubuntu.com/mirror.kakao.com/g' /etc/apt/sources.list && \
    sed -i 's/security.ubuntu.com/mirror.kakao.com/g' /etc/apt/sources.list

# RealSense repository
RUN apt-key adv --keyserver keyserver.ubuntu.com --recv-keys FB0B24895113F120 && \
    echo "deb https://librealsense.intel.com/Debian/apt-repo $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/librealsense.list > /dev/null

# ROS 2 packages for robot/sensor drivers
RUN apt-get update && apt-get install -y --no-install-recommends \
    # Sensor packages
    ros-humble-realsense2-camera \
    ros-humble-realsense2-description \
    ros-humble-usb-cam \
    ros-humble-image-transport-plugins \
    # Robot control
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    ros-humble-controller-manager \
    ros-humble-ruckig \
    ros-humble-joint-state-publisher \
    ros-humble-robot-state-publisher \
    ros-humble-urdf \
    ros-humble-xacro \
    # Motion planning
    ros-humble-moveit \
    ros-humble-moveit-msgs \
    ros-humble-moveit-visual-tools \
    ros-humble-ompl \
    ros-humble-geometric-shapes \
    ros-humble-srdfdom \
    ros-humble-warehouse-ros \
    ros-humble-eigen-stl-containers \
    ros-humble-pybind11-vendor \
    ros-humble-pluginlib \
    # Communication
    ros-humble-rosbridge-server \
    ros-humble-rmw-cyclonedds-cpp \
    ros-humble-rosidl-generator-dds-idl \
    # Build tools
    python3-pip \
    python3-colcon-common-extensions \
    python3-pykdl \
    build-essential cmake \
    # Device access
    v4l-utils can-utils iproute2 ethtool \
    libyaml-cpp-dev \
    && apt-get autoremove -y && apt-get clean -y && rm -rf /var/lib/apt/lists/*

# Python packages for robot SDKs
RUN python3 -m pip install --no-cache-dir \
    setuptools==58.2.0 \
    dynamixel_sdk \
    piper_sdk \
    pyusb \
    python-can \
    pymodbus==3.9.2

WORKDIR /root

# Create empty workspace structure (modules installed at runtime via launcher)
RUN mkdir -p /root/ros2_ws/src

RUN echo "source /opt/ros/humble/setup.bash" >> /etc/bash.bashrc && \
    echo '[ -f /root/ros2_ws/install/setup.bash ] && source /root/ros2_ws/install/setup.bash' >> /etc/bash.bashrc

COPY --chmod=755 ros2/entrypoint.sh /usr/local/bin/entrypoint.sh

ENTRYPOINT ["/bin/bash", "/usr/local/bin/entrypoint.sh"]
