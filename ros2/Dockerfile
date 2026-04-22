# ROS 2 Humble + IK Solver + gRPC Bridge + WebRTC Streaming
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

# ROS 2 packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    # Sensor
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

# Pinocchio + CasADi (IK solver dependencies)
RUN mkdir -p /etc/apt/keyrings && \
    curl -sSL http://robotpkg.openrobots.org/packages/debian/robotpkg.asc | tee /etc/apt/keyrings/robotpkg.asc > /dev/null && \
    echo "deb [arch=amd64 signed-by=/etc/apt/keyrings/robotpkg.asc] http://robotpkg.openrobots.org/packages/debian/pub $(lsb_release -cs) robotpkg" | tee /etc/apt/sources.list.d/robotpkg.list && \
    apt-get update && apt-get install -y --no-install-recommends \
    robotpkg-simde=0.8.0 \
    robotpkg-py310-pinocchio \
    robotpkg-py310-casadi && \
    rm -rf /var/lib/apt/lists/*

ENV PATH="/opt/openrobots/bin:${PATH}"
ENV PYTHONPATH="/opt/openrobots/lib/python3.10/site-packages:${PYTHONPATH}"
ENV LD_LIBRARY_PATH="/opt/openrobots/lib:${LD_LIBRARY_PATH}"

# Python packages for ROS2 bridge
COPY ros2/requirements.ros2.txt /tmp/requirements.ros2.txt
RUN python3 -m pip install --no-cache-dir -r /tmp/requirements.ros2.txt && \
    rm /tmp/requirements.ros2.txt

WORKDIR /root

# Create empty workspace (modules installed at runtime via launcher)
RUN mkdir -p /root/ros2_ws/src

# Bundle ros2_bridge into image
COPY ros2/ros2_bridge /root/ros2/ros2_bridge

RUN echo "source /opt/ros/humble/setup.bash" >> /etc/bash.bashrc && \
    echo '[ -f /root/ros2_ws/install/setup.bash ] && source /root/ros2_ws/install/setup.bash' >> /etc/bash.bashrc

COPY ros2/entrypoint.sh /usr/local/bin/entrypoint.sh
COPY ros2/start_ros2_services.sh /usr/local/bin/start_ros2_services.sh
RUN chmod +x /usr/local/bin/entrypoint.sh /usr/local/bin/start_ros2_services.sh

EXPOSE 50051

ENTRYPOINT ["/bin/bash", "/usr/local/bin/start_ros2_services.sh"]
