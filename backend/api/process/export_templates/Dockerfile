# ──────────────────────────────────────────────────────────────────────────────
# Isolated test container for an exported EasyTrainer planner bundle.
#
# Goal: run run_planner.py / ros_planner_service.py against the unpacked bundle
# WITHOUT depending on the main easytrainer_backend container — fresh ROS 2
# Humble + pip deps + GPU passthrough, nothing else.
#
# Host requirements:
#   - Docker + Docker Compose
#   - NVIDIA driver + nvidia-container-toolkit (for GPU inference; CPU works too)
#   - The robot/sensor ROS 2 topics already publishing somewhere reachable
#     (host network mode by default — same ROS_DOMAIN_ID as the host).
#
# Build & run via docker-compose.yml in this same folder.
# ──────────────────────────────────────────────────────────────────────────────
FROM osrf/ros:humble-desktop

ENV DEBIAN_FRONTEND=noninteractive \
    PYTHONUNBUFFERED=1 \
    PIP_DISABLE_PIP_VERSION_CHECK=1 \
    PIP_NO_CACHE_DIR=1

# System deps:
#   - python3-pip:           install bundle requirements
#   - libgl1 libglib2.0-0:   opencv-python-headless runtime
#   - ffmpeg:                PyAV / lerobot video I/O
#   - curl + ca-certificates + lsb-release: needed for the robotpkg apt repo
#   - ros-humble-trajectory-msgs: needed when robots use JointTrajectory output
#   - ros-humble-rmw-cyclonedds-cpp: optional, more reliable DDS than the default
RUN apt-get update && apt-get install -y --no-install-recommends \
        python3-pip \
        libgl1 \
        libglib2.0-0 \
        ffmpeg \
        curl \
        ca-certificates \
        lsb-release \
        ros-humble-trajectory-msgs \
        ros-humble-rmw-cyclonedds-cpp \
    && rm -rf /var/lib/apt/lists/*

# Pinocchio (Python bindings) for the Common_ArmIK solver used by
# end_effector_position query_pose blocks. We install via the robotpkg apt
# repo — same approach the in-container EasyTrainer ros2 image uses — so the
# C++ runtime + Python bindings stay in sync. pin-pink (the QP-based pink
# wrapper) is installed via pip in requirements.txt.
RUN mkdir -p /etc/apt/keyrings && \
    curl -sSL http://robotpkg.openrobots.org/packages/debian/robotpkg.asc \
        | tee /etc/apt/keyrings/robotpkg.asc > /dev/null && \
    echo "deb [arch=amd64 signed-by=/etc/apt/keyrings/robotpkg.asc] http://robotpkg.openrobots.org/packages/debian/pub $(lsb_release -cs) robotpkg" \
        | tee /etc/apt/sources.list.d/robotpkg.list && \
    apt-get update && apt-get install -y --no-install-recommends \
        robotpkg-simde=0.8.0 \
        robotpkg-py310-pinocchio \
    && rm -rf /var/lib/apt/lists/*

ENV PATH="/opt/openrobots/bin:${PATH}" \
    PYTHONPATH="/opt/openrobots/lib/python3.10/site-packages:${PYTHONPATH}" \
    LD_LIBRARY_PATH="/opt/openrobots/lib:${LD_LIBRARY_PATH}"

WORKDIR /workspace

# Install Python deps from the bundle's requirements.txt up front so the image
# layer caches across runs. The full bundle is mounted at /workspace at run
# time (see docker-compose.yml), so we only copy requirements.txt for the
# build.
COPY requirements.txt /tmp/requirements.txt
RUN pip3 install --no-cache-dir -r /tmp/requirements.txt

# Auto-source ROS 2 in every interactive shell. The bundle's run_planner.py
# already manipulates sys.path so the vendored lerobot wins over any
# pip-installed one — no further setup needed.
RUN echo 'source /opt/ros/humble/setup.bash' >> /root/.bashrc \
 && echo 'echo "[et-test] ROS 2 Humble sourced. Bundle mounted at /workspace."' >> /root/.bashrc \
 && echo 'echo "[et-test] Try: python3 run_planner.py --help"' >> /root/.bashrc

CMD ["/bin/bash"]
