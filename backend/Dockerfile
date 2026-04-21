# Backend: Flask API + PyTorch + CUDA (no ROS 2)
FROM nvidia/cuda:12.4.1-devel-ubuntu22.04

ENV DEBIAN_FRONTEND=noninteractive
ENV TZ=Asia/Seoul
ENV PIP_NO_CACHE_DIR=1
ENV PIP_DEFAULT_TIMEOUT=300
ENV EC_SKIP_TORCHVISION_COMPAT=1
ENV NVIDIA_VISIBLE_DEVICES=all
ENV NVIDIA_DRIVER_CAPABILITIES=compute,utility,video

# Regional PyPI mirror
ARG PIP_INDEX_URL=https://mirror.kakao.com/pypi/simple
ARG PIP_EXTRA_INDEX_URL=https://pypi.org/simple
ENV PIP_INDEX_URL=${PIP_INDEX_URL}
ENV PIP_EXTRA_INDEX_URL=${PIP_EXTRA_INDEX_URL}
RUN printf "[global]\nindex-url = %s\nextra-index-url = %s\n" "$PIP_INDEX_URL" "$PIP_EXTRA_INDEX_URL" > /etc/pip.conf

RUN mkdir -p /opt/easytrainer/logs /tmp/easytrainer/logs && chmod -R 777 /opt/easytrainer /tmp/easytrainer

# System packages (no ROS 2)
RUN sed -i 's/archive.ubuntu.com/mirror.kakao.com/g' /etc/apt/sources.list && \
    sed -i 's/security.ubuntu.com/mirror.kakao.com/g' /etc/apt/sources.list && \
    apt-get update && apt-get install -y --no-install-recommends \
    apt-utils curl gnupg lsb-release ca-certificates software-properties-common \
    python3-pip python3-dev build-essential cmake \
    v4l-utils libyaml-cpp-dev \
    # GStreamer for video processing
    gstreamer1.0-tools gstreamer1.0-libav \
    libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev \
    libgstreamer-plugins-good1.0-dev \
    gstreamer1.0-plugins-good gstreamer1.0-plugins-base \
    # OpenCV system
    python3-opencv \
    && apt-get autoremove -y && apt-get clean -y && rm -rf /var/lib/apt/lists/*

# Pinocchio (IK/trajectory planning)
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

# Python packages
WORKDIR /root
RUN python3 -m pip install --no-cache-dir --upgrade pip setuptools==58.2.0

COPY requirements.txt .
RUN python3 -m pip install --no-cache-dir --ignore-installed -r requirements.txt

# # Custom Python packages
# COPY python_pkgs /root/python_pkgs
# RUN python3 -m pip install --no-cache-dir -e /root/python_pkgs/xr_teleoperate/teleop/robot_control/dex-retargeting && \
#     python3 -m pip install --no-cache-dir -e /root/python_pkgs/xr_teleoperate/teleop/televuer && \
#     python3 -m pip install --no-cache-dir -e /root/python_pkgs/unitree_sdk2_python

# Fix matplotlib + numpy
RUN python3 -m pip install --no-cache-dir --ignore-installed \
    matplotlib==3.8.2 pyyaml==6.0.2 setuptools==58.2.0

# # C++ dependencies (rbpodo)
# COPY cmake_pkgs /root/cmake_pkgs
# WORKDIR /root/cmake_pkgs/rbpodo
# RUN mkdir -p build && cd build && cmake -DCMAKE_BUILD_TYPE=Release .. && make -j"$(nproc)" && make install

# PyTorch (CUDA 12.4)
RUN python3 -m pip install --no-cache-dir --ignore-installed \
    --index-url https://download.pytorch.org/whl/nightly/cu124 \
    torch torchvision torchaudio

# Fix numpy
RUN rm -rf /usr/local/lib/python3.10/dist-packages/numpy* && \
    rm -rf /usr/lib/python3/dist-packages/numpy* && \
    python3 -m pip install --no-cache-dir numpy==1.26.4

# Bundle source into image (overridden by volume mount in dev)
COPY backend /root/backend

WORKDIR /root
RUN chmod -R 777 /opt/easytrainer /tmp/easytrainer

COPY --chmod=755 backend/entrypoint.sh /usr/local/bin/entrypoint.sh

EXPOSE 5000
ENTRYPOINT ["/bin/bash", "/usr/local/bin/entrypoint.sh"]
