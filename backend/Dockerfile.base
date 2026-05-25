# Backend: Flask API + PyTorch + CUDA (no ROS 2, no IK solver)
FROM nvidia/cuda:12.6.3-devel-ubuntu24.04

ENV DEBIAN_FRONTEND=noninteractive
ENV TZ=Asia/Seoul
ENV PIP_NO_CACHE_DIR=1
ENV PIP_DEFAULT_TIMEOUT=300
ENV NVIDIA_VISIBLE_DEVICES=all
ENV NVIDIA_DRIVER_CAPABILITIES=compute,utility,video

# Regional PyPI mirror
ARG PIP_INDEX_URL=https://mirror.kakao.com/pypi/simple
ARG PIP_EXTRA_INDEX_URL=https://pypi.org/simple
ENV PIP_INDEX_URL=${PIP_INDEX_URL}
ENV PIP_EXTRA_INDEX_URL=${PIP_EXTRA_INDEX_URL}
RUN printf "[global]\nindex-url = %s\nextra-index-url = %s\n" "$PIP_INDEX_URL" "$PIP_EXTRA_INDEX_URL" > /etc/pip.conf

RUN mkdir -p /opt/easytrainer/logs /tmp/easytrainer/logs && chmod -R 777 /opt/easytrainer /tmp/easytrainer

# System packages (Ubuntu 24.04)
RUN sed -i 's|http://archive.ubuntu.com|http://mirror.kakao.com|g' /etc/apt/sources.list.d/ubuntu.sources 2>/dev/null || true && \
    sed -i 's|http://security.ubuntu.com|http://mirror.kakao.com|g' /etc/apt/sources.list.d/ubuntu.sources 2>/dev/null || true && \
    apt-get update && apt-get install -y --no-install-recommends \
    apt-utils curl gnupg ca-certificates software-properties-common \
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

# Python packages
WORKDIR /root
RUN python3 -m pip install --no-cache-dir --break-system-packages --ignore-installed setuptools

COPY requirements.txt .
RUN python3 -m pip install --no-cache-dir --break-system-packages --ignore-installed -r requirements.txt

# PyTorch (CUDA 12.6)
RUN python3 -m pip install --no-cache-dir --break-system-packages \
    --index-url https://download.pytorch.org/whl/nightly/cu126 \
    torch torchvision torchaudio

# Bundle source
COPY backend /root/backend

WORKDIR /root
RUN chmod -R 777 /opt/easytrainer /tmp/easytrainer

COPY backend/entrypoint.sh /usr/local/bin/entrypoint.sh
RUN chmod +x /usr/local/bin/entrypoint.sh

EXPOSE 5000
ENTRYPOINT ["/bin/bash", "/usr/local/bin/entrypoint.sh"]
