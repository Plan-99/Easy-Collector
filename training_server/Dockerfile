# ===================================================================
# EasyTrainer Training Server
# GPU-accelerated training server for remote model training
# ===================================================================
FROM nvidia/cuda:12.6.3-devel-ubuntu24.04

ENV DEBIAN_FRONTEND=noninteractive
ENV TZ=Asia/Seoul
ENV PIP_NO_CACHE_DIR=1
ENV PIP_DEFAULT_TIMEOUT=300
ENV NVIDIA_VISIBLE_DEVICES=all
ENV NVIDIA_DRIVER_CAPABILITIES=compute,utility

# System packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-pip python3-dev build-essential cmake \
    && apt-get clean && rm -rf /var/lib/apt/lists/*

WORKDIR /app

# Python dependencies
COPY requirements.txt .
RUN python3 -m pip install --no-cache-dir --break-system-packages -r requirements.txt

# Install PyTorch with CUDA support
RUN python3 -m pip install --no-cache-dir --break-system-packages \
    --index-url https://download.pytorch.org/whl/cu126 \
    torch torchvision torchaudio

# Copy application code
COPY . /app

# Data directories
RUN mkdir -p /data/datasets /data/checkpoints /data/uploads

ENV PYTHONPATH=/app:/app/lerobot/src
ENV TRAINING_SERVER_DATA_DIR=/data

EXPOSE 5100

CMD ["python3", "-u", "app.py"]
