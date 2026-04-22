#!/bin/bash
# 에피소드 데이터 + replay 스크립트를 원격 서버로 전송
# Usage: bash send_replay.sh USER@HOST

set -e

if [ -z "$1" ]; then
    echo "Usage: bash send_replay.sh USER@HOST"
    exit 1
fi

REMOTE="$1"
DATASET_DIR="/opt/easytrainer/datasets/16"
SCRIPT="$(dirname "$0")/replay_episode.py"
REMOTE_DIR="/tmp/replay_test"

# SSH ControlMaster로 연결 한 번만 열기
SOCK="/tmp/ssh-replay-$$"
ssh -fNM -S "$SOCK" "$REMOTE"
trap 'ssh -S "$SOCK" -O exit "$REMOTE" 2>/dev/null' EXIT

echo "==> Creating remote directories..."
ssh -S "$SOCK" "$REMOTE" "mkdir -p ${REMOTE_DIR}/datasets/16/data/chunk-000 ${REMOTE_DIR}/datasets/16/meta"

echo "==> Sending episode data..."
scp -o "ControlPath=$SOCK" "${DATASET_DIR}/data/chunk-000/episode_000000.parquet" "${REMOTE}:${REMOTE_DIR}/datasets/16/data/chunk-000/"
scp -o "ControlPath=$SOCK" "${DATASET_DIR}/meta/info.json" "${REMOTE}:${REMOTE_DIR}/datasets/16/meta/"

echo "==> Sending replay script..."
scp -o "ControlPath=$SOCK" "${SCRIPT}" "${REMOTE}:${REMOTE_DIR}/"

echo "==> Done! Run on remote:"
echo "    cd ${REMOTE_DIR} && python3 replay_episode.py ${REMOTE_DIR}/datasets/16/episode_000000"
