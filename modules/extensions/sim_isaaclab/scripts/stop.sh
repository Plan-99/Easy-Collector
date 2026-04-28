#!/usr/bin/env bash
# Stop Isaac Lab + cuRobo containers.
set -e

MODULE_DIR="$(cd "$(dirname "$0")/.." && pwd)"
COMPOSE_FILE="$MODULE_DIR/docker-compose.isaaclab.yml"

echo "[sim_isaaclab] Stopping..."
docker compose -f "$COMPOSE_FILE" down
echo "[sim_isaaclab] Stopped."
