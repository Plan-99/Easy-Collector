#!/bin/bash
# Copy shared dependencies from Easy-Collector into training_server for Docker build.
# Run this before `docker compose build`.

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"

echo "Copying shared dependencies..."

# Copy policies/utils.py
mkdir -p "$SCRIPT_DIR/policies"
cp "$PROJECT_ROOT/src/backend/policies/utils.py" "$SCRIPT_DIR/policies/utils.py"
cp "$PROJECT_ROOT/src/backend/policies/__init__.py" "$SCRIPT_DIR/policies/__init__.py" 2>/dev/null || touch "$SCRIPT_DIR/policies/__init__.py"

# Copy lerobot (vendored library)
rm -rf "$SCRIPT_DIR/lerobot"
cp -r "$PROJECT_ROOT/src/backend/lerobot" "$SCRIPT_DIR/lerobot"

# Copy lerobot_io utilities
cp "$PROJECT_ROOT/src/backend/api/process/lerobot_io.py" "$SCRIPT_DIR/lerobot_io.py"

echo "Done. You can now run: docker compose build"
