#!/bin/bash
# Copy vendored lerobot library from backend into training_server for Docker build.
# training_server is self-contained otherwise — its policies/ and utils/ directories
# are maintained independently and should NOT be overwritten from backend.
set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"
BACKEND="$PROJECT_ROOT/backend"

echo "Copying vendored lerobot from $BACKEND/lerobot..."

rm -rf "$SCRIPT_DIR/lerobot"
cp -r "$BACKEND/lerobot" "$SCRIPT_DIR/lerobot"

echo "Done."
