#!/usr/bin/env bash
set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
echo "[sam3] Installing SAM 3 segmentation extension..."

# Install apt dependencies
apt_deps=$(python3 -c "import json; deps=json.load(open('$SCRIPT_DIR/module.json')).get('dependencies',{}).get('apt',[]); print(' '.join(deps))" 2>/dev/null)
if [ -n "$apt_deps" ]; then
    echo "[sam3] Installing apt packages: $apt_deps"
    apt-get update -qq && apt-get install -y --no-install-recommends $apt_deps || true
fi

# Install pip dependencies (sam3 from GitHub source — gated weights handled at runtime)
pip_deps=$(python3 -c "import json; deps=json.load(open('$SCRIPT_DIR/module.json')).get('dependencies',{}).get('pip',[]); print(' '.join(deps))" 2>/dev/null)
if [ -n "$pip_deps" ]; then
    echo "[sam3] Installing pip packages: $pip_deps"
    # --break-system-packages: the runtime python is PEP668 externally-managed
    #   (Debian); without this flag pip refuses and the `|| true` silently swallows
    #   it, leaving sam3 NOT installed (root cause of "No module named 'sam3'").
    # NOTE: the sam3 source dist installs without git (PyPI-style tarball); the
    #   git+ URL needs `git` which the backend image lacks.
    python3 -m pip install --break-system-packages $pip_deps || \
        python3 -m pip install $pip_deps || true
fi

# HuggingFace token storage — capture itself is handled by the launcher via
# module.json's post_install.credentials block (runs on host UI thread, so a
# real Qt dialog is possible). This script just ensures directories exist
# and persists $HF_TOKEN if it was passed in via env (CI / manual override).
DATA_DIR="${EASYTRAINER_DATA_DIR:-/opt/easytrainer}"
TOKEN_FILE="$DATA_DIR/.hf_token"

mkdir -p "$DATA_DIR"
mkdir -p "$DATA_DIR/models/sam3"

if [ -n "$HF_TOKEN" ] && [ ! -f "$TOKEN_FILE" ]; then
    echo "[sam3] Saving HF_TOKEN from environment to $TOKEN_FILE"
    echo "$HF_TOKEN" > "$TOKEN_FILE"
    chmod 600 "$TOKEN_FILE" || true
fi

echo "[sam3] Installation complete. Token will be requested by the launcher dialog if missing."
