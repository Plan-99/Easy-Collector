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
    python3 -m pip install --quiet $pip_deps || true
fi

# HuggingFace token setup — SAM3 weights live in a gated repo (facebook/sam3).
# Users must (1) accept the license on https://huggingface.co/facebook/sam3
# and (2) provide an access token. We persist the token under EASYTRAINER_DATA_DIR
# so it survives container restarts. Skip prompt when running non-interactively
# (e.g. CI / automatic install) — in that case rely on $HF_TOKEN env var.
DATA_DIR="${EASYTRAINER_DATA_DIR:-/opt/easytrainer}"
TOKEN_FILE="$DATA_DIR/.hf_token"

mkdir -p "$DATA_DIR"

if [ -n "$HF_TOKEN" ] && [ ! -f "$TOKEN_FILE" ]; then
    echo "[sam3] Saving HF_TOKEN from environment to $TOKEN_FILE"
    echo "$HF_TOKEN" > "$TOKEN_FILE"
    chmod 600 "$TOKEN_FILE" || true
fi

if [ ! -f "$TOKEN_FILE" ] && [ -t 0 ]; then
    echo ""
    echo "[sam3] SAM 3 weights are gated on HuggingFace (facebook/sam3)."
    echo "       1) Visit https://huggingface.co/facebook/sam3 and click \"Agree and access\"."
    echo "       2) Create a read token at https://huggingface.co/settings/tokens"
    echo "       3) Paste it below (or leave blank to set later via HF_TOKEN env)."
    read -r -p "[sam3] HuggingFace token: " HF_INPUT_TOKEN
    if [ -n "$HF_INPUT_TOKEN" ]; then
        echo "$HF_INPUT_TOKEN" > "$TOKEN_FILE"
        chmod 600 "$TOKEN_FILE" || true
        echo "[sam3] Token saved to $TOKEN_FILE"
    fi
fi

# Cache directory for downloaded weights
mkdir -p "$DATA_DIR/models/sam3"

echo "[sam3] Installation complete. Weights will download on first use."
