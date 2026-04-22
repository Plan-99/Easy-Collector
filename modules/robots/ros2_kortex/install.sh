#!/usr/bin/env bash
set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
echo "[robot_kinova] Installing Kinova Kortex..."

# Install apt dependencies
apt_deps=$(python3 -c "import json; deps=json.load(open('$SCRIPT_DIR/module.json')).get('dependencies',{}).get('apt',[]); print(' '.join(deps))" 2>/dev/null)
if [ -n "$apt_deps" ]; then
    echo "[robot_kinova] Installing apt packages: $apt_deps"
    apt-get update -qq && apt-get install -y --no-install-recommends $apt_deps || true
fi

# Install pip dependencies
pip_deps=$(python3 -c "import json; deps=json.load(open('$SCRIPT_DIR/module.json')).get('dependencies',{}).get('pip',[]); print(' '.join(deps))" 2>/dev/null)
if [ -n "$pip_deps" ]; then
    echo "[robot_kinova] Installing pip packages: $pip_deps"
    python3 -m pip install --quiet $pip_deps || true
fi

echo "[robot_kinova] Installation complete."
