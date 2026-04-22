#!/usr/bin/env bash
set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
echo "[custom_interfaces] Installing Custom Interfaces..."

# Install apt dependencies
apt_deps=$(python3 -c "import json; deps=json.load(open('$SCRIPT_DIR/module.json')).get('dependencies',{}).get('apt',[]); print(' '.join(deps))" 2>/dev/null)
if [ -n "$apt_deps" ]; then
    echo "[custom_interfaces] Installing apt packages: $apt_deps"
    apt-get update -qq && apt-get install -y --no-install-recommends $apt_deps || true
fi

# Install pip dependencies
pip_deps=$(python3 -c "import json; deps=json.load(open('$SCRIPT_DIR/module.json')).get('dependencies',{}).get('pip',[]); print(' '.join(deps))" 2>/dev/null)
if [ -n "$pip_deps" ]; then
    echo "[custom_interfaces] Installing pip packages: $pip_deps"
    python3 -m pip install --quiet $pip_deps || true
fi

echo "[custom_interfaces] Installation complete."
