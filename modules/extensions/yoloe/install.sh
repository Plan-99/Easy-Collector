#!/usr/bin/env bash
set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
echo "[yoloe] Installing YOLOE visual-prompt detection extension..."

# apt deps (none by default)
apt_deps=$(python3 -c "import json; deps=json.load(open('$SCRIPT_DIR/module.json')).get('dependencies',{}).get('apt',[]); print(' '.join(deps))" 2>/dev/null)
if [ -n "$apt_deps" ]; then
    echo "[yoloe] Installing apt packages: $apt_deps"
    apt-get update -qq && apt-get install -y --no-install-recommends $apt_deps || true
fi

# pip deps. --break-system-packages: the runtime python is PEP668 externally-managed
# (Debian); without it pip refuses and a swallowed failure leaves YOLOE uninstalled.
pip_deps=$(python3 -c "import json; deps=json.load(open('$SCRIPT_DIR/module.json')).get('dependencies',{}).get('pip',[]); print(' '.join(deps))" 2>/dev/null)
if [ -n "$pip_deps" ]; then
    echo "[yoloe] Installing pip packages: $pip_deps"
    python3 -m pip install --break-system-packages $pip_deps || \
        python3 -m pip install $pip_deps || true
fi

# Pre-fetch the default visual-prompt checkpoint so the first detection is fast.
# Weights are public (Ultralytics assets) — no token needed. Best-effort.
DATA_DIR="${EASYTRAINER_DATA_DIR:-/opt/easytrainer}"
mkdir -p "$DATA_DIR/models/yoloe"
CKPT="${YOLOE_CKPT:-yoloe-11s-seg.pt}"
python3 - "$DATA_DIR/models/yoloe" "$CKPT" <<'PY' || true
import sys, os
out_dir, ckpt = sys.argv[1], sys.argv[2]
try:
    from ultralytics import YOLOE
    os.chdir(out_dir)
    YOLOE(ckpt)  # triggers download into out_dir
    print(f"[yoloe] checkpoint ready: {os.path.join(out_dir, ckpt)}")
except Exception as e:
    print(f"[yoloe] checkpoint prefetch skipped: {e}")
PY

echo "[yoloe] Installation complete."
