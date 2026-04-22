#!/usr/bin/env bash
set -e

LOG_DIR="${EASYTRAINER_LOG_DIR:-/tmp/easytrainer/logs}"
mkdir -p "$LOG_DIR"

# --- Database migration (peewee create_tables) ---
echo "[backend] Running database migrations..."
cd /root
python3 -c "
import sys; sys.path.insert(0, '/root')
try:
    from backend.database.models import create_tables
    create_tables()
    print('[backend] Migration complete')
except Exception as e:
    print(f'[backend][WARN] Migration failed: {e}', file=sys.stderr)
"

# --- Install extension module dependencies (skip if already done) ---
MODULES_DIR="/root/backend/modules"
DEPS_MARKER="/tmp/.backend_deps_installed"
if [ -d "$MODULES_DIR" ] && [ "$(ls -A $MODULES_DIR 2>/dev/null)" ] && [ ! -f "$DEPS_MARKER" ]; then
    for mj in "$MODULES_DIR"/*/module.json; do
        [ -f "$mj" ] || continue
        pip_deps=$(python3 -c "import json; deps=json.load(open('$mj')).get('dependencies',{}).get('pip',[]); print(' '.join(deps))" 2>/dev/null)
        if [ -n "$pip_deps" ]; then
            echo "[backend] Installing module deps: $pip_deps"
            python3 -m pip install --quiet $pip_deps 2>/dev/null || true
        fi
    done
    touch "$DEPS_MARKER"
fi

# --- Start Flask API ---
cd /root
echo "[backend] Starting Flask-SocketIO on port 5000..."

if [ "${EC_BACKEND_AUTORELOAD:-0}" = "1" ]; then
    python3 -m pip install --quiet watchfiles 2>/dev/null || true
    exec python3 -m watchfiles \
        --filter python \
        --target-type=command \
        "bash -lc 'cd /root && python3 -u -m backend.api.app ${EC_DEBUG:+--debug}'" \
        /root/backend
else
    exec python3 -u -m backend.api.app ${EC_DEBUG:+--debug}
fi
