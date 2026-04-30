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

# --- Install module dependencies from manifests (single source of truth) ---
DATA_DIR="${EASYTRAINER_DATA_DIR:-/opt/easytrainer}"
MANIFEST_DIR="$DATA_DIR/project/modules"
DEPS_MARKER="/tmp/.backend_deps_installed"
if [ ! -f "$DEPS_MARKER" ]; then
    if [ -d "$MANIFEST_DIR" ]; then
        for mj in "$MANIFEST_DIR"/*.json; do
            [ -f "$mj" ] || continue
            mod_id=$(python3 -c "import json; print(json.load(open('$mj')).get('id',''))" 2>/dev/null)
            [ -z "$mod_id" ] && continue

            pip_deps=$(python3 -c "import json; deps=json.load(open('$mj')).get('dependencies',{}).get('pip',[]); print(' '.join(deps))" 2>/dev/null)
            if [ -n "$pip_deps" ]; then
                echo "[backend] Restoring pip deps for $mod_id: $pip_deps"
                python3 -m pip install --quiet $pip_deps 2>/dev/null || true
            fi

            apt_deps=$(python3 -c "import json; deps=json.load(open('$mj')).get('dependencies',{}).get('apt',[]); print(' '.join(deps))" 2>/dev/null)
            if [ -n "$apt_deps" ]; then
                echo "[backend] Restoring apt deps for $mod_id..."
                apt-get install -y --no-install-recommends $apt_deps 2>/dev/null || true
            fi
        done
    fi
    touch "$DEPS_MARKER"
fi

# --- Optional: local training server (in-container, shares torch/CUDA with backend) ---
if [ "${EASYTRAINER_LOCAL_TRAINING:-0}" = "1" ]; then
    if [ -f /root/backend/training_server/app.py ]; then
        TS_DATA_DIR="${TRAINING_SERVER_DATA_DIR:-/opt/easytrainer/training_data}"
        # Keep training_server log on host-visible path so the launcher can tail it.
        TS_LOG_DIR="${EASYTRAINER_DATA_DIR:-/opt/easytrainer}/logs"
        mkdir -p "$TS_DATA_DIR" "$TS_LOG_DIR"
        TS_LOG="$TS_LOG_DIR/training_server.log"
        echo "[backend] Starting local training_server on port ${TRAINING_SERVER_PORT:-5100} (logs: $TS_LOG)"
        TRAINING_SERVER_DATA_DIR="$TS_DATA_DIR" \
        PYTHONPATH="/root/backend/training_server:/root/backend/lerobot/src" \
            python3 -u /root/backend/training_server/app.py >> "$TS_LOG" 2>&1 &
        TS_PID=$!
        echo "[backend] training_server PID=$TS_PID"
        # If backend exits, take down training_server too.
        trap "kill $TS_PID 2>/dev/null || true" EXIT
    else
        echo "[backend][WARN] EASYTRAINER_LOCAL_TRAINING=1 but /root/backend/training_server/app.py is missing." >&2
    fi
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
