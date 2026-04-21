#!/usr/bin/env bash
set -e

LOG_DIR="${EASYTRAINER_LOG_DIR:-/tmp/easytrainer/logs}"
mkdir -p "$LOG_DIR"

# --- Database migration ---
echo "[backend] Running database migrations..."
cd /root/backend/database

python3 - <<'PY'
import os, sys
sys.path.insert(0, os.getcwd())
try:
    from importlib import import_module
    cfg = import_module('config.database')
    if hasattr(cfg, 'DATABASES'):
        connections = {k:v for k,v in cfg.DATABASES.items() if isinstance(v, dict)}
    elif hasattr(cfg, 'config'):
        connections = cfg.config
    else:
        raise RuntimeError('No DATABASES/config mapping in config/database.py')
    from orator import DatabaseManager
    from orator.migrations import Migrator, DatabaseMigrationRepository
    db = DatabaseManager(connections)
    repo = DatabaseMigrationRepository(db, 'migrations')
    if not repo.repository_exists():
        repo.create_repository()
    migrator = Migrator(repo, db)
    path = os.path.join(os.getcwd(), 'migrations')
    migrator.run(path)
    print('[backend] Migration complete')
except Exception as e:
    print(f'[backend][WARN] Migration failed: {e}', file=sys.stderr)
PY

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
