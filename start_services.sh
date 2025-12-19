#!/usr/bin/env bash
set -euo pipefail

echo "[ENTRY] Starting Easy Collector services (single container)"

# Print environment and runtime summary for diagnostics
env_summary() {
  echo "[ENV] Flags: EC_NO_FRONTEND=${EC_NO_FRONTEND:-0} EC_NO_SIDE_PROCESSES=${EC_NO_SIDE_PROCESSES:-0} EC_SKIP_TORCHVISION_COMPAT=${EC_SKIP_TORCHVISION_COMPAT:-0} EC_MINIMAL_API=${EC_MINIMAL_API:-0} EC_NO_ROS=${EC_NO_ROS:-0} EC_DEBUG=${EC_DEBUG:-0}"
  echo "[ENV] Node max-old-space-size: ${EC_NODE_MAX_OLD_SPACE_SIZE:-unset}"
  echo "[ENV] Python: $(python3 -V 2>/dev/null || echo N/A) | Pip: $(python3 -m pip -V 2>/dev/null || echo N/A)"
  echo "[ENV] Pip index: ${PIP_INDEX_URL:-unset} | extra-index: ${PIP_EXTRA_INDEX_URL:-unset}"
  echo "[ENV] npm registry: ${NPM_CONFIG_REGISTRY:-unset}"
  python3 - <<'PY'
try:
    import importlib.metadata as m
    def ver(p):
        try:
            return m.version(p)
        except Exception:
            return 'n/a'
    names = ['flask','flask-socketio','orator','torch','torchvision','rclpy']
    print('[ENV] Packages:', ' '.join([f"%s=%s" % (n, ver(n)) for n in names]))
except Exception as e:
    print('[ENV] Packages: unavailable', e)
PY
}

# Detect low-memory environments and enable safe defaults unless explicitly overridden
detect_low_mem() {
  local mem_kb limit_kb
  mem_kb=$(awk '/MemTotal:/ {print $2}' /proc/meminfo 2>/dev/null || echo 0)
  local limit
  limit=$(cat /sys/fs/cgroup/memory.max 2>/dev/null || echo max)
  if [[ "$limit" == "max" || -z "$limit" ]]; then
    limit_kb=$mem_kb
  else
    # memory.max is bytes on cgroup v2
    limit_kb=$(( limit / 1024 ))
  fi
  local avail_kb=$mem_kb
  if (( limit_kb > 0 && limit_kb < mem_kb )); then
    avail_kb=$limit_kb
  fi
  # Consider < 4 GB as low-memory for this stack
  if (( avail_kb < 4*1024*1024 )); then
    LOW_MEM=1
  else
    LOW_MEM=0
  fi
  echo "[ENTRY] Memory total=${mem_kb}kB cgroup_limit=${limit_kb}kB low_mem=${LOW_MEM}"
}

detect_low_mem
if [[ "${LOW_MEM:-0}" == "1" ]]; then
  echo "[ENTRY] Low-memory mode detected (<4GB). Applying safe defaults."
  : "${EC_NO_FRONTEND:=1}"; export EC_NO_FRONTEND
  : "${EC_NO_SIDE_PROCESSES:=1}"; export EC_NO_SIDE_PROCESSES
  : "${EC_SKIP_TORCHVISION_COMPAT:=1}"; export EC_SKIP_TORCHVISION_COMPAT
  : "${EC_NODE_MAX_OLD_SPACE_SIZE:=256}"; export EC_NODE_MAX_OLD_SPACE_SIZE
fi

# Persistent Python vendor path and pip cache (mounted volume on host)
PY_VENDOR_DIR=${PY_VENDOR_DIR:-/root/python_pkgs/vendor}
PIP_CACHE_DIR=${PIP_CACHE_DIR:-/root/python_pkgs/.cache/pip}
mkdir -p "$PY_VENDOR_DIR" "$PIP_CACHE_DIR"

env_summary

# Resolve a developer source path (env > UI config) and sync into /root/src before start
resolve_sync_source() {
  local src="${EC_SOURCE_PROJECT_ROOT:-${EASYTRAINER_DEV_SRC_ROOT:-}}"
  # Fall back to UI config if env not set
  if [[ -z "$src" ]]; then
    local cfg="/root/EasyTrainer/config.json"
    if [[ -f "$cfg" ]]; then
      src=$(python3 - <<'PY'
import json, sys
cfg = sys.argv[1]
try:
    with open(cfg, encoding='utf-8') as f:
        data = json.load(f)
    src = (data.get('dev_src_root') or '').strip()
    if src:
        print(src)
except Exception:
    pass
PY
      "$cfg")
    fi
  fi
  echo "$src"
}

sync_project_root() {
  local src
  src="$(resolve_sync_source)"
  local dest="/root/src"
  if [[ -z "$src" ]]; then
    echo "[SYNC] No dev source configured (env or UI config); skipping sync"
    return
  fi
  if [[ ! -d "$src" ]]; then
    echo "[SYNC][WARN] Dev source path not found: $src"
    return
  fi
  if [[ "$src" == "$dest" ]]; then
    echo "[SYNC] Source and destination are the same ($src); skipping sync"
    return
  fi
  echo "[SYNC] Syncing project from $src -> $dest"
  mkdir -p "$dest"
  rsync -a --delete \
    --exclude '.git' \
    --exclude 'node_modules' \
    --exclude 'logs' \
    "$src"/ "$dest"/ || echo "[SYNC][WARN] rsync encountered errors"
}

sync_project_root

# Ensure ROS 2 environment is available for rclpy and ros2 CLI.
# Temporarily disable nounset because ROS setup scripts reference unset vars.
set +u
if [[ -f "/opt/ros/humble/setup.bash" ]]; then
  echo "[ROS] Sourcing /opt/ros/humble/setup.bash"
  # shellcheck disable=SC1091
  source /opt/ros/humble/setup.bash
fi
if [[ -f "/root/ros2_ws/install/setup.bash" ]]; then
  echo "[ROS] Sourcing /root/ros2_ws/install/setup.bash"
  # shellcheck disable=SC1091
  source /root/ros2_ws/install/setup.bash
fi
set -u

# Optional debug flags for backend
BACKEND_FLAGS=""
if [[ "${EC_DEBUG:-0}" == "1" ]]; then
  BACKEND_FLAGS="--debug"
  echo "[ENTRY] Backend debug mode enabled"
fi

# Prepare log directory
LOG_DIR=/root/easytrainer/logs
mkdir -p "$LOG_DIR"

# 1) Ensure frontend deps; start Quasar dev server (skippable)
if [[ "${EC_NO_FRONTEND:-0}" != "1" ]]; then
  cd /root/src/ui
  # Harden npm for flaky networks; allow overriding registry via NPM_CONFIG_REGISTRY
  export NPM_CONFIG_FETCH_RETRIES=${NPM_CONFIG_FETCH_RETRIES:-5}
  export NPM_CONFIG_FETCH_RETRY_MINTIMEOUT=${NPM_CONFIG_FETCH_RETRY_MINTIMEOUT:-20000}
  export NPM_CONFIG_FETCH_RETRY_MAXTIMEOUT=${NPM_CONFIG_FETCH_RETRY_MAXTIMEOUT:-120000}
  export NPM_CONFIG_AUDIT=${NPM_CONFIG_AUDIT:-false}
  export NPM_CONFIG_FUND=${NPM_CONFIG_FUND:-false}
  if [[ ! -d node_modules ]] || [[ ! -x ./node_modules/.bin/quasar ]]; then
    echo "[FRONTEND] Installing node modules ..."
    (npm ci --no-audit --no-fund --prefer-offline --legacy-peer-deps || npm install --no-audit --no-fund --legacy-peer-deps)
  fi
  echo "[FRONTEND] Starting Quasar dev server on 0.0.0.0:5173"
  export NODE_OPTIONS="--max-old-space-size=${EC_NODE_MAX_OLD_SPACE_SIZE:-384}"
  ./node_modules/.bin/quasar dev -p 5173 --host 0.0.0.0 >>"$LOG_DIR/frontend.log" 2>&1 &
  FRONT_PID=$!
else
  echo "[FRONTEND] Skipped (EC_NO_FRONTEND=1)"
  FRONT_PID=0
fi

ensure_orator() {
  # Check import inside an if to avoid set -e aborts
  if python3 - <<'PY'
import sys
try:
    import orator  # noqa: F401
    sys.exit(0)
except Exception:
    sys.exit(1)
PY
  then
    :
  else
    echo "[SETUP] Installing orator deps (no-deps mode)..."
    python3 -m pip install --no-deps --no-input backpack==0.1 simplejson faker lazy-object-proxy cleo==0.6.8 inflection pendulum==1.5.1 pytzdata python-dateutil >/dev/null 2>&1 || true
    echo "[SETUP] Installing orator (no-deps)..."
    python3 -m pip install --no-deps --no-input --prefer-binary orator==0.9.9 && hash -r || {
      echo "[SETUP][ERROR] Failed to install orator" >&2
      exit 1
    }
    # Verify import now
    if python3 - <<'PY'
import sys
try:
    import orator  # noqa: F401
    print('orator_ready')
    sys.exit(0)
except Exception as e:
    print('orator_import_failed:', e)
    sys.exit(1)
PY
    then
      :
    else
      echo "[SETUP][ERROR] orator import still failing after install" >&2
      exit 1
    fi
  fi

  # Log orator details (module version, CLI path + version) once per container
  if [[ -z "${ORATOR_LOGGED:-}" ]]; then
    ORATOR_LOGGED=1; export ORATOR_LOGGED
    python3 - <<'PY'
try:
    import importlib.metadata as m
    v = m.version('orator')
except Exception:
    v = 'unknown'
print(f"[SETUP] orator python module version: {v}")
PY
    if command -v orator >/dev/null 2>&1; then
      echo "[SETUP] orator CLI path: $(command -v orator)"
      python3 -m orator --version 2>/dev/null || true
    else
      echo "[SETUP] orator CLI path: not found (using python -m orator)"
      python3 -m orator --version 2>/dev/null || true
    fi
  fi
}

# Ensure torchvision/torch compatibility (avoid AttributeError: torch.library.register_fake)
ensure_torchvision_compat() {
  # Decide desired torchvision based on torch version/capabilities
  read -r desired current torch_ver < <(python3 - <<'PY'
import sys
try:
    import torch
    torch_ver = getattr(torch, '__version__', '')
except Exception:
    torch_ver = ''
current = ''
try:
    import importlib.metadata as m
    current = m.version('torchvision')
except Exception:
    try:
        import torchvision
        current = getattr(torchvision, '__version__', '')
    except Exception:
        current = ''

desired = ''
try:
    nums = [int(x) for x in torch_ver.split('+')[0].split('.')[:2]]
    major, minor = nums[0], nums[1]
    if major > 2 or (major == 2 and minor >= 3):
        desired = '0.18.0'
    else:
        desired = '0.15.2'
except Exception:
    desired = '0.18.0'

print(desired or '', current or '', torch_ver or '')
PY
)
  if [[ -n "${desired:-}" && "${current:-}" != "$desired" ]]; then
    echo "[SETUP] Installing torchvision==$desired to match torch ${torch_ver:-unknown} (current: ${current:-none})..."
    python3 -m pip install --no-deps --no-input --prefer-binary \
      --cache-dir "${PIP_CACHE_DIR}" \
      "torchvision==$desired" && hash -r || true
  fi
}

# 2) Ensure DB deps and run migrations (idempotent)
if [[ "${EC_SKIP_TORCHVISION_COMPAT:-0}" == "1" ]]; then
  echo "[STEP] Skipping torchvision/torch compatibility (EC_SKIP_TORCHVISION_COMPAT=1)"
else
  echo "[STEP] Ensuring torchvision/torch compatibility"
  ensure_torchvision_compat
fi
echo "[STEP] Ensuring orator availability"
ensure_orator
echo "[STEP] Switching to /root/src/backend/database"
cd /root/src/backend/database
echo "[DB] Running migrations (Python API) ..."
(
  python3 -m pip show orator >/dev/null 2>&1 || \
    python3 -m pip install --no-deps --no-input --prefer-binary -q orator==0.9.9
  python3 - <<'PY'
import os, sys
sys.path.insert(0, os.path.abspath('.'))
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
        print('[DB] creating migrations repository table ...')
        repo.create_repository()
    migrator = Migrator(repo, db)
    path = os.path.join(os.getcwd(), 'migrations')
    print('[DB] running migrations from', path)
    migrator.run(path)
    print('[DB] migration complete')
except Exception as e:
    import traceback
    traceback.print_exc()
    sys.exit(1)
PY
) || {
  echo "[DB][WARN] Python API migration failed; trying migration.sh if present" >&2
  bash migration.sh || true
}

# 3) Start backend API
cd /root/src
echo "[BACKEND] Starting Flask-SocketIO API on 0.0.0.0:5000"
# Prime backend log so it's never empty
echo "[BACKEND][BOOT] $(date -Is) starting backend.api.app" >>"$LOG_DIR/backend.log" 2>/dev/null || true
# Double-check import before start; hard-fail if unresolved
ensure_orator
if [[ "${EC_SKIP_TORCHVISION_COMPAT:-0}" != "1" ]]; then
  ensure_torchvision_compat
fi
BACKEND_ENTRY="${EC_BACKEND_ENTRY:-backend.api.app}"
echo "[BACKEND] Entry: ${BACKEND_ENTRY}"
python3 -u -m "${BACKEND_ENTRY}" ${BACKEND_FLAGS} >>"$LOG_DIR/backend.log" 2>&1 &
BACK_PID=$!
echo "[BACKEND] PID=${BACK_PID}"

# Probe simple health endpoint so status shows in docker logs
(
  for i in $(seq 1 30); do
    if curl -fsS -m 2 http://127.0.0.1:5000/api/healthz >/dev/null 2>&1; then
      echo "[HEALTH] backend /api/healthz OK"
      break
    else
      echo "[HEALTH] waiting backend (try $i)"
      sleep 1
    fi
  done
) &

# Handle termination and child exit
term() {
  echo "[ENTRY] Shutting down..."
  local pids=()
  [[ ${TAIL_PID:-0} -gt 0 ]] && pids+=("${TAIL_PID}")
  [[ ${BACK_PID:-0} -gt 0 ]] && pids+=("${BACK_PID}")
  [[ ${FRONT_PID:-0} -gt 0 ]] && pids+=("${FRONT_PID}")
  if ((${#pids[@]})); then
    kill -TERM "${pids[@]}" 2>/dev/null || true
  fi
}
trap term INT TERM

# Aggregate logs to container STDOUT so `docker logs` shows both backend and frontend logs
touch "$LOG_DIR/backend.log" "$LOG_DIR/frontend.log" 2>/dev/null || true
echo "[LOGS] Aggregating backend/frontend logs to STDOUT"
tail -n +1 -F "$LOG_DIR/backend.log" "$LOG_DIR/frontend.log" &
TAIL_PID=$!

# Wait for either to exit, then terminate the other
set +e
wait_pids=()
[[ ${BACK_PID:-0} -gt 0 ]] && wait_pids+=("${BACK_PID}")
[[ ${FRONT_PID:-0} -gt 0 ]] && wait_pids+=("${FRONT_PID}")
if ((${#wait_pids[@]})); then
  wait -n "${wait_pids[@]}"
  EC=$?
else
  echo "[ENTRY][ERROR] No child processes to wait for."
  EC=1
fi
term
wait || true
echo "[ENTRY] Exit code: ${EC}"
exit ${EC}
