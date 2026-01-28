#!/usr/bin/env bash
set -euo pipefail

STATUS_LOG=""

log_status() {
  local msg="$1"
  echo "$msg"
  if [[ -n "${STATUS_LOG:-}" ]]; then
    printf '%s\n' "$msg" >>"$STATUS_LOG" 2>/dev/null || true
  fi
}

export PYTHONPATH="/root/src:/opt/openrobots/lib/python3.10/site-packages:${PYTHONPATH:-}"
export LD_LIBRARY_PATH="/opt/openrobots/lib:${LD_LIBRARY_PATH:-}"
export PATH="/opt/openrobots/bin:${PATH}"

log_status "[DEBUG] Library path forced to /opt/openrobots and /root/src"

# Print environment and runtime summary for diagnostics
env_summary() {
  echo "[ENV] Flags: EC_NO_FRONTEND=${EC_NO_FRONTEND:-0} EC_NO_SIDE_PROCESSES=${EC_NO_SIDE_PROCESSES:-0} EC_SKIP_TORCHVISION_COMPAT=${EC_SKIP_TORCHVISION_COMPAT:-0} EC_MINIMAL_API=${EC_MINIMAL_API:-0} EC_NO_ROS=${EC_NO_ROS:-0} EC_DEBUG=${EC_DEBUG:-0}"
  echo "[ENV] Node max-old-space-size: ${EC_NODE_MAX_OLD_SPACE_SIZE:-unset}"
  echo "[ENV] Python: $(python3 -V 2>/dev/null || echo N/A) | Pip: $(python3 -m pip -V 2>/dev/null || echo N/A)"
  echo "[ENV] Pip index: ${PIP_INDEX_URL:-unset} | extra-index: ${PIP_EXTRA_INDEX_URL:-unset}"
  echo "[ENV] Pip cache: NO_CACHE=${PIP_NO_CACHE_DIR:-unset} DIR=${PIP_CACHE_DIR:-unset}"
  echo "[ENV] Backend autoreload: EC_BACKEND_AUTORELOAD=${EC_BACKEND_AUTORELOAD:-0}"
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

init_status_log() {
  STATUS_LOG="${LOG_DIR}/status.log"
  mkdir -p "$LOG_DIR" 2>/dev/null || true
  if ! touch "$STATUS_LOG" 2>/dev/null; then
    sudo touch "$STATUS_LOG" 2>/dev/null || true
  fi
  chmod 666 "$STATUS_LOG" 2>/dev/null || true
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
  log_status "[ENTRY] Memory total=${mem_kb}kB cgroup_limit=${limit_kb}kB low_mem=${LOW_MEM}"
}

# Shared data root for config persistence (keep out of $HOME)
DATA_ROOT=${EASYTRAINER_DATA_DIR:-/opt/easytrainer}
CONFIG_PATH=${EASYTRAINER_CONFIG_PATH:-${DATA_ROOT}/config.json}
# Force logs to /tmp to avoid host permission issues (ignore inbound override)
LOG_DIR="/tmp/easytrainer/logs"
export EASYTRAINER_DATA_DIR="${DATA_ROOT}"
export EASYTRAINER_LOG_DIR="${LOG_DIR}"
export EASYTRAINER_CONFIG_PATH="${CONFIG_PATH}"

ensure_data_root_writable() {
  mkdir -p "$DATA_ROOT" "$LOG_DIR" 2>/dev/null || sudo mkdir -p "$DATA_ROOT" "$LOG_DIR" || true
  chmod 777 "$DATA_ROOT" "$LOG_DIR" 2>/dev/null || true
  if [ ! -f "$CONFIG_PATH" ]; then
    echo "{}" >"$CONFIG_PATH" 2>/dev/null || sudo sh -c "echo '{}' >\"$CONFIG_PATH\"" || true
  fi
  chmod 666 "$CONFIG_PATH" 2>/dev/null || true
  if [ ! -f "$LOG_DIR/launcher.log" ]; then
    touch "$LOG_DIR/launcher.log" 2>/dev/null || sudo touch "$LOG_DIR/launcher.log" 2>/dev/null || true
  fi
  chmod 666 "$LOG_DIR/launcher.log" 2>/dev/null || true

  # If we still cannot write the config, fall back to /tmp (per-container writable)
  if ! touch "$CONFIG_PATH" 2>/dev/null; then
    log_status "[ENTRY][WARN] $CONFIG_PATH is not writable; falling back to /tmp/easytrainer"
    DATA_ROOT="/tmp/easytrainer"
    CONFIG_PATH="${DATA_ROOT}/config.json"
    LOG_DIR="/tmp/easytrainer/logs"
    export EASYTRAINER_DATA_DIR="${DATA_ROOT}"
    export EASYTRAINER_LOG_DIR="${LOG_DIR}"
    export EASYTRAINER_CONFIG_PATH="${CONFIG_PATH}"
    mkdir -p "$LOG_DIR" 2>/dev/null || true
    echo "{}" >"$CONFIG_PATH" 2>/dev/null || true
    chmod 777 "$DATA_ROOT" "$LOG_DIR" 2>/dev/null || true
    chmod 666 "$CONFIG_PATH" 2>/dev/null || true
    touch "$LOG_DIR/launcher.log" 2>/dev/null || true
    chmod 666 "$LOG_DIR/launcher.log" 2>/dev/null || true
  fi
}

apply_ros_domain_from_config() {
  local cfg="${CONFIG_PATH}"
  local value="0"
  if [[ -f "$cfg" && -r "$cfg" ]]; then
    value=$(python3 - "$cfg" <<'PY'
import json, sys
cfg = sys.argv[1] if len(sys.argv) > 1 else ""
value = 0
if cfg:
    try:
        with open(cfg, encoding="utf-8") as f:
            data = json.load(f)
        raw = data.get("ros_domain_id", 0)
        try:
            value = int(str(raw).strip())
        except Exception:
            value = 0
    except Exception:
        value = 0
if value < 0:
    value = 0
if value > 232:
    value = 232
print(value)
PY
    )
  fi
  if [[ -z "$value" ]]; then
    value=0
  fi
  export ROS_DOMAIN_ID="$value"
  log_status "[ROS] ROS_DOMAIN_ID=${ROS_DOMAIN_ID}"
  if [[ -d /etc/profile.d ]]; then
    printf 'export ROS_DOMAIN_ID=%s\n' "$ROS_DOMAIN_ID" > /etc/profile.d/ros_domain_id.sh 2>/dev/null || true
  fi
}

ensure_data_root_writable

init_status_log
log_status "[ENTRY] Starting Easy Collector services (single container)"
log_status "[ENTRY] Data root=${DATA_ROOT} logs=${LOG_DIR}"
apply_ros_domain_from_config

detect_low_mem
if [[ "${LOW_MEM:-0}" == "1" ]]; then
  log_status "[ENTRY] Low-memory mode detected (<4GB). Applying safe defaults."
  : "${EC_NO_FRONTEND:=1}"; export EC_NO_FRONTEND
  : "${EC_NO_SIDE_PROCESSES:=1}"; export EC_NO_SIDE_PROCESSES
  : "${EC_SKIP_TORCHVISION_COMPAT:=1}"; export EC_SKIP_TORCHVISION_COMPAT
  : "${EC_NODE_MAX_OLD_SPACE_SIZE:=256}"; export EC_NODE_MAX_OLD_SPACE_SIZE
fi
# Default to fast dev turnaround (backend autoreload) unless explicitly disabled
: "${EC_DEBUG:=1}"; export EC_DEBUG
: "${EC_BACKEND_AUTORELOAD:=1}"; export EC_BACKEND_AUTORELOAD

# Persistent Python vendor path and pip cache (mounted volume on host)
PY_VENDOR_DIR=${PY_VENDOR_DIR:-/root/python_pkgs/vendor}
PIP_CACHE_DIR=${PIP_CACHE_DIR:-/root/python_pkgs/.cache/pip}
PIP_NO_CACHE_DIR=${PIP_NO_CACHE_DIR:-0}
export PIP_NO_CACHE_DIR
mkdir -p "$PY_VENDOR_DIR" "$PIP_CACHE_DIR"

log_status "[ENTRY] Capturing environment summary"
env_summary

# Resolve a developer source path (env > UI config) and sync into /root/src before start
resolve_sync_source() {
  local src="${EC_SOURCE_PROJECT_ROOT:-${EASYTRAINER_DEV_SRC_ROOT:-}}"
  # Fall back to UI config if env not set
  if [[ -z "$src" ]]; then
    local cfg="$CONFIG_PATH"
    if [[ -f "$cfg" && -r "$cfg" ]]; then
      src=$(python3 - <<'PY'
import json, sys
cfg = sys.argv[1] if len(sys.argv) > 1 else ""
if not cfg:
    sys.exit(0)
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
    log_status "[SYNC] No dev source configured (env or UI config); skipping sync"
    return
  fi
  if [[ ! -d "$src" ]]; then
    log_status "[SYNC][WARN] Dev source path not found: $src"
    return
  fi
  if [[ "$src" == "$dest" ]]; then
    log_status "[SYNC] Source and destination are the same ($src); skipping sync"
    return
  fi
  log_status "[SYNC] Syncing project from $src -> $dest"
  mkdir -p "$dest"
  rsync -a --delete \
    --exclude '.git' \
    --exclude 'node_modules' \
    --exclude 'backend/database/*.db' \
    --exclude 'logs' \
    "$src"/ "$dest"/ || log_status "[SYNC][WARN] rsync encountered errors"
}

sync_project_root

# Ensure missing package init files are present to avoid import failures
ensure_lerobot_dataset_init() {
  local target="/root/src/backend/lerobot/datasets/__init__.py"
  if [ -f "$target" ]; then
    return
  fi
  mkdir -p "$(dirname "$target")"
  cat >"$target" <<'PYCODE'
"""Dataset utilities for LeRobot."""

from .utils import (
    append_jsonlines,
    embed_images,
    flatten_dict,
    get_nested_item,
    load_info,
    load_json,
    load_jsonlines,
    load_stats,
    serialize_dict,
    unflatten_dict,
    write_info,
    write_json,
    write_jsonlines,
    write_stats,
    write_task,
)

__all__ = [
    "append_jsonlines",
    "embed_images",
    "flatten_dict",
    "get_nested_item",
    "load_info",
    "load_json",
    "load_jsonlines",
    "load_stats",
    "serialize_dict",
    "unflatten_dict",
    "write_info",
    "write_json",
    "write_jsonlines",
    "write_stats",
    "write_task",
]
PYCODE
  chmod 644 "$target" 2>/dev/null || true
  log_status "[PATCH] Created missing backend/lerobot/datasets/__init__.py"
}

ensure_lerobot_dataset_init

build_ros2_workspace() {
  local ws="/root/ros2_ws"
  log_status "[ROS] Building ROS 2 workspace at $ws (this may take a while)"
  ( cd "$ws" && source /opt/ros/humble/setup.bash && colcon build --symlink-install )
}

ensure_ros2_workspace() {
  local ws="/root/ros2_ws"
  local install="$ws/install/setup.bash"
  if [[ "${EC_NO_ROS:-0}" == "1" ]]; then
    log_status "[ROS] Skipped workspace build (EC_NO_ROS=1)"
    return
  fi
  if [[ ! -d "$ws" ]]; then
    log_status "[ROS][WARN] ROS 2 workspace not found at $ws; skipping build"
    return
  fi
  if [[ ! -f "$install" ]]; then
    build_ros2_workspace
    return
  fi
  if ! (source /opt/ros/humble/setup.bash && source "$install" && ros2 pkg list 2>/dev/null | grep -q '^piper$'); then
    log_status "[ROS] Workspace found but required packages missing; rebuilding"
    build_ros2_workspace
  else
    log_status "[ROS] Workspace already built (piper package found)"
  fi
}

# Ensure ROS 2 environment is available for rclpy and ros2 CLI.
# Temporarily disable nounset because ROS setup scripts reference unset vars.
set +u
if [[ -f "/opt/ros/humble/setup.bash" ]]; then
  log_status "[ROS] Sourcing /opt/ros/humble/setup.bash"
  # shellcheck disable=SC1091
  source /opt/ros/humble/setup.bash
fi
ensure_ros2_workspace
if [[ -f "/root/ros2_ws/install/setup.bash" ]]; then
  log_status "[ROS] Sourcing /root/ros2_ws/install/setup.bash"
  # shellcheck disable=SC1091
  source /root/ros2_ws/install/setup.bash
fi
set -u

# Optional debug flags for backend
BACKEND_FLAGS=""
if [[ "${EC_DEBUG:-0}" == "1" ]]; then
  BACKEND_FLAGS="--debug"
  log_status "[ENTRY] Backend debug mode enabled"
fi

# Prepare log directory
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
    log_status "[FRONTEND] Installing node modules ..."
    (npm ci --registry=https://registry.npmjs.org --verbose --no-audit --no-fund --prefer-offline --legacy-peer-deps || npm install --no-audit --no-fund --legacy-peer-deps)
  fi
  log_status "[FRONTEND] Starting Quasar dev server on 0.0.0.0:5173"
  export NODE_OPTIONS="--max-old-space-size=${EC_NODE_MAX_OLD_SPACE_SIZE:-384}"
  ./node_modules/.bin/quasar dev -p 5173 --host 0.0.0.0 >>"$LOG_DIR/frontend.log" 2>&1 &
  FRONT_PID=$!
  log_status "[FRONTEND] Quasar dev server launched (pid=${FRONT_PID})"
else
  log_status "[FRONTEND] Skipped (EC_NO_FRONTEND=1)"
  FRONT_PID=0
fi

ensure_orator() {
  # Best-effort install with retries; never fail the entire entrypoint for transient issues.
  # Check import inside an if to avoid set -e aborts
  for attempt in 1 2 3; do
    if python3 - <<'PY'
import sys
try:
    import orator  # noqa: F401
    sys.exit(0)
except Exception:
    sys.exit(1)
PY
    then
      break
    fi
    log_status "[SETUP] Installing orator deps (no-deps mode)... (try ${attempt}/3)"
    python3 -m pip install --no-deps --no-input --timeout 180 --retries 3 \
      --cache-dir "${PIP_CACHE_DIR}" \
      backpack==0.1 simplejson faker lazy-object-proxy cleo==0.6.8 inflection pendulum==1.5.1 pytzdata python-dateutil >/dev/null 2>&1 || true
    log_status "[SETUP] Installing orator (no-deps)... (try ${attempt}/3)"
    if python3 -m pip install --no-deps --no-input --prefer-binary --timeout 180 --retries 3 \
      --cache-dir "${PIP_CACHE_DIR}" \
      orator==0.9.9 >/dev/null 2>&1; then
      hash -r
    else
      log_status "[SETUP][WARN] orator install attempt ${attempt} failed; retrying..."
      sleep 5
    fi
  done
  # Verify import now (do not exit hard; just warn)
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
    log_status "[SETUP][ERROR] orator import still failing after retries; continuing but backend may fail."
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
      log_status "[SETUP] orator CLI path: $(command -v orator)"
      python3 -m orator --version 2>/dev/null || true
    else
      log_status "[SETUP] orator CLI path: not found (using python -m orator)"
      python3 -m orator --version 2>/dev/null || true
    fi
  fi
}

ensure_watchfiles() {
  # Optional dependency for dev autoreload mode
  if [[ "${EC_BACKEND_AUTORELOAD:-0}" != "1" ]]; then
    return
  fi
  for attempt in 1 2; do
    if python3 - <<'PY'
import sys
try:
    import watchfiles  # noqa: F401
    sys.exit(0)
except Exception:
    sys.exit(1)
PY
    then
      break
    fi
    log_status "[SETUP] Installing watchfiles for backend autoreload (try ${attempt}/2)..."
    python3 -m pip install --no-deps --no-input --prefer-binary --timeout 180 --retries 2 \
      --cache-dir "${PIP_CACHE_DIR}" \
      "watchfiles==0.21.0" >/dev/null 2>&1 || true
    hash -r
  done
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
    log_status "[SETUP] Installing torchvision==$desired to match torch ${torch_ver:-unknown} (current: ${current:-none})..."
    python3 -m pip install --no-deps --no-input --prefer-binary \
      --cache-dir "${PIP_CACHE_DIR}" \
      "torchvision==$desired" && hash -r || true
  fi
}

# # 2) Ensure DB deps and run migrations (idempotent)
# if [[ "${EC_SKIP_TORCHVISION_COMPAT:-0}" == "1" ]]; then
#   log_status "[STEP] Skipping torchvision/torch compatibility (EC_SKIP_TORCHVISION_COMPAT=1)"
# else
#   log_status "[STEP] Ensuring torchvision/torch compatibility"
#   ensure_torchvision_compat
# fi
log_status "[STEP] Ensuring orator availability"
ensure_orator
log_status "[STEP] Switching to /root/src/backend/database"
cd /root/src/backend/database
log_status "[DB] Running migrations (Python API) ..."
(
  python3 -m pip show orator >/dev/null 2>&1 || \
    python3 -m pip install --no-deps --no-input --prefer-binary -q \
      --cache-dir "${PIP_CACHE_DIR}" \
      orator==0.9.9
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
  log_status "[DB][WARN] Python API migration failed; trying migration.sh if present"
  bash migration.sh || true
}

log_status "[DB] Ensuring database path is writable for host-side imports"
if [ -d "${DATA_ROOT}/database" ]; then
  chmod 777 "${DATA_ROOT}/database" 2>/dev/null || true
fi
if [ -f "${DATA_ROOT}/database/main.db" ]; then
  chmod 666 "${DATA_ROOT}/database/main.db" 2>/dev/null || true
fi

# 3) Start backend API
cd /root/src
log_status "[BACKEND] Starting Flask-SocketIO API on 0.0.0.0:5000"
# Prime backend log so it's never empty
echo "[BACKEND][BOOT] $(date -Is) starting backend.api.app --debug" >>"$LOG_DIR/backend.log" 2>/dev/null || true
# Double-check import before start; hard-fail if unresolved
ensure_orator
ensure_watchfiles
# if [[ "${EC_SKIP_TORCHVISION_COMPAT:-0}" != "1" ]]; then
#   ensure_torchvision_compat
# fi
BACKEND_ENTRY="${EC_BACKEND_ENTRY:-backend.api.app}"
log_status "[BACKEND] Entry: ${BACKEND_ENTRY}"
BACKEND_CMD=()
if [[ "${EC_BACKEND_AUTORELOAD:-0}" == "1" ]]; then
  log_status "[BACKEND] Autoreload enabled (watchfiles)"
  TARGET_CMD="bash -lc 'cd /root/src && python3 -u -m ${BACKEND_ENTRY} ${BACKEND_FLAGS}'"
  # Use watchfiles CLI in command mode; target is a single shell string (shlex-split inside watchfiles)
  BACKEND_CMD=(python3 -m watchfiles --filter python --target-type=command "$TARGET_CMD" /root/src/backend)
else
  BACKEND_CMD=(python3 -u -m "${BACKEND_ENTRY}" ${BACKEND_FLAGS})
fi
"${BACKEND_CMD[@]}" >>"$LOG_DIR/backend.log" 2>&1 &
BACK_PID=$!
log_status "[BACKEND] PID=${BACK_PID} CMD=${BACKEND_CMD[*]}"

# Probe simple health endpoint so status shows in docker logs (wait up to 100s)
HEALTH_OK=0
log_status "[HEALTH] Probing backend /api/healthz (max 100s)"
for i in $(seq 1 100); do
  if curl -fsS -m 2 http://127.0.0.1:5000/api/healthz >/dev/null 2>&1; then
    log_status "[HEALTH] backend /api/healthz OK"
    HEALTH_OK=1
    break
  else
    if [[ ${BACK_PID:-0} -gt 0 ]] && ! kill -0 "$BACK_PID" >/dev/null 2>&1; then
      log_status "[HEALTH][WARN] backend process exited early (pid=$BACK_PID)"
      break
    fi
    if (( i == 1 || i % 10 == 0 )); then
      log_status "[HEALTH] waiting backend (try $i)"
    fi
    sleep 1
  fi
done
if [[ "${HEALTH_OK}" != "1" ]]; then
  log_status "[HEALTH][ERROR] backend not ready after 100 attempts"
fi

# Handle termination and child exit
term() {
  log_status "[ENTRY] Shutting down..."
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
tail_targets=("$LOG_DIR/backend.log" "$LOG_DIR/frontend.log")
if [[ -n "${STATUS_LOG:-}" ]]; then
  touch "$STATUS_LOG" 2>/dev/null || true
  tail_targets+=("$STATUS_LOG")
fi
log_status "[LOGS] Aggregating backend/frontend/status logs to STDOUT"
tail -n +1 -q -F "${tail_targets[@]}" &
TAIL_PID=$!

# Wait for either to exit, then terminate the other
set +e
wait_pids=()
[[ ${BACK_PID:-0} -gt 0 ]] && wait_pids+=("${BACK_PID}")
[[ ${FRONT_PID:-0} -gt 0 ]] && wait_pids+=("${FRONT_PID}")
if ((${#wait_pids[@]})); then
  wait -n "${wait_pids[@]}"
  EC=$?
  ended_roles=()
  if [[ ${BACK_PID:-0} -gt 0 ]] && ! kill -0 "$BACK_PID" >/dev/null 2>&1; then
    ended_roles+=("backend")
  fi
  if [[ ${FRONT_PID:-0} -gt 0 ]] && ! kill -0 "$FRONT_PID" >/dev/null 2>&1; then
    ended_roles+=("frontend")
  fi
  if ((${#ended_roles[@]})); then
    role_label=$( (IFS=/; echo "${ended_roles[*]}") )
    log_status "[ENTRY] ${role_label} process exited (code=${EC})"
  fi
else
  log_status "[ENTRY][ERROR] No child processes to wait for."
  EC=1
fi
term
wait || true
log_status "[ENTRY] Exit code: ${EC}"
exit ${EC}
