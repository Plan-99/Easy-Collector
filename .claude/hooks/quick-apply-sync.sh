#!/usr/bin/env bash
#
# PostToolUse hook — auto-sync edited backend/frontend sources into the running
# runtime project (/opt/easytrainer/project), which the dev containers
# bind-mount (`/opt/easytrainer/project/backend -> /root/backend`,
# `.../frontend -> /app`). The repo itself is NOT bind-mounted, so without this
# sync, code edits never reach the containers and changes can't be verified
# (here or in another session). Mirrors `bash scripts/quick_apply.sh`.
#
# Only fires for files under backend/ or frontend/. A flock serializes the
# background syncs so rapid multi-edits don't run rsync concurrently; the last
# run picks up every change on disk. Parses stdin JSON with python3 (jq is not
# installed on this host) — same convention as the other hooks here.
set -uo pipefail

INPUT="$(cat)"
f="$(INPUT="$INPUT" python3 - <<'PY' 2>/dev/null
import json, os
try:
    d = json.loads(os.environ.get("INPUT") or "{}")
except Exception:
    d = {}
ti = d.get("tool_input") or {}
tr = d.get("tool_response") or {}
print(ti.get("file_path") or tr.get("filePath") or "")
PY
)"
[ -z "$f" ] && exit 0

case "$f" in
  *backend/*|*frontend/*) ;;
  *) exit 0 ;;
esac

root="${CLAUDE_PROJECT_DIR:-/home/airlab/Easy-Collector}"
cd "$root" 2>/dev/null || exit 0
[ -f scripts/quick_apply.sh ] || exit 0

exec 9>/tmp/.ec-quick-apply.lock
flock 9
bash scripts/quick_apply.sh ./ /opt/easytrainer/project >/dev/null 2>&1 || true
exit 0
