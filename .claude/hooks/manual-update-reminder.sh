#!/usr/bin/env bash
# ===================================================================
# manual-update-reminder.sh  ──  PostToolUse hook
# -------------------------------------------------------------------
# Triggered after Write/Edit/MultiEdit. If the edited file falls inside
# the manual-writer agent's PROACTIVE trigger paths, emit a one-time
# (per session) reminder via additionalContext telling the model to
# consider syncing the Easy Trainer end-user manual when the current
# task is complete.
#
# Hook input (stdin JSON):
#   { "session_id": "...", "tool_name": "Edit",
#     "tool_input": { "file_path": "/abs/path/...", ... }, ... }
#
# Hook output (stdout JSON):
#   (nothing) on no-op
#   { "hookSpecificOutput": { "hookEventName": "PostToolUse",
#       "additionalContext": "..." } } on first match per session
#
# Notes:
#   - Uses python3 (always present on this host) instead of jq so the
#     hook works without extra dependencies. See feedback_install_policy
#     in MEMORY.md — we don't install tooling on the host.
#   - Path patterns must mirror the manual-writer agent's PROACTIVE
#     trigger list.
# ===================================================================
set -euo pipefail

PROJECT_ROOT="/home/airlab/Easy-Collector"

# Single python invocation reads stdin, decides whether to emit the
# reminder, manages the per-session dedup marker, and writes the output
# JSON. Keeping it self-contained avoids bash-side string fragility.
INPUT=$(cat)
INPUT="$INPUT" PROJECT_ROOT="$PROJECT_ROOT" python3 - <<'PY'
import json, os, sys, fnmatch, pathlib

raw = os.environ.get("INPUT", "")
project_root = os.environ.get("PROJECT_ROOT", "")

try:
    data = json.loads(raw)
except Exception:
    sys.exit(0)

tool_input = data.get("tool_input") or {}
file_path = tool_input.get("file_path") or ""
session_id = data.get("session_id") or "unknown"

if not file_path:
    sys.exit(0)

# Normalize to project-relative path. Edits outside the project root
# (e.g. /opt/easytrainer/project mirror used by the running container)
# also count if they map to the same logical tree — but we ignore them
# here because manual sync is driven by the source-of-truth checkout.
if file_path.startswith(project_root + "/"):
    rel = file_path[len(project_root) + 1:]
elif file_path.startswith("/"):
    sys.exit(0)
else:
    rel = file_path

# Trigger globs — mirror manual-writer agent's PROACTIVE list.
TRIGGERS = [
    "frontend/src/pages/v2/**",
    "frontend/src/components/v2/**",
    "release/ui/**",
    "backend/api/routes/**",
    "modules/*/module.json",
    "README.md",
]

def matches(rel_path: str) -> bool:
    p = pathlib.PurePosixPath(rel_path)
    for pat in TRIGGERS:
        # fnmatch handles * but not **; emulate ** as "any number of segments".
        if "**" in pat:
            prefix = pat.split("/**", 1)[0]
            if rel_path == prefix or rel_path.startswith(prefix + "/"):
                return True
        else:
            if fnmatch.fnmatchcase(rel_path, pat):
                return True
            # also try matching just the basename against patterns without /
            if "/" not in pat and fnmatch.fnmatchcase(p.name, pat):
                return True
    return False

if not matches(rel):
    sys.exit(0)

# Per-session dedup marker. /tmp survives until reboot; session ids are
# uuid-shaped so collisions are negligible. We mark on first hit so
# subsequent edits in the same session don't re-emit.
marker = f"/tmp/claude-manual-reminder-{session_id}.flag"
if os.path.exists(marker):
    sys.exit(0)
try:
    pathlib.Path(marker).touch()
except Exception:
    # Even if we can't write the marker, still emit once — better to
    # over-notify than miss the reminder entirely.
    pass

reminder = (
    f"📘 Manual sync 알림 (세션당 1회): 방금 편집한 `{rel}` 는 "
    "manual-writer agent 의 PROACTIVE 트리거 경로입니다. "
    "현재 작업이 마무리될 무렵 /manual-update 스킬을 호출하거나 "
    "manual-writer subagent 를 spawn 해서 end-user manual "
    "(home-next/src/app/docs/_content/ko/**/*.mdx) 와의 동기화를 "
    "검토하세요. 변경이 사소하거나 manual 영향 없다고 판단되면 "
    "그렇게 보고하면 됩니다."
)
print(json.dumps({
    "hookSpecificOutput": {
        "hookEventName": "PostToolUse",
        "additionalContext": reminder,
    },
}))
PY
