#!/usr/bin/env bash
# ===================================================================
# frontend-ui-test-reminder.sh  ──  PostToolUse hook
# -------------------------------------------------------------------
# Triggered after Write/Edit/MultiEdit on a frontend UI file. Emits a
# one-time (per session) reminder that frontend feature work is not
# "done" until a Playwright UI test (via the project's playwright-skill)
# has been run and any issues observed have been fixed.
#
# Trigger paths — user-facing UI surface only:
#   frontend/src/pages/**
#   frontend/src/components/**
#   release/ui/**            (launcher UI is also user-visible)
#
# Other frontend paths (utils, i18n, boot, router, stores) are
# infrastructure — they don't add user-visible features by themselves,
# so we skip them to avoid noise.
#
# Dedup: per-session marker. The model needs to be reminded once per
# feature work session, not once per edit.
# ===================================================================
set -euo pipefail

PROJECT_ROOT="/home/airlab/Easy-Collector"

INPUT=$(cat)
INPUT="$INPUT" PROJECT_ROOT="$PROJECT_ROOT" python3 - <<'PY'
import json, os, sys, pathlib

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

# Normalize to project-relative path. Edits outside the source tree (mirror
# under /opt/easytrainer/project, etc.) are ignored — source-of-truth only.
if file_path.startswith(project_root + "/"):
    rel = file_path[len(project_root) + 1:]
elif file_path.startswith("/"):
    sys.exit(0)
else:
    rel = file_path

UI_PREFIXES = (
    "frontend/src/pages/",
    "frontend/src/components/",
    "release/ui/",
)
matched = any(rel.startswith(p) for p in UI_PREFIXES)
if not matched:
    sys.exit(0)

# Dedup per session — touched UI once, the reminder fires once. The model
# can decide WHEN to actually run the test (typically at the end of the
# task), not on every edit.
marker = f"/tmp/claude-ui-test-reminder-{session_id}.flag"
if os.path.exists(marker):
    sys.exit(0)
try:
    pathlib.Path(marker).touch()
except Exception:
    pass

reminder = (
    f"🧪 UI test 알림 (세션당 1회): 방금 `{rel}` 를 수정했습니다. "
    "이 프로젝트 정책은 **기능 추가/변경은 Playwright UI 테스트로 검증까지 마쳐야 완료** "
    "입니다. 현재 task 가 끝날 무렵 `playwright-skill` 을 호출해서:\n"
    "  1. 변경한 UI 가 실제로 의도대로 렌더되는지 확인 (스크린샷 캡처 → Read 로 직접 확인)\n"
    "  2. 주요 사용자 인터랙션 시나리오 (클릭, 입력, 결과 확인) 실행\n"
    "  3. 콘솔 에러 / 네트워크 에러 / 레이아웃 깨짐 등 관찰\n"
    "  4. 발견된 이슈가 있으면 그 자리에서 수정 후 재테스트\n"
    "Skill 경로: `.claude/skills/playwright-skill/SKILL.md` 참고. "
    "스크린샷은 캡처 직후 Read 로 검증 (memory: feedback_verify_screenshots)."
)

print(json.dumps({
    "hookSpecificOutput": {
        "hookEventName": "PostToolUse",
        "additionalContext": reminder,
    },
}))
PY
