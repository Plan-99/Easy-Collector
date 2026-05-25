#!/usr/bin/env bash
# ===================================================================
# frontend-i18n-check.sh  ──  PostToolUse hook
# -------------------------------------------------------------------
# Triggered after Write/Edit/MultiEdit on frontend files. Extracts the
# i18n keys used in the edited file ($t('key') / t('key')) and checks
# that every key exists in BOTH locale bundles:
#     frontend/src/i18n/ko-KR/index.js
#     frontend/src/i18n/en-US/index.js
#
# Any missing keys cause a non-blocking reminder to be injected via
# additionalContext so the model knows to add them before considering
# the change "done". Unlike the manual-update reminder this fires
# every relevant edit (not just once per session) because i18n drift
# accumulates per-file, not per-session.
#
# No dedup: if there are still missing keys after a fix attempt, we
# want the model to see them again on the next edit.
# ===================================================================
set -euo pipefail

PROJECT_ROOT="/home/airlab/Easy-Collector"

INPUT=$(cat)
INPUT="$INPUT" PROJECT_ROOT="$PROJECT_ROOT" python3 - <<'PY'
import json, os, re, sys, pathlib

raw = os.environ.get("INPUT", "")
project_root = os.environ.get("PROJECT_ROOT", "")

try:
    data = json.loads(raw)
except Exception:
    sys.exit(0)

tool_input = data.get("tool_input") or {}
file_path = tool_input.get("file_path") or ""
if not file_path:
    sys.exit(0)

# Only react to frontend source files. Locale bundle edits and non-vue/js/ts
# files are ignored — otherwise editing the locales themselves would trigger
# noise.
if not file_path.startswith(project_root + "/frontend/src/"):
    sys.exit(0)
if not file_path.endswith((".vue", ".js", ".ts", ".jsx", ".tsx")):
    sys.exit(0)
# Skip locale bundles to avoid a feedback loop when the model is fixing keys.
if "/frontend/src/i18n/" in file_path:
    sys.exit(0)

# Read the edited file. If the file was just deleted / renamed we skip.
try:
    src = pathlib.Path(file_path).read_text(encoding="utf-8", errors="ignore")
except Exception:
    sys.exit(0)

# Extract i18n keys. Patterns supported:
#     $t('key.path')   (template)
#     t('key.path')    (script setup with useI18n() destructure)
#     $t("key")        (double-quoted variant)
#     t("key")
# We deliberately don't try to evaluate computed expressions — only literal
# string keys are checked. Anything else is the developer's problem to verify.
key_re = re.compile(r"""(?:^|[\s\(\>=,])\$?t\(\s*['"]([A-Za-z0-9_.]+)['"]""")
used_keys = sorted(set(key_re.findall(src)))
if not used_keys:
    sys.exit(0)

# Load both locale bundles by extracting top-level keys from the default
# export object. The bundles are plain JS objects so a regex pass is enough —
# we just need the set of keys, not values.
def load_locale_keys(path: str) -> set[str]:
    try:
        text = pathlib.Path(path).read_text(encoding="utf-8", errors="ignore")
    except Exception:
        return set()
    # match e.g. `  appTitle: '...'` or `  augHsv: \"...\"`. Allow nested keys
    # (object literals) by matching anything that looks like an identifier
    # followed by `:`. The check intentionally ignores keys nested inside
    # other objects since the codebase uses flat keys today.
    return set(re.findall(r"^\s{2,4}([A-Za-z0-9_]+)\s*:", text, re.MULTILINE))

ko_keys = load_locale_keys(f"{project_root}/frontend/src/i18n/ko-KR/index.js")
en_keys = load_locale_keys(f"{project_root}/frontend/src/i18n/en-US/index.js")

if not ko_keys or not en_keys:
    # Locale files missing or unreadable — don't block, just bail.
    sys.exit(0)

missing_ko = [k for k in used_keys if k not in ko_keys]
missing_en = [k for k in used_keys if k not in en_keys]
if not missing_ko and not missing_en:
    sys.exit(0)

rel = file_path[len(project_root) + 1:]
lines = [
    f"🌐 i18n 누락 키 감지 — `{rel}` 에서 사용 중인 키 중 일부가 로케일 번들에 없습니다.",
    "프론트엔드 변경은 ko-KR / en-US 두 곳 모두에 i18n 키를 추가하는 것이 정책입니다.",
]
if missing_ko:
    lines.append(f"  • ko-KR 누락 ({len(missing_ko)}): " + ", ".join(missing_ko[:20]) + (" …" if len(missing_ko) > 20 else ""))
if missing_en:
    lines.append(f"  • en-US 누락 ({len(missing_en)}): " + ", ".join(missing_en[:20]) + (" …" if len(missing_en) > 20 else ""))
lines.append("→ `frontend/src/i18n/ko-KR/index.js` 와 `frontend/src/i18n/en-US/index.js` 양쪽에 같은 키로 추가하세요. fallback (`$t('x') || 'default'`) 만으로 끝내지 말 것.")

print(json.dumps({
    "hookSpecificOutput": {
        "hookEventName": "PostToolUse",
        "additionalContext": "\n".join(lines),
    },
}))
PY
