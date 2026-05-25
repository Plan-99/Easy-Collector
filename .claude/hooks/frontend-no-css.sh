#!/usr/bin/env bash
# ===================================================================
# frontend-no-css.sh  ──  PostToolUse hook
# -------------------------------------------------------------------
# Triggered after Write/Edit/MultiEdit on a frontend file. If the edit
# introduced or kept a `<style>` block in a Vue SFC, emit a reminder
# that the project's policy is "no CSS — use Quasar utility classes
# and components". The hook also flags top-level `.css` / `.scss` /
# `.sass` files outside the canonical quasar.variables / app.scss
# locations.
#
# Rationale: the codebase standard, set by the user, is to lean on
# Quasar primitives (q-card, q-card-section, q-separator, row/col,
# q-pa-*, bg-*, text-*) instead of writing custom CSS rules. Dynamic
# positioning (drag overlays, animations) can still use inline
# `:style="..."` bindings — those are not CSS rules.
#
# Always fires (no per-session dedup) — a stray `<style>` block is
# usually an active mistake, not background noise.
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

# Only react to frontend source. Skip the curated stylesheets that the
# build system itself requires (quasar.variables.scss, app.scss).
if not file_path.startswith(project_root + "/frontend/src/"):
    sys.exit(0)
rel = file_path[len(project_root) + 1:]
allowed = (
    "frontend/src/css/quasar.variables.scss",
    "frontend/src/css/app.scss",
)
if rel in allowed:
    sys.exit(0)

try:
    src = pathlib.Path(file_path).read_text(encoding="utf-8", errors="ignore")
except Exception:
    sys.exit(0)

violations = []

# Vue SFCs: flag any <style> block. Scoped or not — the policy is the
# same. Exception: a completely empty / whitespace-only block (e.g. a
# leftover that's about to be removed) is benign.
if file_path.endswith(".vue"):
    for m in re.finditer(r"<style\b[^>]*>(.*?)</style>", src, flags=re.DOTALL | re.IGNORECASE):
        body = m.group(1).strip()
        if body:
            line_no = src[: m.start()].count("\n") + 1
            head = re.sub(r"\s+", " ", body[:80])
            violations.append(
                f"line {line_no}: `<style>` block ({len(body)} chars). "
                f"Excerpt: “{head}{'…' if len(body) > 80 else ''}”"
            )

# Loose .css / .scss / .sass written under src/ outside the curated paths.
if file_path.endswith((".css", ".scss", ".sass")) and rel not in allowed:
    violations.append(f"stylesheet file `{rel}` outside the allowed paths "
                      f"({', '.join(allowed)}).")

if not violations:
    sys.exit(0)

lines = [
    f"🎨 CSS policy 위반 가능성 — `{rel}` 에 custom CSS 가 있습니다.",
    "이 프로젝트는 Quasar utility (q-pa-*, q-mb-*, bg-*, text-*, row/col 등) 와 "
    "Quasar 컴포넌트 (q-card, q-card-section, q-separator, q-bar, q-toolbar …) "
    "로 스타일을 처리하는 것이 정책입니다. dynamic 위치 (drag overlay 등) 는 "
    "`:style=\"...\"` 인라인 바인딩으로 처리하세요.",
]
for v in violations[:10]:
    lines.append(f"  • {v}")
if len(violations) > 10:
    lines.append(f"  (and {len(violations) - 10} more)")
lines.append(
    "→ `<style>` 블록을 제거하고 Quasar 클래스/컴포넌트로 옮기세요. "
    "필요한 경우 quasar.variables.scss 의 색상 토큰만 수정."
)

print(json.dumps({
    "hookSpecificOutput": {
        "hookEventName": "PostToolUse",
        "additionalContext": "\n".join(lines),
    },
}))
PY
