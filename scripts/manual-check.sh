#!/usr/bin/env bash
# ===================================================================
# scripts/manual-check.sh  ──  매뉴얼 staleness 게이트
# -------------------------------------------------------------------
# (foundry-harness/templates/manual-check.sh.tmpl 을 EasyTrainer 의 home-next
#  매뉴얼 트리에 맞춰 인스턴스화한 것.)
#
# "기능 표면이 사용자 매뉴얼보다 새로우면 exit≠0" — "기능 업데이트마다 매뉴얼
# 갱신"을 *부탁(reminder hook)* 이 아니라 *게이트* 로 강제한다 (헌법 제3·4원칙:
# 규칙보다 인터페이스, 말보다 환경).
#
# 기존 .claude/hooks/manual-update-reminder.sh 는 편집 시 1회 권고만 한다(advisory).
# 이 스크립트는 그 권고가 무시됐는지 *기계적으로* 판정하는 hard gate 다.
#
# 빨간불 처방: manual-writer 서브에이전트(/manual-update) 로 매뉴얼 동기화 후 재검.
#
# 종료 코드:
#   0  = 매뉴얼이 기능 표면과 동기화됨 (또는 판정 불가 — 자율 루프 오탐 방지)
#   1  = 기능 표면이 매뉴얼보다 새로움 — 동기화 필요
#
# 사용:
#   bash scripts/manual-check.sh           # 사람용 출력
#   bash scripts/manual-check.sh --quiet   # 결과 줄만
# ===================================================================
set -uo pipefail
cd "$(git rev-parse --show-toplevel 2>/dev/null || dirname "$(dirname "$0")")"

QUIET=0
[ "${1:-}" = "--quiet" ] && QUIET=1
say() { [ "$QUIET" -eq 1 ] || echo "$@"; }

# 매뉴얼 트리 (사람용 + manual-writer 가 갱신).
MANUAL_TREE="home-next/src/app/docs/_content/ko"

# 이 매뉴얼이 책임지는 *기능 표면*. 단일 출처는 manual-writer.md + manual-update-reminder.sh.
# (docs/HARNESS.md 동기화 불변식 #1 — 4곳이 일치해야 함.)
GLOBS=(
  "frontend/src/pages/v2"
  "frontend/src/components/v2"
  "release/ui"
  "backend/api/routes"
  "modules/*/module.json"
  "README.md"
)

if ! git rev-parse --git-dir >/dev/null 2>&1; then
  say "SKIP: git 저장소 아님 — staleness 판정 불가"; exit 0
fi
if [ ! -d "$MANUAL_TREE" ]; then
  say "SKIP: 매뉴얼 트리 없음 ($MANUAL_TREE)"; exit 0
fi

# 매뉴얼 트리를 마지막으로 건드린 커밋.
manual_commit="$(git log -1 --format=%H -- "$MANUAL_TREE" 2>/dev/null || true)"
if [ -z "$manual_commit" ]; then
  say "SKIP: 매뉴얼이 아직 커밋되지 않음 — 판정 불가"; exit 0
fi

# 그 커밋 이후 기능 표면이 바뀌었나 (커밋된 변경 + 워킹트리 미커밋 변경 모두 포함).
changed="$(git diff --name-only "$manual_commit" -- "${GLOBS[@]}" 2>/dev/null || true)"

if [ -n "$changed" ]; then
  say "FAIL: 기능 표면이 매뉴얼보다 새로움. 아래 변경을 매뉴얼에 반영하라"
  say "      (manual-writer 서브에이전트 또는 /manual-update):"
  say "$changed" | sed 's/^/  - /'
  exit 1
fi
say "OK: 매뉴얼($MANUAL_TREE)이 기능 표면과 동기화됨"
exit 0
