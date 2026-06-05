#!/usr/bin/env bash
# ===================================================================
# release_module.sh
# -------------------------------------------------------------------
# 단일 EasyTrainer 모듈을 마켓(Plan-99/Easy-Trainer-Modules)에 릴리즈한다.
#
# 안전성: 격리된 git worktree 를 origin/module_up 위에 띄워서, 현재 작업
# 트리와 진행 중인 다른 작업은 절대 건드리지 않는다. 해당 모듈 폴더만
# 커밋한다. (modules-release.yml CI 는 main/module_up push 의 modules/**
# 변경을 HEAD~1..HEAD diff 로 감지한다.)
#
# 사용:
#   scripts/release_module.sh <category/name> ["commit message"] [--no-watch]
#   예) scripts/release_module.sh robots/omy "ROBOTIS OMY 6-DOF 추가"
#
# 전제: gh CLI 로그인(PUBLIC_REPO_TOKEN 은 CI secret), origin push 권한.
# ===================================================================
set -euo pipefail

MOD="${1:-}"; MSG="${2:-}"; WATCH=1
for a in "$@"; do [ "$a" = "--no-watch" ] && WATCH=0; done
[ -z "$MOD" ] && { echo "usage: $0 <category/name> [\"msg\"] [--no-watch]"; exit 2; }

REPO="$(git -C "$(dirname "$0")/.." rev-parse --show-toplevel)"
MODDIR="$REPO/modules/$MOD"
MJ="$MODDIR/module.json"
REL_REPO="Plan-99/Easy-Trainer-Modules"
[ -f "$MJ" ] || { echo "[release] module.json not found: $MJ"; exit 2; }

ID=$(python3 -c "import json;print(json.load(open('$MJ'))['id'])")
VER=$(python3 -c "import json;print(json.load(open('$MJ'))['version'])")
TAG="module-${ID}-v${VER}"
echo "[release] module=$MOD  id=$ID  version=$VER  tag=$TAG"

# 1) 매니페스트 스키마 검증
python3 "$REPO/scripts/validate_modules.py" "$MJ"

# 2) 이미 릴리즈된 버전인지 (같은 버전이면 CI 가 create 를 스킵함)
if gh release view "$TAG" --repo "$REL_REPO" >/dev/null 2>&1; then
  echo "[release] $TAG 이(가) 이미 $REL_REPO 에 존재합니다."
  echo "          $MJ 의 'version' 을 올린 뒤 다시 실행하세요."
  exit 3
fi

# 3) origin/module_up 위에 격리 worktree
git -C "$REPO" fetch origin --quiet
WT=$(mktemp -d /tmp/etmod_release.XXXXXX)
cleanup(){ git -C "$REPO" worktree remove "$WT" --force >/dev/null 2>&1 || true; }
trap cleanup EXIT
git -C "$REPO" worktree add -B module_up "$WT" origin/module_up --quiet
echo "[release] worktree: $WT (module_up @ origin/module_up)"

# 4) 현재 디스크의 모듈 내용만 스테이징
mkdir -p "$WT/modules/$(dirname "$MOD")"
rm -rf "$WT/modules/$MOD"
cp -r "$MODDIR" "$WT/modules/$MOD"
git -C "$WT" add "modules/$MOD"
STRAY=$(git -C "$WT" diff --cached --name-only | grep -v "^modules/$MOD/" || true)
[ -n "$STRAY" ] && { echo "[release] ABORT: 모듈 외 파일이 스테이징됨:"; echo "$STRAY"; exit 4; }
if git -C "$WT" diff --cached --quiet; then
  echo "[release] origin/module_up 대비 변경 없음 — 이미 최신."; exit 5
fi

# 5) 커밋 + 푸시 (module_up 은 fast-forward)
[ -z "$MSG" ] && MSG="release"
git -C "$WT" commit --quiet -m "$ID $VER: $MSG

Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>"
SHA=$(git -C "$WT" rev-parse HEAD)
git -C "$WT" push origin module_up
echo "[release] pushed module_up @ ${SHA:0:9}"

# 6) CI watch
if [ "$WATCH" = 1 ]; then
  sleep 6
  RUN=$(gh run list -w "Package Modules" -L 8 --json databaseId,headSha \
        -q ".[] | select(.headSha==\"$SHA\") | .databaseId" | head -1 || true)
  [ -z "$RUN" ] && RUN=$(gh run list -w "Package Modules" -L1 --json databaseId -q '.[0].databaseId')
  echo "[release] watching Package Modules run $RUN"
  gh run watch "$RUN" --exit-status || { echo "[release] CI 실패 — Actions 로그 확인"; exit 6; }
fi

# 7) 릴리즈된 tar.gz 검증 (CLAUDE.md 모듈 배포 규칙 4단계)
sleep 3
gh release view "$TAG" --repo "$REL_REPO" >/dev/null 2>&1 || {
  echo "[release] WARNING: CI 후에도 $TAG 릴리즈 없음 — 로그 확인"; exit 7; }
TMP=$(mktemp -d)
gh release download "$TAG" --repo "$REL_REPO" --dir "$TMP" >/dev/null 2>&1 || true
TAR="$TMP/module-${ID}-${VER}.tar.gz"
if [ -f "$TAR" ]; then
  python3 - "$TAR" "$ID" "$VER" <<'PY'
import sys,tarfile,json
tar,eid,ever=sys.argv[1],sys.argv[2],sys.argv[3]
with tarfile.open(tar) as t:
    mj=[m for m in t.getmembers() if m.name.endswith('module.json')][0]
    m=json.loads(t.extractfile(mj).read()); n=len(t.getnames())
assert m['id']==eid and str(m['version'])==ever, (m['id'],m['version'])
print(f"[verify] OK  id={m['id']}  version={m['version']}  files={n}")
PY
else
  echo "[verify] asset 다운로드 실패 — 수동 확인 필요: gh release view $TAG --repo $REL_REPO"
fi
rm -rf "$TMP"
echo "[release] DONE → $REL_REPO  $TAG"
echo "[release] 다음: scripts/upsert_module_catalog.sh $MOD --prod  (홈페이지 카탈로그 노출)"
