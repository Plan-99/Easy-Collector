#!/usr/bin/env bash
# verify_deb.sh — fresh-install 빌드 검증 파이프라인.
#
# deb 파일을 받아서 (1) 내용물에 세션 패치들이 baked-in 되어있는지 정적 검증
# 하고, (2) 도커 이미지를 rebuild_images.sh 로 빌드하고, (3) installer.py 가
# 부르는 것과 동일한 방식으로 마이그레이션을 ephemeral container 에서 실행해
# `from pyarmor_runtime_000000 import __pyarmor__` top-level import 가 깨지지
# 않는지 확인한다.
#
# 단계 어느 한 곳이라도 실패하면 종료코드 != 0 와 함께 분류된 에러 토큰을
# 마지막 줄에 stdout 으로 찍는다. /verify-deb 슬래시 커맨드는 그 토큰을 보고
# 과거 commit 의 fix 패턴을 안내한다.
#
# Usage:
#   bash scripts/verify_deb.sh <deb-path-or-url> [--skip-build] [--skip-migrate]
#
# Exit codes:
#   0  — 모든 단계 통과
#   10 — deb 다운로드/추출 실패
#   11 — deb 내용 정적 검증 실패 (압축 풀린 내용에 세션 패치 누락)
#   20 — 도커 이미지 빌드 실패
#   30 — 마이그레이션 단계 실패 (pyarmor / sqlite / import 등)
#   99 — 알 수 없는 실패

set -u
set -o pipefail

DEB_ARG="${1:-}"
SKIP_BUILD=0
SKIP_MIGRATE=0
for arg in "${@:2}"; do
  case "$arg" in
    --skip-build) SKIP_BUILD=1 ;;
    --skip-migrate) SKIP_MIGRATE=1 ;;
    *) echo "[VERIFY] unknown flag: $arg" >&2; exit 99 ;;
  esac
done

if [ -z "$DEB_ARG" ]; then
  echo "Usage: $0 <deb-path-or-url> [--skip-build] [--skip-migrate]" >&2
  exit 99
fi

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
WORK="$(mktemp -d -t etverify.XXXXXX)"
trap 'rm -rf "$WORK"' EXIT

log()  { printf '[VERIFY] %s\n' "$*"; }
fail() { printf '[VERIFY][FAIL] %s\n' "$*" >&2; echo "REASON=$1"; exit "$2"; }

# ── 1) deb 확보 ──────────────────────────────────────────────────────
DEB_LOCAL="$WORK/pkg.deb"
if [[ "$DEB_ARG" =~ ^https?:// ]]; then
  log "downloading $DEB_ARG"
  if ! curl -fsSL "$DEB_ARG" -o "$DEB_LOCAL"; then
    fail "deb_download_failed: $DEB_ARG" 10
  fi
elif [ -f "$DEB_ARG" ]; then
  cp "$DEB_ARG" "$DEB_LOCAL"
else
  fail "deb_not_found: $DEB_ARG" 10
fi

log "extracting $(basename "$DEB_ARG")"
EXTRACT="$WORK/extract"
mkdir -p "$EXTRACT"
if ! dpkg-deb -x "$DEB_LOCAL" "$EXTRACT" 2>"$WORK/extract.err"; then
  cat "$WORK/extract.err" >&2
  fail "deb_extract_failed" 10
fi

# 페이로드 위치는 build.sh 의 PAYLOAD_DIR 와 동일해야 한다.
PAYLOAD="$EXTRACT/usr/share/easytrainer-project"
if [ ! -d "$PAYLOAD" ]; then
  fail "payload_dir_missing: expected /usr/share/easytrainer-project" 11
fi

# ── 2) 정적 검증: 세션 패치들 baked-in ───────────────────────────────
log "static check: session patches baked into payload"

# 2a) rebuild_images.sh 가 들어가 있어야 함 (build.sh whitelist)
if [ ! -x "$PAYLOAD/scripts/rebuild_images.sh" ] && [ ! -f "$PAYLOAD/scripts/rebuild_images.sh" ]; then
  fail "missing_rebuild_images_sh: scripts/rebuild_images.sh not in payload" 11
fi

# 2b) docker-compose 파일 PYTHONPATH 에 /root/backend 포함
for f in docker-compose.yml docker-compose.cpu.yml; do
  if [ ! -f "$PAYLOAD/$f" ]; then
    fail "missing_compose: $f" 11
  fi
  if ! grep -q 'PYTHONPATH=/root:/root/backend:/root/backend/lerobot/src' "$PAYLOAD/$f"; then
    fail "compose_pythonpath_unpatched: $f missing /root/backend in PYTHONPATH" 11
  fi
done

# 2c) installer.py 의 docker run 마이그레이션 PYTHONPATH (PyInstaller 바이너리는
#     payload 에 들어가지 않으므로 dev tree 의 release/ui/installer.py 를 검사)
if [ -f "$ROOT/release/ui/installer.py" ]; then
  if ! grep -q 'PYTHONPATH=/root:/root/backend:/root/backend/lerobot/src' "$ROOT/release/ui/installer.py"; then
    fail "installer_pythonpath_unpatched: release/ui/installer.py docker run 의 PYTHONPATH 가 /root/backend 미포함" 11
  fi
fi

# 2d) ros2/Dockerfile.base 에 kakao 미러 redirect 가 없어야 한다.
#     주석 안의 historical mention 은 무시하고 RUN/sed 같은 active 사용만 잡는다.
if awk '!/^[[:space:]]*#/' "$PAYLOAD/ros2/Dockerfile.base" 2>/dev/null | grep -q 'mirror.kakao.com'; then
  fail "kakao_mirror_present: ros2/Dockerfile.base 가 다시 kakao 미러를 쓰고 있음 — held broken packages 재현 위험" 11
fi

# 2e) PyArmor 런타임이 backend 아래에 들어가 있어야 한다 (deploy.yml 산출물)
if [ ! -d "$PAYLOAD/backend/pyarmor_runtime_000000" ]; then
  fail "pyarmor_runtime_missing: backend/pyarmor_runtime_000000 not bundled — 난독화 빌드 산출물 누락" 11
fi

# 2f) backend/__init__.py 가 실제로 obfuscated 되어 있어야 한다 (트라이얼 size cap 회피 확인)
if ! head -1 "$PAYLOAD/backend/__init__.py" | grep -q 'Pyarmor'; then
  fail "backend_init_not_obfuscated: backend/__init__.py 가 평문 — PyArmor 단계 실패" 11
fi

log "static check: OK"

# ── 3) 도커 이미지 빌드 ─────────────────────────────────────────────
if [ "$SKIP_BUILD" = "1" ]; then
  log "build step skipped (--skip-build)"
else
  log "running rebuild_images.sh all (extracted payload)"
  BUILD_LOG="$WORK/build.log"
  pushd "$PAYLOAD" >/dev/null
  if bash "$PAYLOAD/scripts/rebuild_images.sh" all >"$BUILD_LOG" 2>&1; then
    popd >/dev/null
    log "image build: OK"
  else
    rc=$?
    popd >/dev/null
    tail -40 "$BUILD_LOG" >&2
    # 알려진 에러 패턴 분류
    if grep -q 'held broken packages\|Unable to correct problems' "$BUILD_LOG"; then
      fail "apt_held_broken: ros2 base 빌드에서 strict-pin 충돌 — Dockerfile.base 의 미러/upgrade 흐름 확인 (commit a3661b8 참고)" 20
    fi
    if grep -qE 'Cannot connect to the Docker daemon|permission denied while trying to connect to the Docker daemon' "$BUILD_LOG"; then
      fail "docker_daemon_unavailable: 도커 데몬에 접근 못함 — 호스트 docker 권한 확인" 20
    fi
    if grep -q 'no such file or directory' "$BUILD_LOG"; then
      fail "build_path_missing: 빌드 컨텍스트 파일 누락 — release/build.sh 의 RSYNC_EXCLUDES 확인" 20
    fi
    fail "image_build_failed (exit=$rc) — tail of $BUILD_LOG 위에 출력" 20
  fi
fi

# ── 4) 마이그레이션 (ephemeral container) ──────────────────────────
if [ "$SKIP_MIGRATE" = "1" ]; then
  log "migrate step skipped (--skip-migrate)"
  log "ALL CHECKS PASSED (build verified, migrate skipped)"
  echo "REASON=ok_partial"
  exit 0
fi

# installer.py 가 실행하는 것과 동일한 호출을 재현한다. 단, 호스트의
# /opt/easytrainer 를 건드리지 않도록 임시 디렉토리에 마운트한다.
ET_TMP="$WORK/et-runtime"
mkdir -p "$ET_TMP/database"
log "running migration in ephemeral container (uses payload backend/)"

MIGRATE_CMD='cd /root && python3 -c "import sys; sys.path.insert(0, '\''/root'\''); from backend.database.models import create_tables; create_tables(); print('\''[DB] Migration complete'\'')"'

MIGRATE_LOG="$WORK/migrate.log"
if docker run --rm \
    -v "$PAYLOAD/backend:/root/backend" \
    -v "$ET_TMP:/opt/easytrainer" \
    -e PYTHONPATH=/root:/root/backend:/root/backend/lerobot/src \
    -e EASYTRAINER_DATA_DIR=/opt/easytrainer \
    -e EASYTRAINER_CONFIG_PATH=/opt/easytrainer/config.json \
    --entrypoint bash \
    easytrainer-backend:latest \
    -c "$MIGRATE_CMD" >"$MIGRATE_LOG" 2>&1; then
  log "migration: OK"
  log "  DB created at: $ET_TMP/database/main.db"
  if [ -f "$ET_TMP/database/main.db" ]; then
    log "  size: $(du -h "$ET_TMP/database/main.db" | awk '{print $1}')"
  fi
else
  rc=$?
  cat "$MIGRATE_LOG" >&2
  if grep -q "No module named 'pyarmor_runtime_000000'" "$MIGRATE_LOG"; then
    fail "pyarmor_runtime_not_found: PYTHONPATH 에 /root/backend 미포함 — release/ui/installer.py 또는 compose 의 docker run 패치 (commit d35cc46 참고)" 30
  fi
  if grep -qE 'pyarmor_runtime\.so:.*undefined symbol|_PyFloat_Pack8|Py_(Decref|Incref)|GLIBC_' "$MIGRATE_LOG"; then
    fail "pyarmor_runtime_abi_mismatch: PyArmor .so 가 backend 이미지의 Python ABI 와 불일치 — .github/workflows/deploy.yml 의 setup-python 버전을 backend/Dockerfile.base 의 베이스 OS Python 버전과 맞출 것 (Ubuntu 24.04 → 3.12)" 30
  fi
  if grep -q "No module named 'lerobot'" "$MIGRATE_LOG"; then
    fail "lerobot_not_found: PYTHONPATH 에 /root/backend/lerobot/src 미포함" 30
  fi
  if grep -qE "No module named '[^']+'" "$MIGRATE_LOG"; then
    MOD=$(grep -oE "No module named '[^']+'" "$MIGRATE_LOG" | head -1)
    fail "import_failure: $MOD — 누락된 모듈을 backend/ 또는 requirements 에서 확인" 30
  fi
  if grep -qE 'sqlite3\.OperationalError|database is locked' "$MIGRATE_LOG"; then
    fail "sqlite_error: 마이그레이션 SQL 실패 — orator/peewee 모델 정의 또는 기존 DB 권한 확인" 30
  fi
  fail "migration_failed (exit=$rc)" 30
fi

log "ALL CHECKS PASSED"
echo "REASON=ok"
exit 0
