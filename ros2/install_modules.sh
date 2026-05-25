#!/usr/bin/env bash
# ===================================================================
# install_modules.sh
# -------------------------------------------------------------------
# Docker 이미지 빌드 시점에 실행되어, staged manifest 들의 apt/pip
# deps 를 한 번에 baked install 한다.
#
# 사용:
#   install_modules.sh <STAGING_DIR>
#
# STAGING_DIR 안엔 각 모듈의 module.json 들이 평탄하게 들어있다고
# 가정한다 (rebuild_ros2_image.sh 가 그렇게 stage 함).
# 비어 있어도 정상 종료 (no-op).
# ===================================================================
set -euo pipefail

STAGING_DIR="${1:-/tmp/modules_staging}"
echo "[install_modules] staging dir: $STAGING_DIR"

if [ ! -d "$STAGING_DIR" ] || [ -z "$(ls -A "$STAGING_DIR" 2>/dev/null)" ]; then
    echo "[install_modules] no manifests staged — skipping."
    exit 0
fi

# aggregate apt + pip deps across all manifests
APT_PKGS=$(python3 -c "
import json, glob, sys
seen = set()
out = []
for p in sorted(glob.glob('$STAGING_DIR/*.json')):
    try:
        meta = json.load(open(p))
    except Exception as e:
        print(f'[install_modules][warn] failed to parse {p}: {e}', file=sys.stderr)
        continue
    deps = (meta.get('dependencies') or {}).get('apt') or []
    for d in deps:
        if d and d not in seen:
            seen.add(d); out.append(d)
print(' '.join(out))
")

PIP_PKGS=$(python3 -c "
import json, glob, sys
seen = set()
out = []
for p in sorted(glob.glob('$STAGING_DIR/*.json')):
    try:
        meta = json.load(open(p))
    except Exception:
        continue
    deps = (meta.get('dependencies') or {}).get('pip') or []
    for d in deps:
        if d and d not in seen:
            seen.add(d); out.append(d)
print(' '.join(out))
")

if [ -n "$APT_PKGS" ]; then
    echo "[install_modules] apt install: $APT_PKGS"
    # 옛 base 이미지(2026-04-22 ~ 2026-05-25) 는 robotpkg.list 가 남아있을 수 있다.
    # robotpkg 미러는 자주 죽어서 apt-get update 를 무한정 stall 시키므로 방어적으로 제거.
    # Pinocchio/CasADi 는 이미 base 에 설치돼 있어 robotpkg 가 필요 없다.
    rm -f /etc/apt/sources.list.d/robotpkg.list /etc/apt/keyrings/robotpkg.asc
    apt-get update
    apt-get install -y --no-install-recommends $APT_PKGS
    rm -rf /var/lib/apt/lists/*  # cache mount 와 무관하게 image 안의 list 는 비워 image 크기 줄임
else
    echo "[install_modules] no apt deps to install."
fi

if [ -n "$PIP_PKGS" ]; then
    echo "[install_modules] pip install: $PIP_PKGS"
    python3 -m pip install --no-cache-dir $PIP_PKGS
else
    echo "[install_modules] no pip deps to install."
fi

echo "[install_modules] done."
