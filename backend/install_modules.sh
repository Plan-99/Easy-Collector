#!/usr/bin/env bash
# ===================================================================
# backend/install_modules.sh
# -------------------------------------------------------------------
# Backend Docker 이미지 빌드 시점에 실행. 설치된 모듈들 중
#   - extension 카테고리의 dependencies.apt / dependencies.pip
#   - 모든 카테고리의 dependencies.backend_apt / dependencies.backend_pip
# 를 한 번에 baked install.
#
# 사용:
#   install_modules.sh <STAGING_DIR>
# STAGING_DIR 가 비어있어도 정상 종료 (no-op).
# ===================================================================
set -euo pipefail

STAGING_DIR="${1:-/tmp/modules_staging}"
echo "[backend/install_modules] staging dir: $STAGING_DIR"

if [ ! -d "$STAGING_DIR" ] || [ -z "$(ls -A "$STAGING_DIR" 2>/dev/null | grep -v '^\.gitkeep$' || true)" ]; then
    echo "[backend/install_modules] no manifests staged — skipping."
    exit 0
fi

APT_PKGS=$(python3 -c "
import json, glob, sys
seen = set(); out = []
for p in sorted(glob.glob('$STAGING_DIR/*.json')):
    try:
        m = json.load(open(p))
    except Exception as e:
        print(f'[backend/install_modules][warn] parse fail {p}: {e}', file=sys.stderr)
        continue
    deps = m.get('dependencies') or {}
    cat = m.get('category', '')
    pkgs = []
    if cat == 'extension':
        pkgs += deps.get('apt') or []
    pkgs += deps.get('backend_apt') or []
    for d in pkgs:
        if d and d not in seen:
            seen.add(d); out.append(d)
print(' '.join(out))
")

PIP_PKGS=$(python3 -c "
import json, glob, sys
seen = set(); out = []
for p in sorted(glob.glob('$STAGING_DIR/*.json')):
    try:
        m = json.load(open(p))
    except Exception:
        continue
    deps = m.get('dependencies') or {}
    cat = m.get('category', '')
    pkgs = []
    if cat == 'extension':
        pkgs += deps.get('pip') or []
    pkgs += deps.get('backend_pip') or []
    for d in pkgs:
        if d and d not in seen:
            seen.add(d); out.append(d)
print(' '.join(out))
")

if [ -n "$APT_PKGS" ]; then
    echo "[backend/install_modules] apt install: $APT_PKGS"
    apt-get update
    apt-get install -y --no-install-recommends $APT_PKGS
    rm -rf /var/lib/apt/lists/*
else
    echo "[backend/install_modules] no apt deps to install."
fi

if [ -n "$PIP_PKGS" ]; then
    echo "[backend/install_modules] pip install: $PIP_PKGS"
    # backend 의 base 가 PEP 668 (Ubuntu 24.04) 환경이라 --break-system-packages 필요
    PIP_FLAG="--break-system-packages"
    python3 -m pip install --quiet --help 2>&1 | grep -q -- '--break-system-packages' || PIP_FLAG=""
    python3 -m pip install --no-cache-dir $PIP_FLAG $PIP_PKGS
else
    echo "[backend/install_modules] no pip deps to install."
fi

echo "[backend/install_modules] done."
