#!/usr/bin/env bash
# ===================================================================
# rebuild_images.sh
# -------------------------------------------------------------------
# 설치된 모듈들의 deps 를 base 이미지 위에 baked 한 derived 이미지를
# 빌드한다. 모듈 설치/제거 후, 또는 첫 setup 시 호출.
#
#   $ bash scripts/rebuild_images.sh [ros2|backend|all]   (default: all)
#
# 동작:
#   1. project/modules/*.json 을 읽어 ros2/backend 별로 stage
#   2. base 이미지가 없으면 먼저 빌드 (easytrainer-ros2-base / easytrainer-backend-base)
#   3. derived 이미지 빌드 (easytrainer-ros2:latest / easytrainer-backend:latest)
#      - BuildKit cache mount 로 apt/pip 영구 캐시
#      - manifest 변경 없으면 COPY layer 캐시 hit → 거의 즉시
#
# 환경:
#   EASYTRAINER_DATA_DIR (default: /opt/easytrainer)
#   DOCKER_BUILDKIT 자동 1로 설정됨
# ===================================================================
set -euo pipefail

TARGET="${1:-all}"
PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
DATA_DIR="${EASYTRAINER_DATA_DIR:-/opt/easytrainer}"
MANIFEST_DIR="$DATA_DIR/project/modules"

export DOCKER_BUILDKIT=1
cd "$PROJECT_ROOT"

if [ ! -d "$MANIFEST_DIR" ]; then
    echo "[rebuild_images] WARN: manifest dir not found: $MANIFEST_DIR — modules layer will be empty."
    MANIFEST_DIR=""
fi

# --- 공통: 한 컨테이너용으로 manifest 들을 stage ---
# $1 = staging dir
# $2 = python filter expr (true 면 include) — 각 manifest dict 에 대해 평가
stage_manifests() {
    local staging="$1"
    local filter="$2"
    rm -rf "$staging"
    mkdir -p "$staging"
    [ -z "$MANIFEST_DIR" ] && return 0
    # placeholder so git/docker is happy even with no modules
    touch "$staging/.gitkeep"
    local count=0
    for mj in "$MANIFEST_DIR"/*.json; do
        [ -f "$mj" ] || continue
        local keep
        keep=$(python3 -c "
import json, sys
try:
    m = json.load(open('$mj'))
except Exception:
    sys.exit(1)
ok = ($filter)
print('1' if ok else '0')
" 2>/dev/null || echo "0")
        if [ "$keep" = "1" ]; then
            cp "$mj" "$staging/$(basename "$mj")"
            count=$((count+1))
        fi
    done
    echo "[rebuild_images] staged $count manifest(s) into $staging"
}

build_ros2() {
    echo ""
    echo "=== Building ros2 images ==="
    # base
    if ! docker image inspect easytrainer-ros2-base:latest >/dev/null 2>&1 \
       || [ "${FORCE_BASE:-0}" = "1" ]; then
        echo "[rebuild_images] building base: easytrainer-ros2-base:latest"
        docker build \
            -f "$PROJECT_ROOT/ros2/Dockerfile.base" \
            -t easytrainer-ros2-base:latest \
            "$PROJECT_ROOT"
    else
        echo "[rebuild_images] base image already exists (set FORCE_BASE=1 to rebuild)"
    fi

    # stage manifests: robot/sensor 카테고리의 deps.apt / deps.pip 가 ros2 컨테이너 대상
    stage_manifests \
        "$PROJECT_ROOT/ros2/_modules_staging" \
        "m.get('category') in ('robot','sensor') and ((m.get('dependencies') or {}).get('apt') or (m.get('dependencies') or {}).get('pip'))"

    echo "[rebuild_images] building modules layer: easytrainer-ros2:latest"
    docker build \
        -f "$PROJECT_ROOT/ros2/Dockerfile" \
        -t easytrainer-ros2:latest \
        --build-arg BASE_IMAGE=easytrainer-ros2-base:latest \
        "$PROJECT_ROOT"
}

build_backend() {
    echo ""
    echo "=== Building backend images ==="
    # base
    if ! docker image inspect easytrainer-backend-base:latest >/dev/null 2>&1 \
       || [ "${FORCE_BASE:-0}" = "1" ]; then
        echo "[rebuild_images] building base: easytrainer-backend-base:latest"
        docker build \
            -f "$PROJECT_ROOT/backend/Dockerfile.base" \
            -t easytrainer-backend-base:latest \
            "$PROJECT_ROOT"
    else
        echo "[rebuild_images] base image already exists (set FORCE_BASE=1 to rebuild)"
    fi

    # backend 컨테이너에 들어가는 deps:
    #   - extension 카테고리의 dependencies.apt / dependencies.pip
    #   - 모든 카테고리의 dependencies.backend_apt / dependencies.backend_pip
    stage_manifests \
        "$PROJECT_ROOT/backend/_modules_staging" \
        "(m.get('category')=='extension' and ((m.get('dependencies') or {}).get('apt') or (m.get('dependencies') or {}).get('pip'))) or ((m.get('dependencies') or {}).get('backend_apt') or (m.get('dependencies') or {}).get('backend_pip'))"

    echo "[rebuild_images] building modules layer: easytrainer-backend:latest"
    docker build \
        -f "$PROJECT_ROOT/backend/Dockerfile" \
        -t easytrainer-backend:latest \
        --build-arg BASE_IMAGE=easytrainer-backend-base:latest \
        "$PROJECT_ROOT"
}

case "$TARGET" in
    ros2)    build_ros2 ;;
    backend) build_backend ;;
    all)     build_ros2; build_backend ;;
    *)
        echo "Usage: $0 [ros2|backend|all]" >&2
        exit 1
        ;;
esac

echo ""
echo "[rebuild_images] done."
