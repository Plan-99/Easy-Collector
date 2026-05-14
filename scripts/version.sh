#!/usr/bin/env bash
# 현재 버전 정보를 출력.
#
# 사용법:
#   scripts/version.sh           # 사람이 읽기 좋은 형식 (VERSION 파일 + 최신 태그 + 미커밋 여부)
#   scripts/version.sh --short   # VERSION 파일 값만 (스크립트에서 파이프할 때)
set -euo pipefail

cd "$(git rev-parse --show-toplevel)"

VERSION_FILE=$(cat VERSION | tr -d '[:space:]')

if [[ "${1:-}" == "--short" ]]; then
    printf '%s\n' "${VERSION_FILE}"
    exit 0
fi

# 패턴 X.Y.Z 만 보고 최신 태그 (없으면 "(none)")
LATEST_TAG=$(git tag --list '[0-9]*.[0-9]*.[0-9]*' --sort=-v:refname | head -n1)
LATEST_TAG=${LATEST_TAG:-"(none)"}

CURRENT_BRANCH=$(git rev-parse --abbrev-ref HEAD)
COMMIT=$(git rev-parse --short HEAD)

DIRTY=""
if ! git diff-index --quiet HEAD --; then
    DIRTY=" (dirty: uncommitted changes)"
fi

# VERSION 파일 값이 태그로도 떴는지
TAG_STATE="not yet tagged"
if git rev-parse -q --verify "refs/tags/${VERSION_FILE}" >/dev/null; then
    TAG_STATE="tagged locally"
    if git ls-remote --exit-code --tags origin "refs/tags/${VERSION_FILE}" >/dev/null 2>&1; then
        TAG_STATE="released (tag pushed to origin)"
    fi
fi

cat <<EOF
VERSION file : ${VERSION_FILE}  [${TAG_STATE}]
latest tag   : ${LATEST_TAG}
branch       : ${CURRENT_BRANCH} @ ${COMMIT}${DIRTY}
EOF
