#!/usr/bin/env bash
# 새 버전으로 main 에 release 태그를 찍어 deploy.yml CI 를 트리거한다.
#
# 사용법:
#   scripts/release.sh 5.0.1            # 5.0.1 릴리즈
#   scripts/release.sh --dry-run 5.0.1  # push 직전까지만 확인 (실제 push 안 함)
#
# 흐름:
#   1) main 체크아웃 + origin/main fast-forward pull
#   2) 워킹트리 깨끗한지, 새 태그가 origin 에 이미 있는지 검사
#   3) VERSION 파일 갱신 → "version up to X.Y.Z" 커밋
#   4) main push
#   5) X.Y.Z 태그 생성 + push  → GitHub Actions(deploy.yml) 가 빌드/배포 실행
set -euo pipefail

DRY_RUN=0
if [[ "${1:-}" == "--dry-run" ]]; then
    DRY_RUN=1
    shift
fi

VERSION="${1:-}"
if [[ -z "${VERSION}" ]]; then
    echo "usage: $0 [--dry-run] <X.Y.Z>" >&2
    exit 2
fi

# deploy.yml 의 트리거 패턴과 동일 (v prefix 없음).
if ! [[ "${VERSION}" =~ ^[0-9]+\.[0-9]+\.[0-9]+$ ]]; then
    echo "error: version must be X.Y.Z (got: ${VERSION})" >&2
    exit 2
fi

cd "$(git rev-parse --show-toplevel)"

# 1) main 으로 이동 + 최신화
echo "[release] git fetch origin"
git fetch origin --tags --prune

CURRENT_BRANCH=$(git rev-parse --abbrev-ref HEAD)
if [[ "${CURRENT_BRANCH}" != "main" ]]; then
    echo "[release] switching to main (was: ${CURRENT_BRANCH})"
    git checkout main
fi
git pull --ff-only origin main

# 2) 안전성 검사
if ! git diff-index --quiet HEAD --; then
    echo "error: working tree has uncommitted changes" >&2
    git status --short >&2
    exit 1
fi

if git rev-parse -q --verify "refs/tags/${VERSION}" >/dev/null \
   || git ls-remote --exit-code --tags origin "refs/tags/${VERSION}" >/dev/null 2>&1; then
    echo "error: tag ${VERSION} already exists (local or origin)" >&2
    exit 1
fi

CURRENT_VERSION=$(cat VERSION | tr -d '[:space:]')
if [[ "${CURRENT_VERSION}" == "${VERSION}" ]]; then
    echo "error: VERSION is already ${VERSION}" >&2
    exit 1
fi
echo "[release] ${CURRENT_VERSION} → ${VERSION}"

# 3) VERSION 갱신 + 커밋
printf '%s\n' "${VERSION}" > VERSION
git add VERSION
git commit -m "version up to ${VERSION}"

if (( DRY_RUN )); then
    echo "[release] dry-run: 커밋까지만 만들고 push 하지 않음. 되돌리려면:"
    echo "    git reset --hard HEAD~1"
    git log -1 --oneline
    exit 0
fi

# 4) main push
echo "[release] git push origin main"
git push origin main

# 5) 태그 생성 + push → deploy.yml 트리거
echo "[release] git tag ${VERSION} && git push origin ${VERSION}"
git tag "${VERSION}"
git push origin "${VERSION}"

cat <<EOF

[release] 완료. 진행 상황 확인:
  gh run list -L 5
  gh run watch \$(gh run list -L 1 --json databaseId -q '.[0].databaseId')

릴리즈 파이프라인:
  https://github.com/Plan-99/Easy-Collector/actions/workflows/deploy.yml
EOF
