#!/usr/bin/env bash
# Run frontend ESLint on staged files.
#
# Tries host first (cheap), falls back to docker exec into the running
# easy_collector_service container, then skips with a warning if neither
# environment is available.
#
# Arguments: absolute or repo-relative paths under frontend/ (passed by pre-commit).
set -euo pipefail

REPO_ROOT="$(git rev-parse --show-toplevel)"
CONTAINER_NAME="easy_collector_service"
CONTAINER_PROJECT="/opt/easytrainer/project"

if [ "$#" -eq 0 ]; then
  exit 0
fi

# Convert any absolute path back to a repo-relative path so it makes sense
# inside the container's project mount.
rel_paths=()
for p in "$@"; do
  case "$p" in
    /*) rel_paths+=("${p#"$REPO_ROOT/"}") ;;
    *)  rel_paths+=("$p") ;;
  esac
done

# Strip leading "frontend/" because eslint is invoked from inside the frontend dir.
inside_paths=()
for p in "${rel_paths[@]}"; do
  inside_paths+=("${p#frontend/}")
done

if [ -x "$REPO_ROOT/frontend/node_modules/.bin/eslint" ]; then
  cd "$REPO_ROOT/frontend"
  exec ./node_modules/.bin/eslint -c ./eslint.config.js "${inside_paths[@]}"
fi

if command -v docker >/dev/null 2>&1 && docker ps --format '{{.Names}}' | grep -qx "$CONTAINER_NAME"; then
  exec docker exec "$CONTAINER_NAME" bash -lc \
    "cd $CONTAINER_PROJECT/frontend && npx --no-install eslint -c ./eslint.config.js $(printf '%q ' "${inside_paths[@]}")"
fi

echo "[lint_frontend] no host node_modules and no running '$CONTAINER_NAME' container — skipping." >&2
echo "[lint_frontend] To enable: 'cd frontend && npm install' on host, or 'docker compose up -d service'." >&2
exit 0
