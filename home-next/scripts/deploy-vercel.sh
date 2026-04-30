#!/usr/bin/env bash
# Deploy home-next to Vercel production from the local working tree.
# Run from anywhere — the script cd's into the home-next root itself.
#
# Usage:
#   bash scripts/deploy-vercel.sh             # production deploy (--prod)
#   bash scripts/deploy-vercel.sh preview     # preview deploy
#
# What it does:
#   1) Sanity-check Node + Vercel CLI availability (uses npx vercel, no global install needed).
#   2) Verify the project is linked (.vercel/project.json present).
#   3) Run `next build` locally first so you don't burn a Vercel build slot on type errors.
#   4) `vercel deploy [--prod] --yes` from this directory.
#   5) Print the deployment URL + run a quick HTTP smoke check on key paths.
#
# Skips:
#   - git commit/push (intentional — deploys current working tree)
#   - prisma migrate (run manually with `npx prisma migrate deploy` when schema changes)
set -euo pipefail

MODE="${1:-prod}"

# Locate home-next root (parent of this scripts dir)
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
APP_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
cd "$APP_DIR"

echo "▶ home-next root: $APP_DIR"

if [ ! -f ".vercel/project.json" ]; then
  echo "✖ .vercel/project.json not found — run \`npx vercel link\` once first." >&2
  exit 1
fi

PROJECT_NAME="$(node -e 'console.log(require("./.vercel/project.json").projectName)')"
echo "▶ Vercel project: $PROJECT_NAME"

# Local build first (catches type errors cheaply)
echo "▶ Local build…"
npx --yes next build > /tmp/home-next-build.log 2>&1 || {
  echo "✖ Local build failed. Last 40 lines:" >&2
  tail -40 /tmp/home-next-build.log >&2
  exit 1
}
echo "  ✓ build ok"

# Deploy
DEPLOY_FLAGS=("--yes")
if [ "$MODE" = "prod" ] || [ "$MODE" = "production" ]; then
  DEPLOY_FLAGS+=("--prod")
  echo "▶ Deploying to PRODUCTION…"
else
  echo "▶ Deploying preview…"
fi

DEPLOY_URL="$(npx --yes vercel deploy "${DEPLOY_FLAGS[@]}" 2>&1 | tee /tmp/home-next-deploy.log | tail -n 1 | tr -d '[:space:]')"

if [[ ! "$DEPLOY_URL" =~ ^https:// ]]; then
  echo "✖ Could not parse deploy URL. Full log:" >&2
  cat /tmp/home-next-deploy.log >&2
  exit 1
fi

echo "  ✓ deployed: $DEPLOY_URL"

# Smoke check (production alias only when --prod)
SMOKE_TARGETS=("$DEPLOY_URL")
if [ "$MODE" = "prod" ] || [ "$MODE" = "production" ]; then
  SMOKE_TARGETS+=("https://easytrainerhome.vercel.app")
fi

echo "▶ Smoke check"
for base in "${SMOKE_TARGETS[@]}"; do
  for path in "/" "/auth/signin" "/api/modules"; do
    code="$(curl -s -o /dev/null -w '%{http_code}' -L --max-time 10 "$base$path" || echo "ERR")"
    printf "  %s %s%s\n" "$code" "$base" "$path"
  done
done

echo "✓ Done."
