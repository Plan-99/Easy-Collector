#!/usr/bin/env bash
set -e

cd /app

# Install deps if node_modules is empty (first run with named volume)
if [ ! -d "node_modules/.package-lock.json" ] && [ -f "package.json" ]; then
  echo "[frontend] Installing dependencies..."
  npm ci --legacy-peer-deps || npm install --legacy-peer-deps
fi

echo "[frontend] Starting Quasar dev server on port 5173..."
exec npx quasar dev -p 5173 --host 0.0.0.0
