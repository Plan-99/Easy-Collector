#!/usr/bin/env bash
# ===================================================================
# upsert_module_catalog.sh
# -------------------------------------------------------------------
# home-next 웹사이트의 모듈 카탈로그(라이브 Neon DB)에 모듈 1행을 upsert.
# /dashboard/modules 와 /api/modules 는 코드가 아니라 이 DB 의 "Module"
# 테이블에서 읽는다. 모듈 생성 API 가 없고 seed 는 수동이라, 새 모듈은
# 여기서 직접 upsert 해야 사이트에 노출된다. (마켓 릴리즈와는 별개 단계!)
#
# 멱등 단일행 upsert — admin 이 설정한 priceKrw/active 는 보존한다.
#
# 사용:
#   scripts/upsert_module_catalog.sh <category/name> [--prod]
#     (--prod 없으면 SQL 만 출력하는 dry-run)
# ===================================================================
set -euo pipefail

MOD="${1:-}"; PROD=0
for a in "$@"; do [ "$a" = "--prod" ] && PROD=1; done
[ -z "$MOD" ] && { echo "usage: $0 <category/name> [--prod]"; exit 2; }

REPO="$(git -C "$(dirname "$0")/.." rev-parse --show-toplevel)"
MJ="$REPO/modules/$MOD/module.json"
HN="$REPO/home-next"
[ -f "$MJ" ] || { echo "module.json not found: $MJ"; exit 2; }
[ -d "$HN" ] || { echo "home-next dir not found: $HN"; exit 2; }

SQLFILE=$(mktemp /tmp/etmod_catalog.XXXX.sql)
python3 - "$MJ" > "$SQLFILE" <<'PY'
import sys, json
m = json.load(open(sys.argv[1]))
def q(s): return "'" + str(s).replace("'", "''") + "'"
asset = f"module-{m['id']}-{{version}}.tar.gz"
print(f"""INSERT INTO "Module" ("id","name","category","description","priceKrw","active","required","installByDefault","assetName","dependencies","createdAt","updatedAt")
VALUES ({q(m['id'])},{q(m['name'])},{q(m['category'])},{q(m.get('description',''))},0,true,false,false,{q(asset)},ARRAY[]::TEXT[],NOW(),NOW())
ON CONFLICT ("id") DO UPDATE SET
  "name"=EXCLUDED."name","category"=EXCLUDED."category","description"=EXCLUDED."description",
  "required"=EXCLUDED."required","installByDefault"=EXCLUDED."installByDefault",
  "assetName"=EXCLUDED."assetName","dependencies"=EXCLUDED."dependencies","updatedAt"=NOW();""")
PY

echo "----- SQL -----"; cat "$SQLFILE"; echo "---------------"
if [ "$PROD" != 1 ]; then
  echo "[catalog] dry-run (DB 미반영). --prod 로 라이브 Neon 에 적용."
  rm -f "$SQLFILE"; exit 0
fi

# prisma 7 은 prisma.config.ts 의 process.env["POSTGRES_URL"] 로 접속한다.
# dotenv 가 .env 를 로드하는데 .env 의 값은 dev(sqlite)라 무용 → .env.local
# 의 진짜 Neon URL 을 env 로 주입해 override (dotenv 는 기존 env 를 안 덮음).
PURL=$(grep -E '^POSTGRES_URL=' "$HN/.env.local" 2>/dev/null | head -1 | cut -d= -f2- | tr -d '"'"'"'')
[ -z "$PURL" ] && { echo "[catalog] POSTGRES_URL 을 home-next/.env.local 에서 못 찾음"; exit 2; }
echo "[catalog] target host: $(echo "$PURL" | sed -E 's#.*@([^/:]+).*#\1#')"
( cd "$HN" && POSTGRES_URL="$PURL" npx prisma db execute --file "$SQLFILE" )
rm -f "$SQLFILE"

ID=$(python3 -c "import json;print(json.load(open('$MJ'))['id'])")
echo "[catalog] 라이브 API 검증…"
curl -s https://easy-trainer-home.vercel.app/api/modules \
  | python3 -c "import sys,json;d=json.load(sys.stdin);print('LIVE robot present:', any(x['id']=='$ID' for x in d['modules']))"
