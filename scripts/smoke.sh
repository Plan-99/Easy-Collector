#!/usr/bin/env bash
# ===================================================================
# scripts/smoke.sh
# -------------------------------------------------------------------
# 30초 안에 끝나는 빠른 검증 게이트. 자율 운영 루프의 첫 번째 거름망.
#
# 검사:
#   1. 핵심 컨테이너 살아있는지 (docker ps)
#   2. backend  /api/healthz                  (200)
#   3. backend  /api/sensors|robots|datasets|... GET (200)
#   4. training_server  /health   (포트 5100, 옵션)
#   5. ros2 컨테이너 안에서 ros2 daemon 응답   (옵션, 최선 노력)
#
# 사용:
#   bash scripts/smoke.sh             # full smoke
#   bash scripts/smoke.sh --quick     # health only (~3s)
#   bash scripts/smoke.sh --json      # machine-readable
#
# 종료 코드:
#   0  = 모든 검사 통과
#   1  = 한 개 이상 실패
#   2  = 핵심 인프라(컨테이너) 없음 — 검증 진행 불가
# ===================================================================
set -uo pipefail

QUICK=0
JSON=0
for arg in "$@"; do
    case "$arg" in
        --quick) QUICK=1 ;;
        --json)  JSON=1 ;;
        -h|--help)
            sed -n '2,18p' "$0"; exit 0 ;;
        *) echo "[smoke] unknown arg: $arg" >&2; exit 64 ;;
    esac
done

BACKEND_HOST="${SMOKE_BACKEND_HOST:-localhost}"
BACKEND_PORT="${SMOKE_BACKEND_PORT:-5000}"
TRAIN_PORT="${SMOKE_TRAIN_PORT:-5100}"
TIMEOUT="${SMOKE_TIMEOUT:-3}"

START_TS=$(date +%s)
RESULTS=()  # "name|status|detail"

record() {
    # name | status (PASS|FAIL|SKIP) | detail
    RESULTS+=("$1|$2|$3")
}

http_get() {
    # http_get <url> -> echoes "<code>|<body-head>"
    local url="$1"
    local code body
    body=$(curl -sS -m "$TIMEOUT" -o /tmp/.smoke_body.$$ -w "%{http_code}" "$url" 2>/tmp/.smoke_err.$$ || echo "000")
    code="$body"
    local head=""
    if [ -s /tmp/.smoke_body.$$ ]; then
        head=$(head -c 200 /tmp/.smoke_body.$$ | tr '\n' ' ')
    fi
    rm -f /tmp/.smoke_body.$$ /tmp/.smoke_err.$$
    echo "$code|$head"
}

# --- 1. 컨테이너 ---
if command -v docker >/dev/null 2>&1; then
    running=$(docker ps --format '{{.Names}}' 2>/dev/null || true)
    for c in easytrainer_backend easytrainer_ros2; do
        if echo "$running" | grep -qx "$c"; then
            record "container:$c" PASS "running"
        else
            record "container:$c" FAIL "not running"
        fi
    done
    # frontend은 dev에서 직접 npm run dev로 돌리는 경우가 있어 SKIP 처리
    if echo "$running" | grep -qx "easytrainer_frontend"; then
        record "container:easytrainer_frontend" PASS "running"
    else
        record "container:easytrainer_frontend" SKIP "not running (ok if dev'd on host)"
    fi
else
    record "container:docker" SKIP "docker CLI 없음"
fi

# --- 2. backend health ---
res=$(http_get "http://${BACKEND_HOST}:${BACKEND_PORT}/api/healthz")
code=${res%%|*}
if [ "$code" = "200" ]; then
    record "backend:/api/healthz" PASS "200"
else
    record "backend:/api/healthz" FAIL "code=$code"
fi

if [ "$QUICK" = "0" ]; then
    # --- 3. 핵심 GET 엔드포인트 ---
    for ep in \
        /api/sensors \
        /api/robots \
        /api/assemblies \
        /api/policies \
        /api/tasks \
        /api/checkpoints \
        /api/datasets \
        /api/teleoperators \
        /api/planners \
        /api/modules/installed \
        /api/tutorial/status \
        /api/demo/list
    do
        res=$(http_get "http://${BACKEND_HOST}:${BACKEND_PORT}${ep}")
        code=${res%%|*}
        if [ "$code" = "200" ]; then
            record "GET $ep" PASS "200"
        else
            head=${res#*|}
            record "GET $ep" FAIL "code=$code body=${head:0:80}"
        fi
    done

    # --- 4. training_server ---
    if (echo > /dev/tcp/localhost/${TRAIN_PORT}) 2>/dev/null; then
        res=$(http_get "http://localhost:${TRAIN_PORT}/health")
        code=${res%%|*}
        if [ "$code" = "200" ]; then
            record "training_server:/health" PASS "200"
        elif [ "$code" = "404" ]; then
            # 포트는 살아있고 /health가 없을 수도 있음 — TCP 살아있으면 PASS
            record "training_server:tcp" PASS "tcp open, /health 404 (ok)"
        else
            record "training_server:/health" FAIL "code=$code"
        fi
    else
        record "training_server:tcp" SKIP "port ${TRAIN_PORT} closed"
    fi

    # --- 5. ros2 daemon (best-effort, 컨테이너 안) ---
    if echo "$running" | grep -qx "easytrainer_ros2"; then
        if timeout "$TIMEOUT" docker exec easytrainer_ros2 \
             bash -lc 'source /opt/ros/humble/setup.bash && ros2 topic list >/dev/null 2>&1'; then
            record "ros2:topic-list" PASS "ok"
        else
            record "ros2:topic-list" FAIL "ros2 topic list failed"
        fi
    fi
fi

# --- 출력 ---
ELAPSED=$(( $(date +%s) - START_TS ))
PASS=0; FAIL=0; SKIP=0
for r in "${RESULTS[@]}"; do
    s=$(echo "$r" | cut -d'|' -f2)
    case "$s" in PASS) PASS=$((PASS+1));; FAIL) FAIL=$((FAIL+1));; SKIP) SKIP=$((SKIP+1));; esac
done

if [ "$JSON" = "1" ]; then
    printf '{"elapsed_s":%d,"pass":%d,"fail":%d,"skip":%d,"checks":[' "$ELAPSED" "$PASS" "$FAIL" "$SKIP"
    first=1
    for r in "${RESULTS[@]}"; do
        n=$(echo "$r" | cut -d'|' -f1)
        s=$(echo "$r" | cut -d'|' -f2)
        d=$(echo "$r" | cut -d'|' -f3- | sed 's/"/\\"/g')
        [ $first -eq 0 ] && printf ','
        first=0
        printf '{"name":"%s","status":"%s","detail":"%s"}' "$n" "$s" "$d"
    done
    printf ']}\n'
else
    for r in "${RESULTS[@]}"; do
        n=$(echo "$r" | cut -d'|' -f1)
        s=$(echo "$r" | cut -d'|' -f2)
        d=$(echo "$r" | cut -d'|' -f3-)
        case "$s" in
            PASS) printf "[smoke] \033[32mPASS\033[0m %-40s %s\n" "$n" "$d" ;;
            FAIL) printf "[smoke] \033[31mFAIL\033[0m %-40s %s\n" "$n" "$d" ;;
            SKIP) printf "[smoke] \033[33mSKIP\033[0m %-40s %s\n" "$n" "$d" ;;
        esac
    done
    echo
    printf "[smoke] %d pass / %d fail / %d skip in %ds\n" "$PASS" "$FAIL" "$SKIP" "$ELAPSED"
fi

# 핵심 인프라 missing이면 exit 2
backend_running=$(printf '%s\n' "${RESULTS[@]}" | grep -E '^container:easytrainer_backend\|FAIL' || true)
if [ -n "$backend_running" ]; then
    exit 2
fi

[ "$FAIL" -eq 0 ] && exit 0 || exit 1
