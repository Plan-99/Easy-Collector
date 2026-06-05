#!/usr/bin/env bash
# ===================================================================
# scripts/db_snapshot.sh
# -------------------------------------------------------------------
# main.db (SQLite) 의 안전한 스냅샷/복원. 자율 운영 루프가 E2E 전후로
# 사용해 한 사이클의 DB 오염이 다음 사이클에 누적되지 않게 한다.
#
# 사용:
#   db_snapshot.sh save <label>      # 현재 main.db 를 <label> 로 저장
#   db_snapshot.sh restore <label>   # main.db 를 <label> 로 덮어쓰기
#   db_snapshot.sh list              # 보유 스냅샷 목록
#   db_snapshot.sh path <label>      # 스냅샷 절대경로 출력
#   db_snapshot.sh clean --older-than <days>
#
# 환경:
#   EASYTRAINER_DATA_DIR (default: /opt/easytrainer)
#   DB_FILE              (default: $EASYTRAINER_DATA_DIR/database/main.db)
#   SNAPSHOT_DIR         (default: $EASYTRAINER_DATA_DIR/db_snapshots)
#
# 내부적으로 sqlite3 .backup 을 우선 시도 (live-safe). 없으면 cp 폴백.
# 복원 시 backend가 떠있으면 종료 코드 3으로 거부 — 사용자가 정지 후 재시도.
# ===================================================================
set -euo pipefail

DATA_DIR="${EASYTRAINER_DATA_DIR:-/opt/easytrainer}"
DB_FILE="${DB_FILE:-$DATA_DIR/database/main.db}"
SNAPSHOT_DIR="${SNAPSHOT_DIR:-$DATA_DIR/db_snapshots}"
HAVE_SQLITE3=0
command -v sqlite3 >/dev/null 2>&1 && HAVE_SQLITE3=1

usage() { sed -n '2,18p' "$0"; }

snap_path() { echo "$SNAPSHOT_DIR/$1.db"; }

ensure_dirs() { mkdir -p "$SNAPSHOT_DIR"; }

backend_running() {
    command -v docker >/dev/null 2>&1 || return 1
    docker ps --format '{{.Names}}' 2>/dev/null | grep -qx easytrainer_backend
}

do_save() {
    local label="$1"
    [ -z "$label" ] && { echo "[db_snapshot] label 필수" >&2; exit 64; }
    if [ ! -f "$DB_FILE" ]; then
        echo "[db_snapshot] DB 파일 없음: $DB_FILE" >&2
        exit 4
    fi
    ensure_dirs
    local dst
    dst="$(snap_path "$label")"
    local tmp="${dst}.partial.$$"
    if [ "$HAVE_SQLITE3" = "1" ]; then
        # online backup (WAL-aware)
        sqlite3 "$DB_FILE" ".backup '$tmp'"
    else
        cp "$DB_FILE" "$tmp"
    fi
    mv "$tmp" "$dst"
    local size
    size=$(stat -c%s "$dst" 2>/dev/null || wc -c <"$dst")
    echo "[db_snapshot] saved: $dst ($size bytes)"
}

do_restore() {
    local label="$1"
    [ -z "$label" ] && { echo "[db_snapshot] label 필수" >&2; exit 64; }
    local src
    src="$(snap_path "$label")"
    if [ ! -f "$src" ]; then
        echo "[db_snapshot] 스냅샷 없음: $src" >&2
        exit 4
    fi
    if backend_running; then
        echo "[db_snapshot] 거부: backend 컨테이너가 동작 중. 먼저 'docker compose stop backend' 또는 backend API 중단 후 재시도." >&2
        exit 3
    fi
    # 안전: 현재 DB를 .pre-restore-<ts> 로 백업
    if [ -f "$DB_FILE" ]; then
        local ts
        ts=$(date +%Y%m%d-%H%M%S)
        cp "$DB_FILE" "$DB_FILE.pre-restore-$ts"
        echo "[db_snapshot] 현재 DB 보존: $DB_FILE.pre-restore-$ts"
    fi
    mkdir -p "$(dirname "$DB_FILE")"
    cp "$src" "$DB_FILE"
    # WAL/SHM 잔여 정리
    rm -f "$DB_FILE-wal" "$DB_FILE-shm"
    echo "[db_snapshot] restored from: $src"
}

do_list() {
    ensure_dirs
    local count=0
    for f in "$SNAPSHOT_DIR"/*.db; do
        [ -f "$f" ] || continue
        local name size mtime
        name=$(basename "$f" .db)
        size=$(stat -c%s "$f" 2>/dev/null || wc -c <"$f")
        mtime=$(date -r "$f" '+%Y-%m-%d %H:%M:%S' 2>/dev/null || echo "?")
        printf "%-30s %10s bytes  %s\n" "$name" "$size" "$mtime"
        count=$((count+1))
    done
    if [ "$count" = "0" ]; then
        echo "[db_snapshot] 스냅샷 없음 ($SNAPSHOT_DIR)"
    fi
}

do_path() {
    local label="$1"
    snap_path "$label"
}

do_clean() {
    local days="${1:-7}"
    case "$days" in
        ''|*[!0-9]*) echo "[db_snapshot] --older-than 값은 정수(일)이어야 함" >&2; exit 64 ;;
    esac
    ensure_dirs
    local removed=0
    while IFS= read -r -d '' f; do
        rm -f "$f"
        echo "[db_snapshot] removed: $f"
        removed=$((removed+1))
    done < <(find "$SNAPSHOT_DIR" -maxdepth 1 -type f -name '*.db' -mtime +"$days" -print0)
    echo "[db_snapshot] cleaned $removed snapshot(s) older than $days day(s)"
}

cmd="${1:-}"
shift || true
case "$cmd" in
    save)    do_save "${1:-}" ;;
    restore) do_restore "${1:-}" ;;
    list)    do_list ;;
    path)    do_path "${1:-}" ;;
    clean)
        # 인자: --older-than <days>
        days=7
        while [ $# -gt 0 ]; do
            case "$1" in
                --older-than) days="$2"; shift 2 ;;
                *) shift ;;
            esac
        done
        do_clean "$days"
        ;;
    -h|--help|help|"") usage ;;
    *) echo "[db_snapshot] unknown command: $cmd" >&2; usage; exit 64 ;;
esac
