#!/usr/bin/env bash
# ===================================================================
# scripts/overnight_summary.sh
# -------------------------------------------------------------------
# 자율 운영 사이클 1개의 결과를 사람이 한 눈에 볼 수 있게 적어주는 헬퍼.
# 한 야간 운영 = 1 디렉토리. 그 안에 각 사이클이 1 섹션을 append.
#
# 사용:
#   # 야간 시작 (디렉토리 초기화)
#   $(overnight_summary.sh start)              # → 새 RUN_DIR 출력 + summary.md 헤더
#   # 호출자는 환경변수 OVERNIGHT_RUN_DIR 으로 후속 호출 묶음
#
#   # 사이클 1건 기록
#   overnight_summary.sh cycle \
#     --spec docs/queue/01_foo.md \
#     --branch overnight/01-foo \
#     --status pass|fail|skip \
#     --stage smoke|implement|verify|commit|pr  (실패한 단계, status=fail일 때만) \
#     --pr <url> \
#     --note "한 줄 설명" \
#     --diff-stat "<git diff --stat 결과 파일>"
#
#   # 야간 종료 (요약 통계)
#   overnight_summary.sh end                    # 합계 표 append
#
# 출력 경로:
#   ~/easytrainer-overnight/<ts>/summary.md
#   ~/easytrainer-overnight/<ts>/cycles/<NN>_<slug>/   # 사이클별 산출물
#   ~/easytrainer-overnight/latest -> <ts>             # 심볼릭 링크
#
# 어떤 worktree 에서 호출돼도 같은 디렉토리에 기록되도록 절대경로 사용.
# ===================================================================
set -euo pipefail

BASE="${OVERNIGHT_BASE:-$HOME/easytrainer-overnight}"

usage() { sed -n '2,28p' "$0"; }

cmd="${1:-}"
shift || true

case "$cmd" in
    start)
        mkdir -p "$BASE"
        ts="$(date +%Y%m%d-%H%M%S)"
        run_dir="$BASE/$ts"
        mkdir -p "$run_dir/cycles"
        {
            echo "# 야간 운영 — $ts"
            echo
            echo "- **시작**: $(date '+%Y-%m-%d %H:%M:%S %Z')"
            echo "- **호스트**: $(hostname)"
            echo "- **repo**: $(git -C "$(pwd)" rev-parse --show-toplevel 2>/dev/null || pwd)"
            echo "- **base branch**: $(git rev-parse --abbrev-ref HEAD 2>/dev/null || '?')"
            echo "- **base commit**: $(git rev-parse --short HEAD 2>/dev/null || '?')"
            echo
            echo "## 사이클"
            echo
            echo "| # | spec | 상태 | branch | PR | 소요 | note |"
            echo "|---|------|------|--------|-----|------|------|"
        } > "$run_dir/summary.md"
        # latest 심볼릭 링크 갱신
        ln -sfn "$ts" "$BASE/latest"
        # 사이클 카운터 파일
        echo 0 > "$run_dir/.cycle_count"
        echo "$run_dir"
        ;;

    cycle)
        run_dir="${OVERNIGHT_RUN_DIR:-$BASE/latest}"
        if [ ! -d "$run_dir" ]; then
            echo "[overnight] RUN_DIR 없음: $run_dir — 먼저 'start' 호출 필요" >&2
            exit 2
        fi
        # 인자 파싱
        spec="" branch="" status="" stage="" pr="" note="" diff_stat="" started="$(date +%s)" elapsed=""
        while [ $# -gt 0 ]; do
            case "$1" in
                --spec)       spec="$2"; shift 2 ;;
                --branch)     branch="$2"; shift 2 ;;
                --status)     status="$2"; shift 2 ;;
                --stage)      stage="$2"; shift 2 ;;
                --pr)         pr="$2"; shift 2 ;;
                --note)       note="$2"; shift 2 ;;
                --diff-stat)  diff_stat="$2"; shift 2 ;;
                --started)    started="$2"; shift 2 ;;
                --elapsed)    elapsed="$2"; shift 2 ;;
                *) shift ;;
            esac
        done
        [ -z "$status" ] && { echo "[overnight] --status 필수 (pass|fail|skip)" >&2; exit 64; }

        # 사이클 번호 증가
        n=$(cat "$run_dir/.cycle_count" 2>/dev/null || echo 0)
        n=$((n+1))
        echo "$n" > "$run_dir/.cycle_count"

        nn=$(printf "%02d" "$n")
        slug="$(basename "$spec" .md 2>/dev/null | tr -c 'A-Za-z0-9_-' '_' || echo unknown)"
        cycle_dir="$run_dir/cycles/${nn}_${slug}"
        mkdir -p "$cycle_dir"

        # 상태별 이모지
        case "$status" in
            pass)  emoji="✅" ;;
            fail)  emoji="❌${stage:+ ($stage)}" ;;
            skip)  emoji="⏭" ;;
            *)     emoji="?" ;;
        esac

        # 소요 계산
        if [ -z "$elapsed" ]; then
            elapsed=$(( $(date +%s) - started ))
        fi
        elapsed_h=$(printf '%dm%ds' $((elapsed/60)) $((elapsed%60)))

        # markdown table 행 (note 안의 | 를 escape)
        note_safe="$(echo "$note" | sed 's/|/\\|/g')"
        pr_md=""
        [ -n "$pr" ] && pr_md="[link]($pr)"
        echo "| $nn | \`$(basename "$spec")\` | $emoji | \`$branch\` | $pr_md | $elapsed_h | $note_safe |" \
            >> "$run_dir/summary.md"

        # diff stat 보존
        if [ -n "$diff_stat" ] && [ -f "$diff_stat" ]; then
            cp "$diff_stat" "$cycle_dir/diff-stat.txt"
        fi

        echo "$cycle_dir"
        ;;

    end)
        run_dir="${OVERNIGHT_RUN_DIR:-$BASE/latest}"
        if [ ! -d "$run_dir" ]; then
            echo "[overnight] RUN_DIR 없음" >&2
            exit 2
        fi
        # 합계
        n=$(cat "$run_dir/.cycle_count" 2>/dev/null || echo 0)
        # markdown 표에서 status 컬럼 카운트
        pass=$(grep -c '| ✅' "$run_dir/summary.md" || true)
        fail=$(grep -c '| ❌' "$run_dir/summary.md" || true)
        skip=$(grep -c '| ⏭'  "$run_dir/summary.md" || true)
        {
            echo
            echo "## 결과"
            echo
            echo "- **종료**: $(date '+%Y-%m-%d %H:%M:%S %Z')"
            echo "- **총 사이클**: $n  (PASS $pass / FAIL $fail / SKIP $skip)"
            if [ "$fail" -gt 0 ]; then
                echo "- **실패한 spec은 \`docs/queue/_failed/\`로 이동됨** — 일어나서 \`summary.md\`와 함께 검토"
            fi
            echo
            echo "## 다음 단계"
            echo
            echo "1. \`gh pr list --author @me\` 로 새로 만들어진 PR 확인 → 검토 후 머지"
            echo "2. \`docs/queue/_failed/\` 가 비어있지 않으면 각 spec의 사이클 디렉토리에서 stage/diff 검토"
            echo "3. 머지 후 \`git checkout main && git pull\` 로 base 정리"
        } >> "$run_dir/summary.md"
        echo "$run_dir/summary.md"
        ;;

    path)
        echo "${OVERNIGHT_RUN_DIR:-$BASE/latest}"
        ;;

    -h|--help|"") usage ;;
    *) echo "[overnight] unknown command: $cmd" >&2; usage; exit 64 ;;
esac
