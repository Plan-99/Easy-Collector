# HARNESS.md — EasyTrainer 자율 운영 하네스 지도

> 이 프로젝트의 **하네스**(자율 개발/평가를 굴리는 비계 일체)의 단일 목차다.
> "어떤 하네스 요소가 존재하는가"를 한눈에 본다. 요소를 더하거나 뺄 때는 이 파일과
> [learning.md](../learning.md) 를 함께 갱신한다.
>
> 설계 철학과 주조/진화 방법론은 [foundry-harness/](../foundry-harness/) (메타-하네스) 와
> 그 헌법 [foundry-harness/references/principles.md](../foundry-harness/references/principles.md) 를 따른다.
> 핵심 원칙: **모든 하네스 요소는 세금이다 — 관찰된 실패가 없으면 넣지 않는다.**

## 슬래시 명령 (`.claude/commands/`)

| 명령 | 역할 | 비고 |
|------|------|------|
| `/smoke` | 30초 검증 게이트 (컨테이너 + healthz + 핵심 GET) | 모든 루프의 첫 게이트. `scripts/smoke.sh` |
| `/quick-apply` | 소스 → 런타임(`/opt/easytrainer/project`) 동기화 | 코드 수정 후 필수. `scripts/quick_apply.sh` |
| `/manual-update` | 사용자 매뉴얼 동기화 (manual-writer 위임) | |
| `/feature-from-spec` | spec 1개 → worktree → 구현 → 검증 → PR (자율 1사이클) | `/overnight-queue` 가 호출 |
| `/overnight-queue` | `docs/queue/` 의 spec 들을 순서대로 처리 (야간 자율 진입점) | 6시간 무인 운영 |
| `/model-test` | 튜토리얼 peg-in-hole 정책 e2e 평가 (수집→학습→평가) | 단일 정책 평가 (랩과 공유하는 model_tester 도구) |
| `/nightwatch` | 시나리오 기반 자율 UI 검증·수선 루프 (device-free 표면) | `nightwatch/` 자산, 인터랙티브 우선 |

> **정책 연구/토너먼트/진화는 여기 없다 — `policy-lab/` (별도 하네스)로 이관.** `/policy-design`·
> `/policy-tournament`·`/policy-evolve`·`/lab-overnight`·`/lab-smoke` 와 `policy-researcher` 에이전트는
> [../policy-lab/](../policy-lab/) 가 소유한다. `cd policy-lab && claude` 로 운영. 아래 "별도 하네스" 절 참고.

## 서브에이전트 (`.claude/agents/`) — [README](../.claude/agents/README.md)

| 에이전트 | 역할 |
|----------|------|
| `manual-writer` | 사용자 매뉴얼(`home-next/.../ko/**`) 전담. 제품 소스는 read-only. |

(policy-researcher 는 `policy-lab/.claude/agents/` 로 이관됨.)

## PostToolUse 훅 (`.claude/hooks/`)

모두 Write/Edit/MultiEdit 후 실행 (`.claude/settings.json` 에 배선). **프로젝트 루트는
`$CLAUDE_PROJECT_DIR`(없으면 스크립트 위치 기준)로 동적 도출 — 절대 하드코딩 금지** (learning.md F1).

| 훅 | 트리거 | 동작 |
|----|--------|------|
| `manual-update-reminder.sh` | 매뉴얼 트리거 표면 변경 | 세션당 1회 매뉴얼 동기화 리마인더 |
| `frontend-i18n-check.sh` | `frontend/src/**` (로케일 제외) | ko-KR/en-US 양쪽에 누락된 i18n 키 경고 (매 편집) |
| `frontend-no-css.sh` | `frontend/src/**` | Vue SFC `<style>` 블록·생 CSS 파일 경고 |
| `frontend-ui-test-reminder.sh` | `frontend/src/{pages,components}/**`, `release/ui/**` | 세션당 1회 Playwright UI 테스트 리마인더 |

## 스킬 (`.claude/skills/`)

- `playwright-skill/` — 브라우저 자동화/UI 검증.

## 스크립트 (`scripts/`)

| 스크립트 | 용도 |
|----------|------|
| `smoke.sh` | 빠른 검증 게이트 (env-var 기반, 호스트 무관) |
| `quick_apply.sh` | 소스 → 런타임 rsync |
| `db_snapshot.sh` | DB 스냅샷 save/clean (`EASYTRAINER_DATA_DIR` 기반) |
| `overnight_summary.sh` | 야간 루프 run-dir / summary.md 관리 (`$HOME/easytrainer-overnight/`) |
| `manual-check.sh` | 매뉴얼 staleness hard 게이트 (기능 표면이 매뉴얼보다 새로우면 exit 1) |

## 작업 큐 / 진화 산출물

- `docs/queue/` — `/overnight-queue` 가 처리할 spec 큐. 상태는 디렉토리 위치로 표현
  (`_in-progress/`, `_done/`, `_failed/`, `_drafts/`). [README](queue/README.md), [_TEMPLATE](queue/_TEMPLATE.md).
- `nightwatch/` — `/nightwatch` 자산: `scenarios/*.md`(시험지), `baselines/`(시각 기준, `_pending/`=미승인),
  `findings/round_<N>.md`(FAIL 기록). fix 단계는 이 폴더를 절대 편집 안 함(오라클 비침해).

## 설정 (`.claude/`)

- `settings.json` — 훅 배선 + 공유 허용 권한.
- `settings.local.json` — 자율 루프용 허용/거부(deny 가드레일) 권한 집합. 로컬 전용.

## 메타-하네스 (`foundry-harness/`)

제안서 → 오버피팅된 서브-하네스를 주조(`/forge`)하고 운영 학습으로 진화(`/evolve`)하는
공장. `principles.md`(헌법), `maze-auditor.md`(Policy Maze 자가 감사), `patterns/`(아키텍처
어휘), `templates/`(방출 산출물 틀). 이 EasyTrainer 하네스를 정리/평가할 때 이 도구를 쓴다.

## 별도 하네스 (`policy-lab/`) — EasyTrainer 위에서 도는 독립 프로젝트

정책 연구/토너먼트/진화 전용 하네스. EasyTrainer 하네스와 **분리**되어 자체 `.claude/`·원칙·채점·
진화 로그·야간 루프를 가진다. `cd policy-lab && claude` 로 운영. EasyTrainer 트리는 read(구조 맵) +
지정 위치 write (`EASYTRAINER_ROOT=$(git rev-parse --show-toplevel)`).
- 명령: `/lab-smoke` `/policy-design` `/policy-tournament` `/policy-evolve` `/lab-overnight`(야간 진입점).
- 에이전트: `policy-researcher`. 지도: [../policy-lab/README.md](../policy-lab/README.md),
  EasyTrainer 통합점: [../policy-lab/docs/easytrainer-integration.md](../policy-lab/docs/easytrainer-integration.md).
- foundry `/forge` 로 주조됨. EasyTrainer 의 `/overnight-queue`(기능 spec)와 역할이 겹치지 않는다.

## 영속 메모리

- `~/.claude/projects/-home-hjhj-EasyTrainer-v2-3-1/memory/` — 세션 간 유지되는 사실/피드백/
  프로젝트 메모. `MEMORY.md` 가 인덱스. 사실상 golden-principles + learning 의 일부 역할.

## 시스템 오브 레코드 (루트)

| 파일 | 역할 |
|------|------|
| `CLAUDE.md` | 세션 진입 가이드 (폴더 판단 + 하네스 포인터) |
| `FOLDERS.md` | 폴더별 책임 단일 출처 |
| `golden-principles.md` | 기계검증 가능한 규칙 (i18n·CSS·매뉴얼·lerobot read-only·경로 등) |
| `rubric.md` | 자율 사이클 / 정책 토너먼트 채점 기준 (생성≠평가) |
| `learning.md` | 관찰된 실패·가지치기·진화 로그 (foundry evolve 입력) |
| `docs/HARNESS.md` | 이 파일 — 하네스 일체의 지도 |

> foundry 템플릿(`golden-principles`/`rubric`/`manual-check.sh`)을 이 프로젝트에 맞게
> 인스턴스화한 것. `sprint-contract`/`agent`/`MANUAL` 템플릿은 기존(`docs/queue/_TEMPLATE.md`·
> `manual-writer.md`·home-next 매뉴얼)이 더 풍부해 **대체하지 않고 유지** (회귀 방지 — learning.md 참고).

---

## 동기화 불변식 (어기면 조용히 깨진다)

1. **매뉴얼 트리거 표면 glob 은 4곳에 사본이 있다** — 반드시 일치:
   - `.claude/agents/manual-writer.md` `description` (canonical)
   - `.claude/hooks/manual-update-reminder.sh` `TRIGGERS` (canonical)
   - `.claude/commands/feature-from-spec.md` §4.5
   - `.claude/agents/README.md` "매뉴얼 트리거 표면"
   현재 목록: `frontend/src/pages/v2/**`, `frontend/src/components/v2/**`, `release/ui/**`,
   `backend/api/routes/**`, `modules/*/module.json`, `README.md`.
2. **훅의 프로젝트 루트는 동적 도출** (`$CLAUDE_PROJECT_DIR`). 하드코딩 금지 (learning.md F1).
3. **새 라우트 블루프린트 추가 시** `scripts/smoke.sh` 의 GET 목록에 한 줄 추가 (안 하면 smoke 가 못 잡음).
4. **호스트 절대경로 금지** — 명령/스크립트는 repo-root 상대경로 또는 `$(git rev-parse --show-toplevel)` 사용.
