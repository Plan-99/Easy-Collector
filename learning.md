# learning.md — 하네스 작업 메모리

> Foundry 진화 루프의 입력이자, 하네스 운영 중 관찰된 실패·가지치기·생략을 누적하는 파일.
> 사람/에이전트의 기억이 아니라 **여기에 기록**한다 (자기보고 편향 차단 — 헌법 제6원칙).
> 지도는 [docs/HARNESS.md](docs/HARNESS.md), 방법론은 [foundry-harness/](foundry-harness/).
>
> 기록 관례:
> - `[failure] Fn — <증상> / <원인> / <처방>` — 관찰된 실패. 규칙·수선의 근거가 된다.
> - `[fixed] <요소> — <무엇을 고쳤나> (근거: Fn)`
> - `[pruned] <요소> — 이유: <maze-auditor 항목/사유>`
> - `[skipped] <요소> — 이유` — 의도적으로 만들지/하지 않은 것.

## 관찰된 실패 (failures)

- `[failure] F1` — **죽은 훅 (환경 parity 위반).** 증상: `.claude/hooks/*.sh` 4개가 매
  Write/Edit 마다 실행되지만 아무 일도 안 함. 원인: 모든 훅이
  `PROJECT_ROOT="/home/airlab/Easy-Collector"` 를 하드코딩 — 실제 루트는
  `/home/hjhj/EasyTrainer_v2.3.1` 이라 모든 `file_path.startswith(project_root)` 검사가
  즉시 false → 조용히 무력화 (다른 호스트에서 체크아웃한 흔적). **순수 세금**: 효과 0, 비용
  매 편집마다 서브프로세스 4개. 처방: `$CLAUDE_PROJECT_DIR`(없으면 스크립트 위치 기준 2단계
  상위)로 동적 도출. 같은 하드코딩이 `model-test.md`·`quick-apply.md` 명령에도 있었음.
  교훈: **호스트 절대경로는 하네스에서 금지** (docs/HARNESS.md 불변식 #2·#4).

- `[failure] F2` — **detect 러너 node 버전.** 증상: `/nightwatch` detect 가 host 기본 `node`(v12.22.9)로는
  Playwright 를 못 띄움(needs ≥18). 처방: nvm 의 v20 사용 — `export PATH="$HOME/.nvm/versions/node/v20.20.0/bin:$PATH"`.
  Chromium 은 `~/.cache/ms-playwright` 에 이미 설치돼 있음. round_0 에서 적용해 6/7 PASS. nightwatch.md step 2 에 반영.

- `[failure] F3` — **프론트엔드 편집이 컨테이너에 반영 안 됨.** 증상: PlannerPage.vue 의 새 블록 폼이
  브라우저에 안 뜸(빈 폼). 원인: `easytrainer_frontend` 컨테이너는 호스트 `frontend/`가 아니라
  **`/opt/easytrainer/project/frontend`(런타임)를 `/app`에 마운트**한다 (backend 와 동일 패턴). 호스트
  편집 후 `bash scripts/quick_apply.sh ./ /opt/easytrainer/project` 를 **안 하면** Vite 가 옛 소스를 서빙.
  처방: 프론트 편집 후에도 반드시 quick_apply. (backend·ros2 와 동일하게 런타임 동기화 필요 — ros2 는
  추가로 colcon 빌드.) 디버깅에 시간 소모 → 이 사실을 CLAUDE.md/quick-apply.md 가 명시하면 좋음.

- `[failure] F4` — **joint_position 블록: "현재 자세 적용" 버튼이 신규 블록에서 안 보임(chicken-and-egg).**
  증상: tutorial_env 등에서 joint_position 블록 편집 시 펜던트 힌트는 "현재 자세 적용을 누르라"는데 그 버튼이
  없음. 원인: `PlannerPage.vue` 의 "현재 자세 적용"(`applyCurrentPos`) 버튼이 **"저장된 자세" 카드
  (`v-if="blockForm.positions[robot.id]"`) 안에만** 있어서, 저장된 자세가 없는 새 블록에선 카드+버튼이 모두
  숨겨짐. RobotPendant 는 jog/pose-preset 만 있고 현재자세 저장 기능 없음 → 첫 자세를 캡처할 방법이 없었음.
  처방: 로봇이 `status==='on'` 일 때 펜던트 위에 **항상 보이는 "현재 자세 적용" 버튼** 추가(F-fix). 검증:
  Playwright 로 신규 joint_position 블록에 버튼 1개 표시 + 클릭 시 "저장된 자세" 카드 출현 확인. (내 visual_reach
  작업과 무관한 기존 UI 버그. 로봇 off 면 펜던트·버튼 모두 숨고 off-hint 표시되는 것은 의도된 동작.)

## 가지치기 / 수선 기록 (evolve pass — 2026-06-01)

foundry-harness 의 maze-auditor + evolve 규율로 EasyTrainer 하네스를 1차 정리:

- `[fixed]` 훅 4개 (`manual-update-reminder`/`frontend-i18n-check`/`frontend-no-css`/
  `frontend-ui-test-reminder`) — 하드코딩 루트 → 동적 도출. 실행 경로로 검증 (실 payload 투입,
  CLAUDE_PROJECT_DIR 유/무 + 음성 대조 모두 통과). 근거: F1.
- `[fixed]` `model-test.md` step 0, `quick-apply.md` — `/home/airlab/Easy-Collector` 경로 →
  repo-root 상대경로. 근거: F1.
- `[fixed]` `policy-design.md`·`policy-tournament.md`·`model-test.md` — `/home/hjhj/EasyTrainer_v2.3.1`
  절대경로(현 호스트에선 동작하나 비이식적) → 상대경로.
- `[fixed]` `feature-from-spec.md` — base 체크아웃 mv 의 하드코딩 경로 → `BASE="$(git rev-parse
  --show-toplevel)"` 변수화 (worktree 안에서도 base 를 정확히 가리킴).
- `[fixed]` `CLAUDE.md` — 존재하지 않는 레이아웃 기술 수정: `src/backend/`→`backend/`,
  `src/ui/`→`frontend/`, `start_services.sh`(삭제됨)→컨테이너별 `entrypoint.sh`, docker 서비스명
  `service`→`frontend/backend/ros2`, `easy_collector_service`→`easytrainer_backend`.
- `[fixed]` `.claude/agents/README.md` — 매뉴얼 트리거 glob 에서 `backend/api/routes/**` 누락
  (drift) 보완 + `policy-researcher` 에이전트 항목 추가 (누락돼 있었음).
- `[fixed]` `settings.local.json` — 자율 루프용 허용/거부 집합 적용 (기존 3개 allow → 전체 집합 +
  deny 가드레일). `settings.local.json.proposed` 스캐폴드 제거.
## foundry 템플릿 인스턴스화 (2차 — 사용자 요청)

사용자가 "foundry 템플릿으로 기존 것 대체"를 요청. **회귀 0 원칙**으로, 부재했던 것만 신규 생성하고
기존이 더 풍부한 건 유지:

- `[added]` `golden-principles.md` (root) — 훅·memory 에 분산돼 있던 기계검증 규칙 9개를 1곳에 통합.
- `[added]` `rubric.md` (root) — 자율 사이클(A)·정책 토너먼트(B) 채점 기준 명시화 (기존엔 smoke exit/
  success-rate 암묵만). objective 게이트 위주로 제6원칙(생성≠평가) 준수.
- `[added]` `scripts/manual-check.sh` — manual-check.sh.tmpl 을 home-next 매뉴얼 트리에 맞춰
  인스턴스화. advisory 였던 `manual-update-reminder.sh` 위에 **hard staleness 게이트** 추가
  (제3·4원칙). 실행 경로 검증: 현재 `frontend/.../WorkspacePage.vue` 등 3개가 매뉴얼보다 새로워
  정확히 exit 1 (true positive).
- `[kept]` `docs/queue/_TEMPLATE.md`(↔sprint-contract), `manual-writer.md`(↔agent-manual-writer),
  home-next 매뉴얼(↔MANUAL.md), `policy-researcher.md`(↔agent.md) — 기존이 템플릿보다 풍부해
  대체=다운그레이드. 유지.
- `[skipped]` 루트 `AGENTS.md` — `CLAUDE.md`+`docs/HARNESS.md` 가 이미 TOC. 4번째 인덱스는 중복 세금.

## policy-lab 분리 (4차 — 사용자 요청)

정책 토너먼트/진화를 **EasyTrainer 위에서 도는 독립 프로젝트**로 분리:
- `[moved]` `/policy-design`·`/policy-tournament`·`/policy-evolve` + `policy-researcher` 에이전트 +
  `docs/evolve/` → `policy-lab/` (레포 내 top-level 별도 하네스). EasyTrainer `.claude/` 에서 제거 →
  EasyTrainer 루트 세션에선 `/policy-*` 안 보임(의도됨).
- `[added]` `policy-lab/` 에 자체 `.claude/{commands,agents,settings}`, AGENTS/golden-principles/rubric/
  learning/README/docs(easytrainer-integration 지도), 신규 `/lab-smoke`(readiness)·`/lab-overnight`(야간 진입점).
- 독립성: 하네스 소유권 분리. 결합: 정책 산출물은 EasyTrainer 트리에 써야 하므로 `EASYTRAINER_ROOT=$(git
  rev-parse --show-toplevel)`(랩이 레포 안 → toplevel=EasyTrainer 루트)로 read+write. 운영: `cd policy-lab && claude`.
- `[kept]` `/model-test` 는 EasyTrainer 측 유지 (튜토리얼 파이프라인 단일 평가, model_tester 도구 공유).
- `[note]` lab settings.local.json 에 `Bash(bash:*)` 광역 허용은 auto-mode classifier 가 차단(권한 확대) →
  좁은 세트로 두고 호스트 스크립트 호출은 프롬프트되게 함. 사용자가 필요시 좁은 규칙 추가.

## nightwatch 패턴 인스턴스화 (3차 — 사용자 요청)

device-free 표면 + 인터랙티브 우선으로 nightwatch(자율 UI 검증·수선 루프) 생성:

- `[added]` `/nightwatch` 명령 + `nightwatch/{README,scenarios/*,baselines/_pending,findings}`.
  detect(Playwright)→fix(제품 소스만)→verify(green까지). 오라클 비침해(scenarios/baselines 편집 금지)·
  매뉴얼 staleness 게이트·재배포 후 재검 불변식 인코딩.
- 시나리오 5개(navigation/datasets/train/planner/tutorial)는 **소스에서 유도** — Explore 에이전트로
  실제 i18n 라벨·셀렉터·관찰가능 결과 추출(지어내지 않음). 전부 `validate: first-run` — 첫 detect 에서
  실측 확정. 흐름 어긋나면 시나리오를 고치지 말고 findings 에 scenario-mismatch 로 기록(사람 검토).
- `[skipped]` "Settings" 시나리오 — 전역 설정 페이지가 **존재하지 않음**(Explore 확인). 유령 시나리오
  금지(golden-principles #4).
- `[skipped]` 무인(cron) 모드 — 인터랙티브에서 1회 green + scenario-mismatch 0 으로 시나리오 확정 후
  승격. 그 전엔 자율 커밋/sparse-worktree 가드 안 만든다(관찰된 안정성 없음).
- 디바이스 의존 페이지(Robot/Sensor/Assemble/Teleop)는 **범위 밖** — mock/실장비 없이 헤드리스 불가.

## 다음 진화까지 지켜볼 지표 (갱신)

- nightwatch 시나리오가 첫 detect 에서 몇 개나 scenario-mismatch 인가 (실측 확정률).
- 매뉴얼 트리거 glob 4-way 동기화 유지.
- 훅 발화 정상 (F1 재발 감시). 호스트 절대경로 재유입 감시.

## 다음 진화까지 지켜볼 지표

- 매뉴얼 트리거 glob 4-way 동기화가 유지되는가 (또 drift 나면 단일 출처 추출 검토).
- 훅이 실제로 발화하는가 (조용한 무력화 재발 감시 — F1 재현 방지).
- 호스트 절대경로가 새 명령/스크립트에 다시 스며드는가.
