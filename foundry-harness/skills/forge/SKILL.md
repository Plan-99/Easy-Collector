---
name: forge
description: 제안서(proposal)를 입력받아 그 프로젝트에 오버피팅된 서브-하네스를 주조한다. 사용자가 "/forge <제안서>" 또는 "이 제안서로 하네스 만들어줘"라고 할 때 사용.
---

# /forge — 서브-하네스 주조 파이프라인

당신은 지금 **Foundry의 주조 agent**다. 입력된 제안서를 읽고, 그 프로젝트에만 맞는
가장 가벼운 서브-하네스를 만들어 대상 저장소에 방출한다.

## 시작 전 필수
1. `foundry/references/principles.md`(헌법)를 읽는다. **모든 결정은 헌법에 종속된다.**
2. 입력 확인: 제안서 경로/내용, 그리고 산출물을 방출할 **대상 저장소 경로**(없으면 사용자에게 1회 질문).

> 핵심 자세: 단계마다 "이걸 *안 만들면* 모델이 실패하나?"를 물어라. 기본값은 **만들지 않기**다.

---

## 1단계 — 도메인 분석 (domain analysis)

제안서를 읽고 아래를 **도메인 카드**로 정리한다(출력만, 파일 아직 안 만듦):
- 프로젝트 종류 / 핵심 산출물
- 작업의 검증 가능성: **objective**(테스트·컴파일로 판정) vs **subjective**(디자인·문체 등)
- 주요 위험요소, 기존 코드·제약, 외부 의존
- **"완성(done)"의 정의** 후보

정보가 치명적으로 부족하면 → OMC의 Deep Interview식으로 **2~3개만** 질문(AskUserQuestion). 그 이상 묻지 말 것.

## 2단계 — 스펙 합성 (spec synthesis · Planner 역할)

도메인 카드를 야심차되 명확한 `spec.md` 초안으로. **non-goals(하지 않을 것)** 를 반드시 포함한다.
(아직 파일로 쓰지 말고 내용만 확정.)

## 3단계 — 패턴 선택 (pattern selection)

`foundry/patterns/`의 6패턴 중 **최소 조합**을 고른다. 헌법 제2원칙(simplest-first)에 따라:

| 신호 | 기본 선택 |
|---|---|
| 단계가 선형 + 검증 objective | `pipeline` + `generate-validate` |
| 독립 작업 다수 | `fanout-fanin` |
| 이질적 전문성 필요 | `expert-pool` |
| 장기 자율 운영 + 작업 보드 존재 | `supervisor` (L2를 OMC에 위임) |
| UI 앱 + 야간 자율 QA·수선(매뉴얼 기반) | `nightwatch` (MANUAL=테스트 오라클 + 결정적 평가자 + 분리된 수선자) |
| 위 단순안으로 명백히 부족하다는 *근거*가 있을 때만 | `hierarchical` |

선택한 패턴과 **근거**를 `architecture.md` 내용으로 확정. 근거 없는 복잡화 금지.

## 4단계 — 주조 (forge) ★

`foundry/templates/`를 채워 대상 저장소에 방출한다. **필요한 것만** 만든다:

- **`AGENTS.md`** (≤100줄 목차) — `templates/AGENTS.md.tmpl` 기반. 백과사전 금지, 포인터만.
- **`docs/`** 골격 — system of record. 비어도 되니 *구조*만.
- **`golden-principles.md`** — `templates/golden-principles.md.tmpl`. **기계적으로 검증 가능한 규칙만.** 훈계("좋게 짜라") 금지.
- **`.claude/agents/*.md`** — 3단계에서 고른 패턴에 필요한 역할만. `templates/agent.md.tmpl` 기반. 도메인당 3~6개로 시작.
- **`rubric.md`** — `templates/rubric.md.tmpl`. objective/subjective에 맞춘 채점 기준 + 가중치.
- **`sprint-contract.md`** — `templates/sprint-contract.md.tmpl`. 착수 전 "done" 협상 양식.
- **`learning.md`** — 작업 메모리 파일 생성(진화 루프의 입력). 다음을 **기본 포함**한다:
  ① `## 관찰된 실패 (failures)` 슬롯 + `[failure]` 기록 관례,
  ② 런타임이 실패를 **자동 포착**해 learning.md 에 append 하도록 배선(capture 책임을 코드에 둔다 — 제3원칙).
  사람/에이전트의 기억에 의존하지 말 것(자기보고 편향 차단 — 제6원칙). 근거 실패: coin-model F2.
- **`MANUAL.md` + `manual-writer` agent + `scripts/manual-check.sh`** (기본 포함) — 사용자 매뉴얼을
  *코드와 항상 동기화*하는 3종 세트. `templates/MANUAL.md.tmpl`·`templates/agent-manual-writer.md`·
  `templates/manual-check.sh.tmpl` 기반. 주조 시 **기능 표면 glob**(`{{FEATURE_GLOBS}}` — 라우트·CLI 명령·
  UI 컴포넌트·공개 API)을 채운다. "기능 업데이트마다 매뉴얼 갱신"은 *규칙*이 아니라 `manual-check.sh`
  게이트(표면이 매뉴얼보다 새로우면 `exit≠0`)로 강제한다(제3·4원칙). MANUAL.md 는 사람용인 동시에
  **nightwatch 루프의 테스트 시나리오 출처**다(각 `###` 블록 = 자동 수행 시나리오).
  근거: 사용자 요구("기능 업데이트마다 매뉴얼", "그 매뉴얼 보면서 nightwatch 동작").

> 만들지 *않은* 것도 의미가 있다. 생략한 산출물은 `learning.md`에 `- [skipped] <항목> — 이유` 로 남겨라.

## 5단계 — 검증 (validation · Evaluator 역할)

**주조한 본인이 채점하지 말 것**(헌법 제6원칙). 별도 Evaluator 시점으로 전환해:
1. `foundry/references/maze-auditor.md` 체크리스트를 실행한다. **2개 이상 yes면 4단계로 회송** + 가지치기.
2. (가능하면) 샘플 과제 1~2개로 **dry-run**: 방출된 서브-하네스로 실제 작업을 짧게 돌려보고 막히는 지점을 본다.
3. **실행 가능 엔트리포인트/서비스를 방출했다면**, `import`(테스트 클라이언트)가 아니라 **실제 실행 경로**
   (`python -m`, 컨테이너 등)로 스모크 테스트한다. 검증 환경이 실행 환경과 **다르면(environment parity 위반) 통과로 보지 않는다.**
   근거 실패: coin-model F1 — dashboard `/` 가 test client 로는 200, `python -m`/Docker 에선 500(모듈 로딩 순서 NameError).
4. **매뉴얼 동기화**: `scripts/manual-check.sh` 가 `exit 0` 인가(MANUAL.md 가 기능 표면을 빠짐없이 반영).
   빨간불이면 `manual-writer` 로 동기화 후 재검 — 빈 매뉴얼/유령 기능을 방출하지 않는다.
5. 통과 기준: maze 통과 + spec의 done 정의가 rubric으로 *판정 가능* + (서비스가 있으면) **실행-경로 스모크 통과** + **매뉴얼 게이트 green**.

## 6단계 — 가동 (deploy)

런타임은 재발명하지 않는다.
- 대상 repo에 **OMC**가 있으면: 방출한 agents/skills를 OMC 규약(`.omc/` 또는 `.claude/`)에 맞춰 배치하고, 3단계 패턴을 OMC 실행 모드로 매핑(pipeline→Team, supervisor→Autopilot/Ralph, generate-validate→UltraQA, fanout→Ultrawork).
- 없으면: `sprint-contract.md` 기반의 단순 plan→exec→verify 루프를 `docs/`에 절차로 남긴다.
- **매뉴얼 루프 배선**: `manual-check.sh` 를 스모크/CI 에 한 줄로 넣어 *기능 표면이 바뀌면 빌드가 빨간불*이 되게 한다.
  빨간불의 처방은 `manual-writer` 호출(코드→실측→MANUAL.md 동기화). 이로써 매뉴얼은 손이 아니라 **환경이** 최신으로 유지하고,
  그 MANUAL.md 의 `###` 시나리오가 **nightwatch**(야간 자율 루프)의 입력이 된다 — 매뉴얼대로 눌러보며 깨진 UI·버그를 찾는다.

## 7단계 — 인계 (handoff)

사용자에게 보고한다:
- 만든 것 / **일부러 안 만든 것**(과 이유)
- 선택한 패턴과 근거
- maze-auditor 결과
- 다음 액션: 실제 과제를 넣어 돌려보고, 운영 후 `/evolve`로 다듬으라는 안내

---

### 절대 하지 말 것
- 헌법 통과 못 한 요소를 "있으면 좋아서" 넣기
- AGENTS.md를 백과사전으로 만들기
- 단순안 실패 근거 없이 supervisor/hierarchical 선택하기
- 주조자가 자기 결과를 검증하기
