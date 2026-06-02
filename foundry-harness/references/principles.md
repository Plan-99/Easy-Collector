# Foundry 헌법 (Principles)

> 이 파일은 Foundry가 무언가를 주조하거나 진화시킬 때 **항상 먼저 읽고 지켜야 하는 상위 규칙**이다.
> 충돌이 생기면 항상 이 헌법이 개별 패턴/템플릿보다 우선한다.
>
> **출처 표기 규칙**: 각 원칙 끝에 `> 출처:` 로 근거를 단다.
> 원칙의 *내용*은 모두 아래 6개 소스에서 추출했고, *7원칙으로 묶고 이름 붙인 구조화*는 Foundry의 재구성이다.
> 소스: Anthropic(Harness Design) / OpenAI(Harness Engineering) / Symphony / revfactory·harness / Henry Pan(Self-Improvement Harness) / oh-my-claudecode

## 제1원칙 — 모든 하네스 요소는 세금(tax)이다

agent 하나, skill 하나, 규칙 한 줄을 추가할 때마다 모델은 "본업"에 더해
"이 요소를 어떻게 다루지?"라는 부담을 진다. 따라서 추가의 기본 답은 **"넣지 않는다"** 이고,
넣으려면 다음 질문을 통과해야 한다:

> **"이 요소가 없으면 모델이 *실제로* 실패하는가?"**
> (관찰된 실패 사례가 있어야 한다. "있으면 좋아 보여서"는 탈락 사유.)

> 출처: **Henry Pan**, Self-Improvement Harness — *"each harness rule becomes a 'tax' on Task LLM capability"* ("Model-Specific Tax").

## 제2원칙 — 가장 단순한 안이 기본값 (simplest-first)

패턴 선택에서 항상 가장 단순한 패턴(보통 `pipeline` 또는 `generate-validate`)을 기본값으로 둔다.
복잡한 패턴(`supervisor`, `hierarchical`)은 **단순한 안이 실패한다는 증거**가 있을 때만 채택한다.

> 출처: **Anthropic**, Harness Design — *"the simplest solution possible, and only increase complexity when needed."*

## 제3원칙 — 규칙보다 인터페이스 (Interfaces over Rules)

문제가 생기면 규칙을 *추가*하기 전에, 모델에게 정보를 **보여주는 방식**을 먼저 바꾼다.
- 나쁜 예: "X를 하지 마라"는 규칙 10개 추가
- 좋은 예: 애초에 X가 불가능하거나 비자연스럽도록 상태/도구/문서 구조를 재설계
- 참고: append-only 상태가 prefix cache 적중률을 높이듯, 인터페이스 개선은 복리로 돌아온다.

> 출처: **Henry Pan**, Self-Improvement Harness — 섹션 *"Interfaces Over Rules"* + "State Redesign Over Rules"(append-only로 prefix cache 적중률 13%→77%).

## 제4원칙 — 말이 아니라 환경으로 막아라 (Deterministic Supervisor)

"이 파일은 건드리지 마"를 프롬프트로 부탁하지 않는다. **물리적으로 못 건드리게** 한다.
Foundry가 주조/진화할 때 손대도 되는 파일 범위를 명시적으로 좁힌다(sparse worktree 등).

> 출처: **Henry Pan**, Self-Improvement Harness — *"Deterministic Supervisor … sparse git worktrees … caught 15 protocol violations before contaminating runs."*

## 제5원칙 — 진화는 추가가 아니라 *덜어내기*도 포함한다

`/evolve`는 새 요소를 더하는 것만큼 **쓸모를 잃은 요소를 제거**하는 것이 본분이다.
특히 모델 세대가 올라가면 "옛 모델용 비계(scaffolding)"는 짐이 되므로 걷어낸다.

> 출처: **Anthropic**, Harness Design — "Evolution with Model Capability"(모델이 좋아지면 load-bearing 컴포넌트가 불필요한 오버헤드가 되므로 단순화). 보조적으로 Henry Pan의 "mechanisms proliferate → diminishing returns".

## 제6원칙 — 생성과 평가를 분리한다

만든 주체가 자기 결과를 채점하면 후하게 준다. 주조하는 agent와 검증하는 Evaluator,
일하는 Task LLM과 개선하는 Improvement Agent는 **항상 다른 역할**로 둔다.

> 출처: **Anthropic**, Harness Design — *"Separating Generation from Evaluation … Models tend to praise their own work uncritically."* Planner/Generator/Evaluator 3-에이전트 구조.

## 제7원칙 — 오버피팅과 일반화를 명시적으로 구분한다

서브-하네스는 *의도적으로* 그 프로젝트에 오버피팅된다(그게 목적). 단,
운영에서 얻은 교훈 중 **다른 프로젝트에도 일반화 가능한 것만** L0 patterns로 역류시킨다.
프로젝트 특수적 교훈은 그 서브-하네스에만 남긴다. (판정은 Promotion Gate가 한다.)

> 출처: "sub-harness/overfitting" 용어는 **사용자(의뢰인)** 정의. 초기↔운영 델타를 학습해 되먹이는 진화 메커니즘은 **revfactory/harness**의 `/harness:evolve`. Promotion Gate(통계 승급)는 **Henry Pan**의 "Candidate Promotion Gate".

---

### 자가 점검 한 줄
> "지금 내가 만드는 게 모델을 *본업에 집중하게* 하는가, 아니면 *새 과제(미로)를 주는가*?"
> 출처: **Henry Pan** — *"the Task LLM is no longer solving the original task, it is now navigating a harness policy maze."*
