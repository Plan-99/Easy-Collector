# Pattern: pipeline (순차)

**언제**: 작업이 선형 단계로 나뉘고, 각 단계 산출물이 다음 단계 입력일 때. (기본값 1순위)

**구조**: A → B → C. 각 단계 1개 agent. 단계 사이에 산출물 검증.

**비용(tax)**: 낮음. 가장 가벼운 패턴.

**OMC 매핑**: Team(canonical) — team-plan→exec→verify.

**안티패턴**: 단계 간 의존이 없는데 억지로 직렬화 → 그건 `fanout-fanin`이 맞다.
