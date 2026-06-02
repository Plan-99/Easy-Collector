# Pattern: generate-validate (품질 게이트)

**언제**: 결과 품질이 중요하고, 만든 주체와 검증 주체를 분리해야 할 때. (거의 항상 동반되는 보조 패턴)

**구조**: Generator가 만들고 → 독립된 Evaluator가 rubric으로 채점 → 통과 못하면 피드백 후 반복.

**비용(tax)**: 낮음~중간. 단, 반복 루프가 무한정 돌지 않게 종료 조건 필수.

**핵심**: 생성≠평가 (헌법 제6원칙). Evaluator는 회의적으로 튜닝.

**OMC 매핑**: UltraQA(품질 게이트 cycling).

**안티패턴**: Generator가 자기 결과를 채점 → 항상 후하게 줌.
