# Pattern: hierarchical (계층 위임)

**언제**: 작업이 너무 커서 단일 agent의 context로 못 잡고, 재귀적 하위 분해가 필요할 때.

**구조**: 상위 agent가 큰 목표를 하위 목표로 쪼개 하위 agent에 위임, 결과를 다시 종합. 다단 중첩.

**비용(tax)**: 가장 높음. context 분절·중간 종합 손실 위험. **최후의 수단.**

**OMC 매핑**: Team을 중첩하거나 sub-harness를 재귀 forge.

**안티패턴**: 2계층이면 충분한데 더 깊게 → 종합 단계마다 정보 손실. 가능하면 `fanout-fanin`으로 평탄화.
