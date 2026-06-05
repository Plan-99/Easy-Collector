# Pattern: supervisor (중앙 지휘 · 장기 자율)

**언제**: 장기간 자율 운영, 작업 보드(Linear 등)가 있고, 멈춘 작업을 자동 재시작해야 할 때.

**구조**: 보드를 **FSM**(대기/진행/검토/완료)으로. 작업당 agent 1개·격리 워크스페이스.
슈퍼바이저가 **stall 감지 → continuation context로 재시작**.

**비용(tax)**: 높음. **단순안 실패 근거가 있을 때만 채택**(헌법 제2원칙).

**OMC 매핑**: Autopilot / Ralph(persistent verify-fix 루프). rate-limit 자동 재개 활용.

**안티패턴**: 단발성·단순 작업에 도입 → 과잉 엔지니어링.
