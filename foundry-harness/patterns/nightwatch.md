# Pattern: nightwatch (매뉴얼 기반 자율 UI 검증·수선 루프)

**언제**: UI가 있는 앱에서 "사람이 자는 동안 깨진 화면·버그를 찾아 고치는 개발팀 루프"가 필요할 때.
기능이 늘 때마다 *항상 최신인 매뉴얼*을 테스트 명세로 재활용하고 싶을 때.

**구조** (매뉴얼이 단일 소스 = 사람용 문서 + 테스트 시나리오):
```
discover → spec(MANUAL) → detect → fix → verify  (green 또는 한계까지 반복)
  · MANUAL.md   각 기능을 ```nightwatch 블록(단계·관찰가능한 기대결과)으로. manual-writer 가 코드 실측해 갱신.
  · detect      평가자: 실제 브라우저(Playwright+system Chrome)로 시나리오 수행. 기능(expect-*) + 시각(baseline diff) 판정.
  · fix         수선자: 결함을 코드로 고침. 인터랙티브면 메인 에이전트 본인, 무인(cron)이면 headless(claude -p/gemini/OMC Ralph).
  · verify      detect 재실행. 종료 조건은 오직 평가자 green.
```

**핵심 원칙** (이 패턴이 인코딩하는 불변식):
- **생성≠평가(제6)**: 합격 판정은 *결정적 평가자(nightwatch)* 만. 수선자가 "고쳤다" 자기보고해도 무시.
- **오라클 비침해(제4)**: 수선자가 MANUAL·baseline·평가 스크립트(=시험지)를 못 고치게 — 편집도구만 허용(셸 차단) + 호출 전후 스냅샷/복원. 안 막으면 LLM이 매뉴얼·baseline을 고쳐 **가짜 green** 생성.
- **매뉴얼 staleness 게이트(제3·4)**: 기능 표면이 MANUAL 보다 새로우면 `exit≠0` → "기능 업데이트마다 매뉴얼"을 규칙 아닌 환경으로 강제.
- **재배포 후 재검(제4)**: 소스 수정 뒤 반드시 build → detect. 소스만 고치고 안 돌리는 자기기만 차단.

**비용(tax)**: 중간. Playwright/Chrome 의존 + 시각 baseline 관리. *관찰된 요구(밤샘 UI QA)* 있을 때만.

**OMC 매핑**: detect/verify=UltraQA류 게이트, fix=Ralph, 생존(rate-limit)=`omc wait`, 알림=notification tags.

**검증된 능력**: 버그 수정과 **없던 기능 추가가 같은 메커니즘**(매뉴얼=원하는 동작, 실패=아직 없음, 수선=구현). 발견(discovery) 에이전트가 *필요한 기능*을 제안→매뉴얼→자동 구현까지 됨(근거: k-aviation 글자크기 기능 자율 추가).

**안티패턴 / 함정**:
- 수선자에게 오라클 쓰기 허용 → 시험지 고쳐 가짜 green. (반드시 가드)
- 시각 baseline: 안티앨리어싱·애니메이션으로 flaky → 스크린샷 전 **애니메이션/트랜지션/캐럿 freeze**. 임계값 올리기로 때우지 말 것.
- **의도적 UI 변경**은 기존 시각 baseline 을 깸 → 루프가 "회귀"로 오인. 의도적 변경 뒤엔 **사람이 baseline 재승인**.
- **자동 제안 시나리오엔 검토 게이트**: LLM 이 만든 시나리오가 실제 UI 흐름(예: 접힌 목록)을 틀리게 가정할 수 있음.
- happy-path만 검증 말고 **경계·악성 입력**(빈/최대길이/0·결측) 시나리오 포함(golden-principles 기본 규칙과 동일 계열).
