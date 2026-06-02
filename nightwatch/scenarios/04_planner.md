# 04 — Planner 페이지 (device-free)

블록 플래너 UI. 실제 plan 실행(로봇 동작)은 하지 않는다 — 빈 상태/블록 편집 UI까지만.

### pl-empty — 플래너 미선택 빈 상태
<!-- validate: confirmed (round_0 2026-06-01, headless) -->
- route: `/#/planner`
- **단계**
  1. Planner 페이지로 이동.
- **기대결과**: 플래너 선택 셀렉터(i18n `plannerSelectLabel`)가 보이고, 미선택이면 "Select Planner First"(i18n `plannerSelectFirst`)가 보인다.
- **엣지**: —

### pl-create-planner — 플래너 생성 폼
<!-- validate: first-run -->
- route: `/#/planner`
- **단계**
  1. 플래너 셀렉터 펼치고 "Create New Planner"(노란색 항목) 클릭.
- **기대결과**: 플래너 생성 폼 다이얼로그(i18n `plannerCreateFormTitle`)가 열린다. 취소/닫기로 닫으면 페이지 정상.
- **엣지**: 폼을 빈 채로 제출 시 검증 메시지 (크래시 아님).

### pl-blocks-ui — 그룹/블록 영역 + 빈 상태
<!-- validate: first-run -->
- route: `/#/planner`
- **단계**
  1. 플래너가 선택된 상태(없으면 pl-create-planner 로 하나 생성 후)에서 우측 Plans 영역 확인.
- **기대결과**: Plans 헤더(i18n `plannerPlansTitle`, icon `hub`)가 보인다. plan group 이 없으면 "No plan groups"(i18n `plannerNoGroups`), 그룹은 있지만 블록이 없으면 "No blocks"(i18n `plannerNoBlocks`). 그룹에 **블록 추가**(`add`, tooltip i18n `plannerNewBlock`) 버튼이 보인다.
- **엣지**: 블록 0개에서 **Export**(i18n `plannerExport`) / **Run all**(i18n `plannerRunAllGroups`) 버튼은 비활성이어야 한다 (`!hasAnyBlock`).
