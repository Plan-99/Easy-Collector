# 01 — 기본 네비게이션 (device-free)

앱이 뜨고 사이드바로 device-free 라우트를 오갈 수 있는가. 가장 싼 smoke 시나리오.

### nav-loads — 앱 로드 + 기본 라우트
<!-- validate: confirmed (round_0 2026-06-01, headless) -->
- route: `/` (hash 라우팅, `/`→`/sensors` 리다이렉트)
- **단계**
  1. `$URL/#/` 접속.
- **기대결과**: 페이지가 크래시 없이 렌더되고 좌측 드로어(MainLayout)가 보인다. URL 이 `/#/sensors` 로 정착. 콘솔에 uncaught error 없음.
- **엣지**: backend 가 아직 부팅 중이면 spinner/플레이스홀더가 보여도 OK (흰 화면·JS 크래시는 FAIL).

### nav-datasets — Datasets 라우트
<!-- validate: confirmed (round_0 2026-06-01, headless) -->
- route: `/#/datasets`
- **단계**
  1. 좌측 드로어에서 **Datasets**(i18n `navDatasets`) 클릭 또는 `$URL/#/datasets` 직접 이동.
- **기대결과**: Datasets 페이지 렌더. 워크스페이스 선택 셀렉터(i18n `workspaceSelectLabel`)가 보인다. 워크스페이스 미선택이면 "Select Workspace First"(i18n `selectWorkspaceFirst`) 힌트가 보인다.
- **엣지**: 데이터셋 0개면 "데이터셋이 없습니다"(i18n `datasetEmpty`).

### nav-train — Train 라우트
<!-- validate: confirmed (round_0 2026-06-01, headless) -->
- route: `/#/train`
- **단계**
  1. **Train**(i18n `navTrain`) 클릭 또는 직접 이동.
- **기대결과**: Train 페이지의 3-step stepper 가 렌더. Step 1 제목(i18n `trainStep1Title`, "Select Datasets")이 보인다.
- **엣지**: 워크스페이스 미선택 → "Select Workspace First" 힌트.

### nav-planner — Planner 라우트
<!-- validate: confirmed (round_0 2026-06-01, headless) -->
- route: `/#/planner`
- **단계**
  1. **Planner**(i18n `navPlanner`) 클릭 또는 직접 이동.
- **기대결과**: Planner 페이지 렌더. 플래너 선택 셀렉터(i18n `plannerSelectLabel`)가 보인다.
- **엣지**: 플래너 미선택 → "Select Planner First"(i18n `plannerSelectFirst`) 힌트.
