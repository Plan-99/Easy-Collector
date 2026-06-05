# 02 — Dataset 페이지 (device-free)

데이터셋/에피소드 브라우징. 디바이스 불필요 (DB 데이터만).

### ds-empty-or-list — 데이터셋 목록 또는 빈 상태
<!-- validate: first-run -->
- route: `/#/datasets`
- **단계**
  1. Datasets 페이지로 이동.
  2. 워크스페이스 셀렉터(i18n `workspaceSelectLabel`)에서 워크스페이스가 있으면 첫 항목 선택.
- **기대결과**: 워크스페이스 선택 후 좌측에 데이터셋 목록(`q-expansion-item`)이 보이거나, 데이터셋이 없으면 "데이터셋이 없습니다"(i18n `datasetEmpty`)가 보인다. 우측엔 "왼쪽에서 에피소드를 선택하세요"(i18n `datasetSelectEpisodeHint`).
- **엣지**: 워크스페이스가 하나도 없으면 "Select Workspace First" 힌트만 (FAIL 아님).

### ds-toolbar — 데이터셋 추가/불러오기 버튼 존재
<!-- validate: first-run -->
- route: `/#/datasets`
- **단계**
  1. 워크스페이스 선택 상태에서 헤더의 액션 버튼 확인.
- **기대결과**: "데이터셋 폴더 추가"(i18n `workspaceAddDatasetFolder`, icon `add`) 와 "데이터셋 불러오기"(i18n `workspaceImportDataset`, icon `file_upload`) 버튼이 보인다. 클릭 시 해당 폼 다이얼로그가 열린다 (`datasetAddFormTitle`).
- **엣지**: 다이얼로그를 열고 닫아도 페이지 크래시 없음.

### ds-open-episode — 에피소드 패널 표시
<!-- validate: first-run -->
- route: `/#/datasets`
- **단계**
  1. 에피소드가 있는 데이터셋 expansion 을 펼친다.
  2. 첫 에피소드 항목을 클릭한다.
- **기대결과**: 우측에 EpisodePanel 이 렌더 — 에피소드 이름 + 프레임 수(i18n `datasetFrames`) + Trim 섹션(`content_cut` 아이콘, i18n `datasetTrimRangeLabel`) + Range 슬라이더 + Trim 버튼(i18n `datasetTrimApply`). 닫기(`close`) 버튼 있음.
- **엣지**: 에피소드가 0개인 데이터셋을 펼치면 "데이터셋이 없습니다"류 빈 메시지 (크래시 아님). **이 시나리오는 데이터가 있어야 의미가 있으므로, 에피소드가 없으면 skip 으로 보고**.
