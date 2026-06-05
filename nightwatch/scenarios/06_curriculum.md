# 06 — Curriculum 페이지 (device-free)

"스스로 배우는 로봇" — 플래너 기반 자동 재학습 루프 UI. 실제 롤아웃(로봇 동작·학습)은
하지 않는다 — 빈 상태 / 플래너 선택 → 커리큘럼 패널 / 탭 전환 / 블록 설정 다이얼로그까지만.

> 커밋된 Playwright 검증 + 매뉴얼 스크린샷 스크립트:
> `backend/tests/curriculum/test_curriculum_ui.js`
> (`cd .claude/skills/playwright-skill && node run.js ../../../backend/tests/curriculum/test_curriculum_ui.js`)

### curr-empty — 플래너 미선택 빈 상태
<!-- validate: first-run -->
- route: `/#/curriculum`
- **단계**
  1. Curriculum 페이지로 이동.
- **기대결과**: 헤더에 "스스로 배우는 로봇"(i18n `currIntroTitle`)과 플래너 선택 셀렉터(i18n `currSelectPlanner`)가 보이고, 미선택이면 안내 문구 "플래너를 선택하면 해당 플래너의 커리큘럼을 설정할 수 있습니다."(i18n `currSelectPlannerHint`)가 보인다. 우상단 상태칩은 "대기"(i18n `currIdle`).
- **엣지**: 백엔드 기동 중이면 잠깐 스피너 가능(크래시 아님).

### curr-select-planner — 플래너 선택 → 커리큘럼 자동 생성/로드
<!-- validate: first-run -->
- route: `/#/curriculum`
- **단계**
  1. 플래너 셀렉터(`currSelectPlanner`)를 펼치고 튜토리얼 플래너(이름 접두어 `tutorial_e2e_planner`)를 선택.
- **기대결과**: 좌측에 3개 탭 플래너 / 디바이스 / 그룹 정책(i18n `currTabPlanner` / `currTabDevice` / `currTabPolicy`)이 나타나고, 우측 롤아웃 컨트롤 바에 시작 / 정지 / 전체초기화 / 업그레이드(i18n `currStart` / `currStop` / `currReset` / `currUpgradeNow`)와 Group 대시보드 카드가 보인다. (커리큘럼이 없으면 자동 생성되며 멱등.)
- **엣지**: 자동 생성 중 "커리큘럼 준비 중..."(i18n `currPreparing`) 스피너는 정상. 30s 내 패널이 안 뜨면 FAIL.

### curr-tabs — 좌측 탭 전환(플래너/디바이스/그룹 정책)
<!-- validate: first-run -->
- route: `/#/curriculum`
- **단계**
  1. 커리큘럼이 로드된 상태에서 디바이스 탭, 그룹 정책 탭을 차례로 클릭.
- **기대결과**: 디바이스 탭은 로봇/센서 섹션(i18n `currDeviceRobots` / `currDeviceSensors`)과 각 디바이스 토글/상태(켜짐·꺼짐)를 보인다. 그룹 정책 탭은 "그룹 선택"(i18n `currSelectGroup`) 셀렉터를 보이고, 그룹을 고르면 Stage Mission 폼(목표 성공 + 교정 개수 / 실패 저장 확률 / 저장, i18n `currTargetSuccess` / `currFailureSaveProb` / `currSave`)이 나타난다. 콘솔 `pageerror` 없음.
- **엣지**: 플래너가 사용하는 디바이스가 없으면 "이 플래너가 사용하는 디바이스가 없습니다"(i18n `currNoDevices`)가 정상.

### curr-block-settings — 블록 설정 다이얼로그(체크포인트/모션)
<!-- validate: first-run -->
- route: `/#/curriculum`
- **단계**
  1. 플래너 탭에서 설정 가능한 블록의 톱니(settings) 버튼을 클릭.
- **기대결과**: 다이얼로그가 열린다. 체크포인트 블록이면 "체크포인트별 학습 설정"(i18n `currCheckpointSettings`) + Base 데이터셋 + 판정 조건(최초 길이 제한 / 길이 제한 rate / 성공 임계값)이, 모션 블록이면 "영향 줄 그룹"(i18n `currAffectsGroup`) + 노이즈 rate / offset + 예상 노이즈 프리뷰가 보인다. 닫기/Esc로 닫으면 페이지 정상.
- **엣지**: 설정 가능한 블록이 0개면 톱니 버튼이 없는 것이 정상(다이얼로그 캡처 생략).

### curr-rollout-guard — 디바이스 미기동 시 시작 가드
<!-- validate: first-run -->
- route: `/#/curriculum`
- **단계**
  1. 디바이스를 켜지 않은 상태에서 우측 롤아웃 컨트롤 바를 확인.
- **기대결과**: "로봇과 센서를 모두 켜야 시작할 수 있습니다. (디바이스 탭에서 확인)"(i18n `currStartNeedsDevices`) 경고가 보이고, 타겟 그룹 미선택 또는 디바이스 미기동이면 시작 버튼이 비활성(`!allDevicesOn`)이다.
- **엣지**: device-free 환경에서는 실제 시작을 누르지 않는다(로봇 동작 유발 금지). 가드 노출까지만 검증.
