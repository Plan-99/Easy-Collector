# 05 — 튜토리얼 모드 토글 + 힌트 (device-free)

좌측 드로어의 튜토리얼 토글과 페이지 내 TutorialHint 노출. UI 상태까지만 — 실제 MuJoCo sim
부팅(backend `/tutorial:start`)은 무겁고 flaky 할 수 있어 **선택(heavy)** 으로 분리.

### tut-toggle-ui — 토글 컨트롤 존재 + 상태 캡션
<!-- validate: confirmed (round_0 2026-06-01, headless; toggle present, sim-start not exercised) -->
- route: `/` (좌측 드로어, MainLayout)
- **단계**
  1. 아무 페이지에서 좌측 드로어의 **Tutorial Mode**(i18n `tutorialModeLabel`, `school` 아이콘) 항목 확인.
- **기대결과**: `q-toggle` 컨트롤이 보인다. 꺼진 상태면 캡션이 "Off"(i18n `tutorialOff`). 토글 클릭 시 캡션이 "Starting..."(i18n `tutorialSwitching`, 주황)으로 바뀐다 (busy 상태).
- **엣지**: 토글이 `busy` 동안 disable 되어야 한다. 시작 실패 시 빨간 에러 텍스트(`tutorial.lastError`)가 보이고 토글이 off 로 복귀 — **크래시는 FAIL, 에러 표시는 정상 동작**.

### tut-hints-visible — 튜토리얼 ON 시 힌트 노출 (heavy)
<!-- validate: first-run -->
<!-- heavy: backend /tutorial:start + MuJoCo sim 필요. 인프라 불안정하면 skip 가능. -->
- route: `/#/train`
- **단계**
  1. 튜토리얼 토글을 ON 하고 `tutorial.running` 이 될 때까지 대기 (status 폴링 ~3s, "Simulation is running" i18n `tutorialSimRunning` 캡션).
  2. Train 페이지로 이동.
- **기대결과**: `tutorial.running === true` 동안 페이지에 TutorialHint 블록(teal 배경, `lightbulb`/step 원형)이 보인다 (예: Step 1 힌트 i18n `tutorialTrainStep1`). 튜토리얼 OFF 면 힌트가 사라진다.
- **엣지**: sim 시작이 타임아웃되면 이 시나리오는 **skip 보고** (FAIL 아님 — 인프라 한계). 자동 수선 대상 아님.
