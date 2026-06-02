# nightwatch round_0 — 2026-06-01 (interactive, headless)

- base URL: `http://localhost:5173` (quasar dev, hash routing). backend `:5000` up (healthz 200), ros2 down.
- runner: Playwright/Chromium via host nvm node v20.20.0 (`.claude/skills/playwright-skill`).
- 스크립트: `/tmp/nightwatch-detect.js`, 스크린샷: `/tmp/nightwatch-shots/*.png` (기능 판정 + 사람 시각 확인 완료).

## 결과: 6 PASS / 1 SKIP / 0 FAIL · 콘솔 에러 0 · 5xx 0

| scenario-id | status | evidence |
|---|---|---|
| nav-loads | PASS | `#q-app` 마운트, `/#/` → `/#/sensors` 정착. SensorPage 풀 렌더(사이드바+튜토리얼 토글). |
| nav-datasets | PASS | "데이터셋 관리" + "워크스페이스 선택" + "워크스페이스를 먼저 선택하세요" 빈 상태. |
| nav-train | PASS | "학습…" + 워크스페이스 게이트 빈 상태. |
| tr-stepper | SKIP | Train 이 워크스페이스 미선택으로 게이트됨 → stepper 도달 불가(정상 device-free 동작). |
| nav-planner | PASS | "플래너…" + "플래너 선택" + "준비 중..." 스피너. |
| pl-empty | PASS | 플래너 선택 셀렉터 노출. |
| tut-toggle-ui | PASS | 좌측 드로어 "튜토리얼 모드" + q-toggle 1개 (꺼짐). |

## 수선(fix)
- 없음 — FAIL 0. 제품 소스 무변경.

## scenario-mismatch / 사람 검토 필요
- 없음. 위 7개는 소스 유도 시나리오와 실제 UI 가 일치 → `01_navigation`(4), `pl-empty`, `tut-toggle-ui` 를
  `validate: confirmed` 로 승급.

## 미확정(다음 라운드)
워크스페이스/데이터/딥 인터랙션이 필요해 이번 헤드리스 라운드에서 미수행 (여전히 `first-run`):
- 02 `ds-toolbar`, `ds-open-episode` — 워크스페이스 선택 + 에피소드 데이터 필요.
- 03 `tr-policy-types`, `tr-hyperparam-help` — 워크스페이스+데이터셋 선택해 Step 2 진입 필요.
- 04 `pl-create-planner`, `pl-blocks-ui` — 플래너 생성/블록 편집 인터랙션 필요.
- 05 `tut-hints-visible` (heavy) — ros2 + MuJoCo sim 필요 (이번 미기동).

## 운영 메모
- detect 러너는 host node v12 가 아니라 **nvm node v20** 필요 (`export PATH="$HOME/.nvm/versions/node/v20.20.0/bin:$PATH"`).
  → `/nightwatch` 0단계에 반영 검토 (learning.md 후보).
- 이번 라운드 위해 `docker compose up -d frontend backend` 기동함 (실행 전 down 상태였음).
