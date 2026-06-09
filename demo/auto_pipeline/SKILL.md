---
name: auto_pipeline
description: demo 환경의 데이터 수집 → ACT 학습 → 추론을 "실제 사용자처럼" EasyTrainer UI를 Playwright로 직접 조작해서 끝까지 돌리는 표준 파이프라인. demo 폴더의 국룰. backend collector/스크립트를 직접 호출하지 말고 반드시 UI를 통해서 진행한다.
---

# auto_pipeline — demo 자동 파이프라인 (UI + Playwright 국룰)

**demo 국룰:** demo 환경의 데이터 수집과 학습은 **항상 EasyTrainer UI 를 Playwright 로 직접
조작**해서 진행한다. `tutorial_collector_node` 같은 backend 스크립트를 직접 호출하지 않는다 —
실제 앱 사용자가 하는 방식 그대로(센서 등록 → 워크스페이스 → 데이터 수집 → 학습 → 추론)를
브라우저로 자동화한다. 이렇게 해야 수집/학습 경로가 제품과 1:1로 검증된다.

## 감시 가능하게 — 보이는 창 + 주기적 스크린샷 (필수)

수집·학습은 backend script 를 직접 찌르지 말고 **사람처럼 UI 를 Playwright 로 조작**한다. 그리고:

- **진행 창을 띄운다(headed)** — `chromium.launch({ headless: false })` + `DISPLAY=:1`. 사람이 화면에서
  직접 수집/학습이 잘 도는지 볼 수 있어야 한다. (collect-hold 스크립트는 workspace 페이지를 열어
  2개 wrist cam 라이브 피드 + 로봇 관절 + record 콘솔을 그대로 보여준다.)
- **같은 세션에서 주기적으로 스크린샷 + console-error 스캔** — 폴링 루프(30s)마다
  `page.screenshot({path:'/tmp/collect-mon/latest.png'})` + `page.on('console'/'pageerror')`. `Read` 로
  그 스크린샷을 주기적으로 확인해 UI 깨짐/버그가 없는지 *실측*한다(자기보고 금지).
- **새 탭/브라우저로 모니터링 금지 (단일 세션 불변식)** — `App.vue` 의 onMount `cleanup()` 이
  `/stop_process` 로 실행 중인 `record_episode` 를 죽인다. **수집/학습 중 앱을 새로 열면 그 작업이
  중단된다.** (실제로 학습 dry-run 으로 페이지를 한 번 더 열었다가 수집이 멈춘 사례.) 모니터링은
  *작업을 시작한 그 페이지* 에서만, 별도 작업은 끝난 뒤에.
- **부하 주의** — 단일 스레드 sim 은 host 부하에 민감. 수집/학습 중 rebuild·다중 브라우저 등 무거운
  작업 금지(planner 성공률 하락).

## 전제

- demo sim 이 켜져 있어야 한다 (상단 nav bar **Sim Activation** 버튼 → peg_in_hole → 실행하기,
  또는 `POST /api/demo:start {"demo_id":"peg_in_hole"}`). bridge 가 띄우는 sim 은
  **ROS_DOMAIN_ID=0**. demo peg 환경은 **wrist 2-cam** (`wrist_cam`, `wrist_cam_down`) 만 발행한다
  (`demo_pegtask.launch.py`). 카메라가 적을수록 단일스레드 sim 의 물리가 렌더에 starve 되지 않아
  모션플래너가 안정적으로 성공한다.
- Playwright 실행: nvm node v20 + `/home/hjhj/.cache/ms-playwright`,
  스킬 실행기 `cd .claude/skills/playwright-skill && DISPLAY=:1 node run.js /tmp/<script>.js`.
  프론트엔드 dev 서버는 `http://localhost:5173`.

## 1) 센서 — `tutorial_wrist_cam_2` (custom)

demo 워크스페이스는 wrist cam **2개**만 쓴다: 기존 `tutorial_wrist_cam`(/tutorial/wrist_cam) +
새로 만드는 `tutorial_wrist_cam_2`(/tutorial/wrist_cam_down). UI `/#/sensors` → `+` →
type=custom → 읽기 토픽 `/tutorial/wrist_cam_down/image_raw/compressed` → 추가.

## 2) 워크스페이스 — `peg_in_hole_sim`

`/#/workspace` → 워크스페이스 선택 → **새 워크스페이스 만들기 +** → 이름 `peg_in_hole_sim` → 생성.
- **로봇 설정** → 로봇 어셈블리 = `tutorial_agent`
- **센서 설정** → `+` → `tutorial_wrist_cam` + `tutorial_wrist_cam_2` 선택 → 저장
- **태스크 설정** → 에피소드 길이 = `320` (20Hz × 16s; 모션 ~13s 커버 + 여유. 너무 크면 저장이 느림)
- **데이터** 탭 → 데이터셋 폴더 추가 → 이름 `peg_demo` → 저장

## 3) 데이터 수집 — External + Motion Planning

워크스페이스 **설정** 탭 하단 수집 바:
- 데이터 수집용 데이터셋 = `peg_demo`
- 텔레오퍼레이션 타입 = **External + Motion Planning**
- ⚙️ → 서비스 이름 = `/tutorial/run_episode`
- Hz = 20, REC 의 home 배지는 **off** (모션플래너가 randomize 안에서 home 리셋을 직접 함)
- **REC** 클릭 → 이후 backend `record_episode` 프로세스가 독립적으로 돈다.
  - 매 에피소드: `/tutorial/run_episode` 호출(planner 가 randomize→pick→insert→ground-truth check),
    **성공한 에피소드만 자동 저장**, 실패는 자동 재시도(저장 안 함). COMPLETE 클릭 불필요.
  - 100개 모일 때까지 두고, 디스크(`/opt/easytrainer/datasets/<id>/*.parquet`)로 카운트.
    100개 되면 UI 또는 `POST /api/dataset/<id>/:stop_collection` 로 중지.

### 이게 동작하려면 필요한 backend 수정 (이미 반영됨 — `record_episode.py`)

motion_planning 수집이 원래는 깨져 있었다. 세 가지를 고쳐야 한다:
1. **deferred service start** — `/tutorial/run_episode` 호출을 `env.reset()` *이후*로 미룬다.
   `env.reset()` 은 `/tutorial/reset_world` 로 movable object 를 home 으로 되돌리는데, 이게 planner 의
   randomize 를 덮어써서 peg 가 home 으로 돌아가 매 에피소드 grasp 가 빈 공간을 잡고 실패했다.
   planner 호출을 env.reset 뒤로 미루면 randomize 가 "마지막 말" 이 된다.
2. **ground-truth success 파싱** — `resp.success`(gRPC 호출 성공 여부) 가 아니라 Trigger 응답
   `response_json` 안의 `success`(=peg 삽입 성공) 로 저장 여부를 판단. 안 그러면 실패 에피소드도 저장됨.
3. **motion_planning 에선 reset_tutorial_world 스킵** — planner 가 scene 을 관리하므로 중복/충돌 제거.

## 4) 학습 — ACT (UI 3-step stepper)

`/#/train` → 워크스페이스 = `peg_in_hole_sim`:
- **Step 1 데이터셋 선택**: `peg_demo` 카드 체크 → 계속
- **Step 2 정책 선택**: 정책 선택 → **새 정책 만들기 +** → 정책 이름 입력 → 정책 종류 = **ACT** →
  (기본 파라미터: chunk_size 15, ResNet18, lr 1e-5) → 저장 → 계속
- **Step 3 모델 학습**: 학습 서버 주소 = `http://localhost:5100` (로컬 training_server) →
  체크포인트 이름 → num_epochs(기본 1000; 성공률 낮으면 늘림) → **학습 시작**.
  TrainingDialog 의 progress/loss 로 모니터. 끝나면 Checkpoint 가 `finished`.

## 5) 추론 / 성공률

`/#/workspace` → `peg_in_hole_sim` → **추론** 탭 → CheckpointGraph 에서 finished 체크포인트 선택 →
MonitoringWindow 의 추론 바에서 Hz 설정 → **추론 시작**. Succeed 점수로 성공률 기록.
결과는 `demo/RESULTS.md` 에 남긴다.

## 검증된 사실 / 함정

- bridge sim = **ROS_DOMAIN_ID=0** (model-test 스킬의 1 은 구버전).
- demo peg sim 은 wrist 2-cam 만. 센서 매핑: `sensor_<id>` 가 dataset feature
  `observation.images.sensor_<id>` 로 1:1. (예: 7=wrist_cam, 12=wrist_cam_down)
- planner 성공률은 시스템이 idle 할 때 ~100%. rebuild/다중 sim 등으로 부하를 주면 물리가 starve 되어
  떨어진다 — 수집/학습 중에는 무거운 작업을 같이 돌리지 말 것.
- Playwright 폼 입력: EasyTrainer `FormDialog` 는 라벨이 input 위의 `div.col` 이다. xpath
  `//div[contains(@class,"col") and normalize-space(text())="<label>"]/ancestor::div[contains(@class,"row")][1]/following-sibling::label[1]`
  로 input/select 를 잡는다. q-select floating label 은 `.q-field__label`.

## 참고 스크립트

수집/설정 Playwright 스크립트 예시는 `/tmp/pw-*.js` (create-sensor, create/config-workspace,
create-dataset, start-collection) 패턴을 따른다. 헬퍼: 라벨→필드 xpath, q-select 메뉴 클릭,
multiselect_list q-item 클릭.
