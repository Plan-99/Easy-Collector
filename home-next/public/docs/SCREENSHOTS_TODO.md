# 미캡처 스크린샷 TODO

`SCREENSHOTS.md`에서 아직 `[ ]` 상태인 슬롯만 추려낸 작업용 체크리스트입니다. 위에서 아래로 진행하면 첫 사용자 동선 그대로 따라가는 순서가 됩니다.

총 **111장 중 78장 완료, 33장 미캡처** (남은 분량 — datasets 9, training 8, data-collection 6, robots 4, teleoperation 3, inference 2, workspace 1). installation / quickstart / sensors / modules / planner 섹션은 모두 캡처 완료, planner의 checkpoint/run 슬롯은 통합 컷으로 채워져 있습니다.

캡처할 때마다 해당 줄 앞에 `[x]`로 체크해 주세요. 그 후 `SCREENSHOTS.md` 본 표에도 동일하게 체크하면 됩니다.

---

## 1. installation — 설치 (11장)

폴더: `public/docs/installation/`

런처 설치 직후, 깔끔한 새 우분투에서 끝까지 따라가며 한 번에 캡처하면 좋습니다.

- [x] `01-deb-download.png` — 다운로드 페이지 또는 GitHub Releases — `easytrainer_*.deb` 항목
- [x] `02-gui-install.png` — 파일 매니저에서 deb 더블클릭 후 뜨는 Ubuntu Software Install 다이얼로그 (Install 버튼 보이게)
- [x] `03-launcher-app-menu.png` — Activities/시작 메뉴에서 "Easy Trainer" 검색 결과 + 아이콘
- [x] `04-login-dialog.png` — PyQt "Easy Trainer 로그인" 다이얼로그 (560×260)
- [x] `05-browser-oauth.png` — 브라우저 `/auth/device?code=...` Google OAuth 동의 화면
- [x] `06-installer-eula.png` — 설치 마법사 page 0 — EULA + 동의함 체크박스
- [x] `07-installer-disk.png` — page 1 — 저장 공간 확인 (현재 여유 공간 X.X GB)
- [x] `08-installer-mode.png` — page 2 — 학습 서버 옵션 라디오 (원격 기본값 / 로컬)
- [x] `09-installer-progress.png` — page 3 — 설치 중 progress bar + 로그
- [x] `10-installer-done.png` — page 4 — 설치 완료 + "Easy Trainer 실행하기" 체크박스
- [x] `11-launcher-main.png` — 메인 런처 floating pill bar 전체 (상태등 포함)

---

## 2. quickstart — 빠른 시작 (39장)

폴더: `public/docs/quickstart/`

튜토리얼 모드(sim)를 켜고 가이드대로 따라가면서 단계별로 캡처. 일부 reference 섹션 슬롯도 여기서 함께 채워집니다.

### 튜토리얼 진입 (2장)
- [x] `01-tutorial-toggle.png` — MainLayout 사이드바 Tutorial 토글
- [x] `03-pipeline-guide.png` — PipelineGuideDialog — 6단계 stepper 펼친 상태

### 센서 확인 (3장)
- [x] `04-sensors-overview.png` — SensorPage 전체 (sim 카메라 카드들)
- [x] `05-sensor-card.png` — 단일 센서 카드 zoom (ONLINE 상태 + 토글)
- [x] `06-sensor-preview.png` — 하단 WebRTC 라이브 프리뷰

### 로봇 움직임 (4장)
- [x] `07-robots-overview.png` — RobotPage 전체
- [x] `08-robot-online.png` — 로봇 카드 ONLINE 상태
- [x] `09-robot-pendant-joint.png` — RobotPendant Joint 제어 영역
- [x] `10-robot-pendant-ik.png` — RobotPendant IK 제어 (xyz/rpy) zoom

### Assemble (5장)
- [x] `11-assemble-new-tab.png` — AssemblePage "New" 탭 진입 직후
- [x] `12-assemble-form.png` — AssemblyForm 우측 로봇 선택 + Left/Right 버튼
- [x] `13-assemble-canvas.png` — 좌측 Canvas — 그려진 조립 결과
- [x] `14-assemble-name.png` — Assembly 이름 입력 (Save 직전 화면)
- [x] `15-assemble-card.png` — 그리드에 새로 생성된 Assembly 카드

### Teleoperation 키보드 테스트 (4장)
- [x] `16-teleop-overview.png` — TeleoperationPage — 카드 mode indicator (키보드 ✅, 게임패드 ❌)
- [x] `17-teleop-setting-open.png` — 카드 ⚙️ 버튼 클릭 → TeleSettingDialog 열림
- [x] `19-teleop-keyboard.png` — Keyboard 탭 — step size + 키 매핑 표
- [x] `20-teleop-running.png` — TeleopConsole — Start Keyboard Teleop 활성 + 로그

### Workspace (8장)
- [x] `21-workspace-create.png` — "Create New Workspace +" select dropdown 펼친 상태
- [x] `22-workspace-form.png` — Workspace 생성 폼 다이얼로그
- [x] `23-workspace-sensor.png` — setting 탭 — Sensor Setting expansion (센서 추가 직후)
- [x] `24-workspace-sensor-crop.png` — sensor config 패널 — crop 캔버스 + Cropped Area 입력
- [x] `25-workspace-robot.png` — Robot Setting expansion + Assembly select
- [x] `26-workspace-homepose.png` — robot config — Home Pose 입력 + sync/save 버튼
- [x] `27-workspace-task.png` — Task Setting — Episode Length 입력
- [x] `28-workspace-dataset-add.png` — data 탭 — Add Dataset Folder 폼

### 데이터 수집 (4장)
- [x] `29-monitoring-overview.png` — MonitoringWindow 전체 (sensor feed + robot state)
- [x] `30-record-setup.png` — 수집 setup row (Dataset/Instruction/Teleop type/Hz/REC)
- [x] `31-moving-home.png` — "Moving to Home Pose" 스피너
- [x] `33-dataset-10-episodes.png` — data 탭 expansion — 10 에피소드 쌓인 모습

### 학습 (7장)
- [x] `35-train-step1.png` — TrainPage stepper 1단계 진입
- [x] `36-train-dataset-select.png` — step1 — 데이터셋 카드 선택 + Continue
- [x] `37-train-step2-newpolicy.png` — step2 — "Create New Policy +" 선택
- [x] `38-train-act-form.png` — Policy 타입 = ACT 파라미터 폼
- [x] `40-train-server-ok.png` — Training Server URL 입력 + 헬스체크 OK
- [x] `41-training-running.png` — TrainingDialog — progress bar + loss chart + ProcessConsole
- [x] `42-train-done.png` — 학습 완료 — 우하단 floating button

### 추론 (3장)
- [x] `43-inference-list.png` — Workspace inference 탭 — 체크포인트 카드
- [x] `44-inference-setup.png` — MonitoringWindow inference 헤더 — Hz + Start Inference
- [x] `45-inference-running.png` — 추론 중 — Succeed / OOD 배지

---

## 3. modules — 모듈 추가 (8장)

폴더: `public/docs/modules/`

런처 floating pill bar에서 모듈 관리 다이얼로그 한 번 열고 시나리오대로 클릭하면서 캡처.

- [x] `01-launcher-modules-button.png` — 런처 pill bar 🧩 모듈 관리 hover
- [x] `02-modules-dialog.png` — 모듈 관리 다이얼로그 — 3 카테고리 (로봇/센서/확장)
- [x] `03-module-installing.png` — 설치 진행 중 ⏳
- [x] `04-module-installed.png` — 설치 완료 (v버전 + 초록색)
- [x] `05-module-paid.png` — 유료 모듈 — "결제" 버튼 + ₩가격
- [x] `06-module-checkout.png` — "결제 진행" 다이얼로그 (URL + 복사)
- [x] `07-module-after-pay.png` — 결제 완료 후 자동 설치
- [x] `08-module-removed.png` — "제거" 클릭 후 미설치 상태

---

## 4. sensors — 센서 추가 (6장)

폴더: `public/docs/sensors/`

타입별 SensorForm 폼 + 등록 직후 ONLINE 카드 한 쌍씩.

- [x] `realsense-01-form.png` — SensorForm — Type=realsense, Serial Number 입력
- [x] `realsense-02-online.png` — Realsense 카드 ONLINE + 라이브 프리뷰
- [x] `webcam-01-form.png` — SensorForm — Type=webcam, Device Index
- [x] `webcam-02-preview.png` — Webcam 라이브 프리뷰
- [x] `custom-01-form.png` — SensorForm — Type=custom, Read Topic, Msg Type, Resolution
- [x] `custom-02-topic-on.png` — Custom 센서 TOPIC ON 상태

---

## 5. robots — 로봇 추가 (5장)

폴더: `public/docs/robots/`

- [ ] `piper-01-form.png` — RobotForm — Type=piper, CAN Port=can0
- [ ] `piper-02-online.png` — Piper 카드 ONLINE
- [ ] `piper-03-pendant.png` — Piper RobotPendant — Joint + IK
- [x] `custom-01-form.png` — RobotForm — Role / Joint Names / Bounds / Read·Write 토픽 / IK Settings JSON 한 화면
- [ ] `custom-02-online.png` — Custom 로봇 TOPIC ON 상태

---

## 6. teleoperation — 텔레오퍼레이션 (6장)

폴더: `public/docs/teleoperation/`

- [ ] `assembly-02-role-buttons.png` — single_arm / tool 역할별 Left·Right 슬롯 버튼
- [ ] `ee-01-general-tab.png` — TeleSettingDialog General 탭
- [ ] `ee-02-offset-explained.png` — EE offset x/y/z 입력 + 설명 텍스트
- [x] `leader-01-tab.png` — Leader Robot 탭 — 좌측 Start Leader Robot 버튼 + 우측 Stepper
- [x] `leader-02-origin-mapping.png` — Origin Setting — DXL 카드 드래그 + Set as Gripper 버튼
- [x] `testing-01-console.png` — TeleopConsole 초기 상태

---

## 7. workspace — 워크스페이스 (1장)

폴더: `public/docs/workspace/`

- [x] `create-03-edit-delete.png` — edit / delete 버튼
- [x] `robots-02-list.png` — Assembly 내부 로봇 리스트
- [x] `dataset-01-add.png` — Add Dataset Folder 버튼
- [ ] `dataset-04-incompatible.png` — metadata remap 워닝

---

## 8. data-collection — 데이터 수집 (8장)

폴더: `public/docs/data-collection/`

- [ ] `mode-01-select.png` — Teleop Type select 펼침 (keyboard/leader_robot/vive/...)
- [ ] `mode-02-keyboard.png` — keyboard 모드 — Step Size 입력 추가
- [ ] `mode-03-leader.png` — leader_robot 모드
- [x] `home-01-toggle.png` — home 배지 — 파란색(ON) / 회색(OFF) 두 상태
- [x] `btn-01-toolbar.png` — MonitoringWindow 툴바 — SUCCESS / DONE / STOP 3개 버튼
- [ ] `replay-01-select.png` — 워크스페이스 data 탭 — 에피소드 선택
- [ ] `replay-02-replay-panel.png` — Replay 헤더 — Action Type(qaction/ee_delta_action) + Capture Episode + Target Dataset
- [ ] `replay-03-progress.png` — Capturing X% progress

---

## 9. datasets — 데이터셋 관리 (9장)

폴더: `public/docs/datasets/`

이미 수집된 데이터셋이 있어야 캡처할 수 있는 화면들.

- [ ] `edit-00-context-menu.png` — 데이터셋 우클릭 메뉴 — Edit / Augment / Merge / Delete
- [ ] `edit-01-edit.png` — Edit Dataset 폼
- [ ] `edit-02-merge.png` — Merge Dataset 폼
- [ ] `edit-03-remap.png` — metadata remap (sensor/robot select)
- [ ] `edit-04-delete.png` — Delete 확인
- [ ] `augment-01-context-menu.png` — 데이터셋 우클릭 — Augment Dataset 메뉴
- [ ] `augment-02-dialog.png` — DataAugmentationDialog 전체
- [ ] `episode-edit-01-panel.png` — 데이터셋 탭 — 에피소드 클릭 시 우측 EpisodePanel
- [ ] `episode-edit-02-trim.png` — Trim 슬라이더 + Apply 버튼

---

## 10. training — 학습 (8장)

폴더: `public/docs/training/`

- [ ] `server-01-launcher-btn.png` — 런처 pill bar 🔥 학습 서버 hover
- [ ] `server-02-dialog.png` — 학습 서버 관리 다이얼로그 (전체)
- [ ] `server-03-install-run.png` — 설치 / 실행 버튼 + 진행 로그 + 실행 중 상태
- [ ] `policy-01-select.png` — step2 Policy select 펼침
- [ ] `policy-04-save-as.png` — "Save Policy As" 다이얼로그
- [ ] `params-01-form.png` — Step 3 — Checkpoint Name + lr / batch_size / epochs 등 학습 폼
- [ ] `queue-01-start.png` — "Start Training" 버튼
- [ ] `queue-04-loss-chart.png` — loss chart zoom (Train/Validate)

---

## 11. inference — 추론 (2장)

폴더: `public/docs/inference/`

- [ ] `select-02-context.png` — 우클릭 메뉴 (Show Details / Edit / OOD / Export / Delete)
- [ ] `select-03-info-dialog.png` — Show Details — CheckpointInfo 다이얼로그

---

## 12. planner — 플래너 (3장)

폴더: `public/docs/planner/`

- [x] `create-04-edit-delete.png` — 플래너 항목 우측 edit/delete 아이콘
- [x] `blocks-03-name.png` — 블록 이름 입력 필드
- [x] `blocks-04-save.png` — 블록 다이얼로그 저장 버튼

> `create-02-new.png`(영문 다이얼로그)을 Step 2에서 재사용하므로 별도의 `create-03-form.png`은 두지 않음. `blocks-checkpoint.png`(체크포인트 블록 select+파라미터 통합)와 `run.png`(플랜 실행 통합 컷)도 이미 캡처됨.

---

## 캡처 우선순위

1. **installation (11장)** — 첫 사용자 진입점. 미캡처면 매뉴얼 첫 페이지가 비어 보임.
2. **quickstart (39장)** — 가장 가치 높은 가이드. 한 번에 sim 모드로 끝까지 따라가며 일괄 캡처 가능.
3. **modules / training / inference (18장)** — 자주 참조됨.
4. **sensors / robots (11장)** — 실제 하드웨어 필요.
5. 나머지 reference 섹션 — 점진적으로.
