# 매뉴얼 스크린샷 마스터 리스트

이 파일은 Easy Trainer 매뉴얼(`home-next/src/app/docs/_content/ko/`)에서 참조하는 모든 스크린샷의 단일 출처입니다.

## 규칙

- **저장 위치**: 같은 폴더 안 — `home-next/public/docs/<section>/<NN>-<short-name>.png`
- **권장 해상도**:
  - 데스크톱 풀스크린 UI: 1600 × 1000 px
  - 다이얼로그/패널: 800 × 600 px
  - 단일 카드/위젯 zoom: 600 × 400 px
- **형식**: PNG 우선. 3 MB를 넘으면 `pngquant --quality=70-90` 또는 `oxipng -o 4`로 압축.
- **민감 정보**: IP, 시리얼번호, 사용자 이메일은 캡처 전후로 마스킹.
- **파일명 컨벤션**: zero-padded 번호 + kebab-case 영문. 예: `09-installer-eula.png`.
- **상태 표기**: 캡처 완료한 항목은 체크박스 체크 (`- [x]`). MDX는 `<DocsImage status="missing" />`로 누락 표시.

> 매뉴얼 작성 에이전트가 누락 스크린샷 목록을 자동으로 추출하므로, 이 파일의 체크박스 상태를 진실로 유지해 주세요.

---

## 1. installation — 설치

폴더: `public/docs/installation/`

GUI 흐름 우선. 터미널 명령은 트러블슈팅에서만 다루므로 별도 캡처 슬롯 없음.

| # | 파일명 | 캡처 대상 (UI 위치) |
|---|---|---|
| - [ ] 01 | `01-deb-download.png` | 다운로드 페이지 또는 GitHub Releases — easytrainer_*.deb 항목 |
| - [ ] 02 | `02-gui-install.png` | 파일 매니저에서 deb 더블클릭 후 뜨는 Ubuntu Software Install 다이얼로그 (Install 버튼 보이게) |
| - [ ] 03 | `03-launcher-app-menu.png` | Activities/시작 메뉴에서 "Easy Trainer" 검색 결과 + 아이콘 |
| - [ ] 04 | `04-login-dialog.png` | PyQt "Easy Trainer 로그인" 다이얼로그 (560×260) |
| - [ ] 05 | `05-browser-oauth.png` | 브라우저 `/auth/device?code=...` Google OAuth 동의 화면 |
| - [ ] 06 | `06-installer-eula.png` | 설치 마법사 page 0 — EULA + 동의함 체크박스 |
| - [ ] 07 | `07-installer-disk.png` | page 1 — 저장 공간 확인 (현재 여유 공간 X.X GB) |
| - [ ] 08 | `08-installer-mode.png` | page 2 — 학습 서버 옵션 라디오 (원격 기본값 / 로컬) |
| - [ ] 09 | `09-installer-progress.png` | page 3 — 설치 중 progress bar + 로그 |
| - [ ] 10 | `10-installer-done.png` | page 4 — 설치 완료 + "Easy Trainer 실행하기" 체크박스 |
| - [ ] 11 | `11-launcher-main.png` | 메인 런처 floating pill bar 전체 (상태등 포함) |

---

## 2. quickstart — 빠른 시작

폴더: `public/docs/quickstart/`

### 튜토리얼 모드
| # | 파일명 | 캡처 대상 |
|---|---|---|
| - [ ] 01 | `01-tutorial-toggle.png` | MainLayout 사이드바 Tutorial 토글 |
| - [ ] 03 | `03-pipeline-guide.png` | PipelineGuideDialog — 6단계 stepper 펼친 상태 |

### 센서 확인
| # | 파일명 | 캡처 대상 |
|---|---|---|
| - [ ] 04 | `04-sensors-overview.png` | SensorPage 전체 (sim 카메라 카드들) |
| - [ ] 05 | `05-sensor-card.png` | 단일 센서 카드 zoom (ONLINE 상태 + 토글) |
| - [ ] 06 | `06-sensor-preview.png` | 하단 WebRTC 라이브 프리뷰 |

### 로봇 움직임
| # | 파일명 | 캡처 대상 |
|---|---|---|
| - [ ] 07 | `07-robots-overview.png` | RobotPage 전체 |
| - [ ] 08 | `08-robot-online.png` | 로봇 카드 ONLINE 상태 |
| - [ ] 09 | `09-robot-pendant-joint.png` | RobotPendant Joint 제어 영역 |
| - [ ] 10 | `10-robot-pendant-ik.png` | RobotPendant IK 제어 (xyz/rpy) zoom |

### Assemble
| # | 파일명 | 캡처 대상 |
|---|---|---|
| - [ ] 11 | `11-assemble-new-tab.png` | AssemblePage "New" 탭 진입 직후 |
| - [ ] 12 | `12-assemble-form.png` | AssemblyForm 우측 로봇 선택 + Left/Right 버튼 |
| - [ ] 13 | `13-assemble-canvas.png` | 좌측 Canvas — 그려진 조립 결과 |
| - [ ] 14 | `14-assemble-name.png` | Assembly 이름 입력 (Save 직전 화면) |
| - [ ] 15 | `15-assemble-card.png` | 그리드에 새로 생성된 Assembly 카드 |

### Teleoperation 키보드 테스트
| # | 파일명 | 캡처 대상 |
|---|---|---|
| - [ ] 16 | `16-teleop-overview.png` | TeleoperationPage — 카드 mode indicator (키보드 ✅, 게임패드 ❌) |
| - [ ] 17 | `17-teleop-setting-open.png` | 카드 ⚙️ 버튼 클릭 → TeleSettingDialog 열림 |
| - [ ] 19 | `19-teleop-keyboard.png` | Keyboard 탭 — step size + 키 매핑 표 |
| - [ ] 20 | `20-teleop-running.png` | TeleopConsole — Start Keyboard Teleop 활성 + 로그 |

### Workspace
| # | 파일명 | 캡처 대상 |
|---|---|---|
| - [ ] 21 | `21-workspace-create.png` | "Create New Workspace +" select dropdown 펼친 상태 |
| - [ ] 22 | `22-workspace-form.png` | Workspace 생성 폼 다이얼로그 |
| - [ ] 23 | `23-workspace-sensor.png` | setting 탭 — Sensor Setting expansion (센서 추가 직후) |
| - [ ] 24 | `24-workspace-sensor-crop.png` | sensor config 패널 — crop 캔버스 + Cropped Area 입력 |
| - [ ] 25 | `25-workspace-robot.png` | Robot Setting expansion + Assembly select |
| - [ ] 26 | `26-workspace-homepose.png` | robot config — Home Pose 입력 + sync/save 버튼 |
| - [ ] 27 | `27-workspace-task.png` | Task Setting — Episode Length 입력 |
| - [ ] 28 | `28-workspace-dataset-add.png` | data 탭 — Add Dataset Folder 폼 |

### 데이터 수집 (10 에피소드)
| # | 파일명 | 캡처 대상 |
|---|---|---|
| - [ ] 29 | `29-monitoring-overview.png` | MonitoringWindow 전체 (sensor feed + robot state) |
| - [ ] 30 | `30-record-setup.png` | 수집 setup row (Dataset/Instruction/Teleop type/Hz/REC) |
| - [ ] 31 | `31-moving-home.png` | "Moving to Home Pose" 스피너 |
| - [ ] 33 | `33-dataset-10-episodes.png` | data 탭 expansion — 10 에피소드 쌓인 모습 |

### 학습 (기본 ACT)
| # | 파일명 | 캡처 대상 |
|---|---|---|
| - [ ] 35 | `35-train-step1.png` | TrainPage stepper 1단계 진입 |
| - [ ] 36 | `36-train-dataset-select.png` | step1 — 데이터셋 카드 선택 + Continue |
| - [ ] 37 | `37-train-step2-newpolicy.png` | step2 — "Create New Policy +" 선택 |
| - [ ] 38 | `38-train-act-form.png` | Policy 타입 = ACT 파라미터 폼 |
| - [ ] 40 | `40-train-server-ok.png` | Training Server URL 입력 + 헬스체크 OK |
| - [ ] 41 | `41-training-running.png` | TrainingDialog — progress bar + loss chart + ProcessConsole |
| - [ ] 42 | `42-train-done.png` | 학습 완료 — 우하단 floating button |

### 추론
| # | 파일명 | 캡처 대상 |
|---|---|---|
| - [ ] 43 | `43-inference-list.png` | Workspace inference 탭 — 체크포인트 카드 |
| - [ ] 44 | `44-inference-setup.png` | MonitoringWindow inference 헤더 — Hz + Start Inference |
| - [ ] 45 | `45-inference-running.png` | 추론 중 — Succeed / OOD 배지 |

---

## 3. modules — 모듈 추가 (런처)

폴더: `public/docs/modules/`

| # | 파일명 | 캡처 대상 |
|---|---|---|
| - [ ] 01 | `01-launcher-modules-button.png` | 런처 pill bar 🧩 모듈 관리 hover |
| - [ ] 02 | `02-modules-dialog.png` | 모듈 관리 다이얼로그 — 3 카테고리 (로봇/센서/확장) |
| - [ ] 03 | `03-module-installing.png` | 설치 진행 중 ⏳ |
| - [ ] 04 | `04-module-installed.png` | 설치 완료 (v버전 + 초록색) |
| - [ ] 05 | `05-module-paid.png` | 유료 모듈 — "결제" 버튼 + ₩가격 |
| - [ ] 06 | `06-module-checkout.png` | "결제 진행" 다이얼로그 (URL + 복사) |
| - [ ] 07 | `07-module-after-pay.png` | 결제 완료 후 자동 설치 |
| - [ ] 08 | `08-module-removed.png` | "제거" 클릭 후 미설치 상태 |

---

## 4. sensors — 센서 추가

폴더: `public/docs/sensors/`

| 페이지 | # | 파일명 | 캡처 대상 |
|---|---|---|---|
| Realsense | - [ ] 01 | `realsense-01-form.png` | SensorForm — Type=realsense, Serial Number 입력 |
| Realsense | - [ ] 02 | `realsense-02-online.png` | Realsense 카드 ONLINE + 라이브 프리뷰 |
| Webcam | - [ ] 01 | `webcam-01-form.png` | SensorForm — Type=webcam, Device Index |
| Webcam | - [ ] 02 | `webcam-02-preview.png` | Webcam 라이브 프리뷰 |
| Custom | - [ ] 01 | `custom-01-form.png` | SensorForm — Type=custom, Read Topic, Msg Type, Resolution |
| Custom | - [ ] 02 | `custom-02-topic-on.png` | Custom 센서 TOPIC ON 상태 |

---

## 5. robots — 로봇 추가

폴더: `public/docs/robots/`

| 페이지 | # | 파일명 | 캡처 대상 |
|---|---|---|---|
| Piper | - [ ] 01 | `piper-01-form.png` | RobotForm — Type=piper, CAN Port=can0 |
| Piper | - [ ] 02 | `piper-02-online.png` | Piper 카드 ONLINE |
| Piper | - [ ] 03 | `piper-03-pendant.png` | Piper RobotPendant — Joint + IK |
| Custom | - [ ] 01 | `custom-01-role.png` | RobotForm — Role select (Single Arm / Dual Arm / Tool) |
| Custom | - [ ] 02 | `custom-02-joints.png` | Joint Names 그리드 + Tool Joint 체크박스 |
| Custom | - [ ] 03 | `custom-03-bounds.png` | Joint Lower / Upper Bounds 입력 |
| Custom | - [ ] 04 | `custom-04-topics.png` | Read Topic / Write Type / Write Topic |
| Custom | - [ ] 05 | `custom-05-ik-json.png` | IK Settings (JSON) textarea + 예시 placeholder |
| Custom | - [ ] 06 | `custom-06-online.png` | Custom 로봇 TOPIC ON 상태 |

---

## 6. teleoperation — 텔레오퍼레이션

폴더: `public/docs/teleoperation/`

| 페이지 | # | 파일명 | 캡처 대상 |
|---|---|---|---|
| Assembly | - [ ] 01 | `assembly-01-form-overview.png` | AssemblyForm 좌/우 패널 전체 |
| Assembly | - [ ] 02 | `assembly-02-singlearm-buttons.png` | single_arm 역할 — Left/Right 버튼 |
| Assembly | - [ ] 03 | `assembly-03-tool-buttons.png` | tool 역할 — Left/Right 버튼 |
| Assembly | - [ ] 04 | `assembly-04-canvas.png` | 좌측 Canvas zoom |
| End-effector | - [ ] 01 | `ee-01-general-tab.png` | TeleSettingDialog General 탭 |
| End-effector | - [ ] 02 | `ee-02-offset-explained.png` | EE offset x/y/z 입력 + 설명 텍스트 |
| Keyboard | - [ ] 01 | `keyboard-01-tab.png` | TeleSettingDialog Keyboard 탭 전체 |
| Keyboard | - [ ] 02 | `keyboard-02-mapping-table.png` | 키 매핑 표 (Left/Right arm) |
| Keyboard | - [ ] 03 | `keyboard-03-step-size.png` | Step size 입력 + 설명 |
| Leader | - [ ] 01 | `leader-01-tab.png` | Leader Robot 탭 전체 (좌측 콘솔 + 우측 stepper) |
| Leader | - [ ] 02 | `leader-02-start-leader.png` | "Start Leader Robot" 버튼 활성 |
| Leader | - [ ] 03 | `leader-03-stepper-origin.png` | Origin Setting 단계 |
| Leader | - [ ] 04 | `leader-04-drag-drop-joint.png` | drag-drop 영역 + DXL ID/port/origin |
| Leader | - [ ] 05 | `leader-05-set-gripper.png` | "Set as Gripper" 버튼 |
| Testing | - [ ] 01 | `testing-01-console.png` | TeleopConsole 초기 상태 |
| Testing | - [ ] 02 | `testing-02-running.png` | Start Keyboard Teleop 활성 상태 |
| Testing | - [ ] 03 | `testing-03-logs.png` | 키보드 / 리더 로그 영역 |

---

## 7. workspace — 워크스페이스

폴더: `public/docs/workspace/`

| 페이지 | # | 파일명 | 캡처 대상 |
|---|---|---|---|
| Create | - [ ] 01 | `create-01-select.png` | 워크스페이스 select dropdown |
| Create | - [ ] 02 | `create-02-form.png` | 생성 폼 다이얼로그 |
| Create | - [ ] 03 | `create-03-edit-delete.png` | edit / delete 버튼 |
| Sensors | - [ ] 01 | `sensors-01-expansion.png` | Sensor Setting expansion 펼침 |
| Sensors | - [ ] 02 | `sensors-02-add.png` | Add Sensor 폼 |
| Sensors | - [ ] 03 | `sensors-03-common-resolution.png` | Common Width/Height 입력 |
| Sensor Config | - [ ] 01 | `sensor-cfg-01-crop-canvas.png` | crop 캔버스 + 마우스 드래그 |
| Sensor Config | - [ ] 02 | `sensor-cfg-02-coords.png` | x1, y1, x2, y2 입력 |
| Sensor Config | - [ ] 03 | `sensor-cfg-03-rotation.png` | Rotation 0/90/180/270 select |
| Sensor Config | - [ ] 04 | `sensor-cfg-04-reset.png` | Reset crop 버튼 |
| Robots | - [ ] 01 | `robots-01-assembly-select.png` | Robot Assembly select |
| Robots | - [ ] 02 | `robots-02-list.png` | Assembly 내부 로봇 리스트 |
| Home Pose | - [ ] 01 | `home-01-section.png` | Home Pose 섹션 전체 |
| Home Pose | - [ ] 02 | `home-02-sync.png` | sync 버튼 클릭 직전/직후 |
| Home Pose | - [ ] 03 | `home-03-save.png` | save 버튼 |
| Episode | - [ ] 01 | `episode-01-task-setting.png` | Task Setting — Episode Length |
| Dataset | - [ ] 01 | `dataset-01-add.png` | Add Dataset Folder 버튼 |
| Dataset | - [ ] 02 | `dataset-02-form.png` | Dataset 생성 폼 |
| Dataset | - [ ] 03 | `dataset-03-list.png` | 데이터셋 목록 expansion |
| Dataset | - [ ] 04 | `dataset-04-incompatible.png` | metadata remap 워닝 |

---

## 8. data-collection — 데이터 수집

폴더: `public/docs/data-collection/`

| 페이지 | # | 파일명 | 캡처 대상 |
|---|---|---|---|
| Teleop Mode | - [ ] 01 | `mode-01-select.png` | Teleop Type select 펼침 (keyboard/leader_robot/vive/...) |
| Teleop Mode | - [ ] 02 | `mode-02-keyboard.png` | keyboard 모드 — Step Size 입력 추가 |
| Teleop Mode | - [ ] 03 | `mode-03-leader.png` | leader_robot 모드 |
| Home Return | - [ ] 01 | `home-01-toggle-on.png` | REC 버튼 옆 home 배지 — blue (ON) |
| Home Return | - [ ] 02 | `home-02-toggle-off.png` | home 배지 — grey (OFF) |
| Record Flow | - [ ] 01 | `record-01-rec.png` | REC 버튼 hover |
| Record Flow | - [ ] 02 | `record-02-moving.png` | "Moving to Home Pose" 화면 |
| Record Flow | - [ ] 03 | `record-03-recording.png` | 수집 중 progress bar |
| Record Flow | - [ ] 04 | `record-04-progress.png` | progress 100% 직전 |
| Buttons | - [ ] 01 | `btn-01-success.png` | SUCCESS (C) 버튼 |
| Buttons | - [ ] 02 | `btn-02-done.png` | DONE 버튼 |
| Buttons | - [ ] 03 | `btn-03-stop.png` | STOP 버튼 |

---

## 9. datasets — 데이터셋 관리

폴더: `public/docs/datasets/`

| 페이지 | # | 파일명 | 캡처 대상 |
|---|---|---|---|
| Replay | - [ ] 01 | `replay-01-select.png` | data 탭 — 에피소드 선택 |
| Replay | - [ ] 02 | `replay-02-header.png` | replay 헤더 (action type / capture / Hz / Play) |
| Replay | - [ ] 03 | `replay-03-action-type.png` | qaction vs ee_delta_action 라디오 |
| Replay | - [ ] 04 | `replay-04-capture.png` | Capture Episode 체크박스 + Target Dataset |
| Replay | - [ ] 05 | `replay-05-progress.png` | Capturing X% progress |
| Augment | - [ ] 01 | `augment-01-context-menu.png` | 데이터셋 우클릭 — Augment Dataset 메뉴 |
| Augment | - [ ] 02 | `augment-02-dialog.png` | DataAugmentationDialog 전체 |
| Edit | - [ ] 01 | `edit-01-edit.png` | Edit Dataset 폼 |
| Edit | - [ ] 02 | `edit-02-merge.png` | Merge Dataset 폼 |
| Edit | - [ ] 03 | `edit-03-remap.png` | metadata remap (sensor/robot select) |
| Edit | - [ ] 04 | `edit-04-delete.png` | Delete 확인 |

---

## 10. training — 학습

폴더: `public/docs/training/`

| 페이지 | # | 파일명 | 캡처 대상 |
|---|---|---|---|
| Server | - [ ] 01 | `server-01-launcher-btn.png` | 런처 pill bar 🔥 학습 서버 hover |
| Server | - [ ] 02 | `server-02-dialog.png` | 학습 서버 관리 다이얼로그 (전체) |
| Server | - [ ] 03 | `server-03-install.png` | "설치" 클릭 → 다운로드 진행 로그 |
| Server | - [ ] 04 | `server-04-running.png` | "🟢 실행 중" 상태 + 로그 |
| Server | - [ ] 05 | `server-05-frontend-url.png` | TrainPage step3 — Training Server URL 입력 |
| Server | - [ ] 06 | `server-06-health-ok.png` | URL 옆 check_circle (positive) + GPU Available 배지 |
| Dataset Select | - [ ] 01 | `dataset-01-grid.png` | step1 데이터셋 카드 그리드 |
| Dataset Select | - [ ] 02 | `dataset-02-selected.png` | 선택된 카드 (border primary) |
| Policy | - [ ] 01 | `policy-01-select.png` | step2 Policy select 펼침 |
| Policy | - [ ] 02 | `policy-02-create-new.png` | "Create New Policy +" 클릭 후 폼 |
| Policy | - [ ] 03 | `policy-03-act-params.png` | ACT 파라미터 폼 전체 |
| Policy | - [ ] 04 | `policy-04-pretrained.png` | pretrained_backbone_weights select |
| Policy | - [ ] 05 | `policy-05-save-as.png` | "Save Policy As" 다이얼로그 |
| Parameters | - [ ] 01 | `params-01-checkpoint-name.png` | Checkpoint Name 입력 |
| Parameters | - [ ] 02 | `params-02-form.png` | training 폼 (lr, batch_size, epochs 등) |
| Start & Queue | - [ ] 01 | `queue-01-start.png` | "Start Training" 버튼 |
| Start & Queue | - [ ] 02 | `queue-02-buttons.png` | 우하단 Train Queue floating buttons |
| Start & Queue | - [ ] 03 | `queue-03-dialog.png` | TrainingDialog 열린 모습 |
| Start & Queue | - [ ] 04 | `queue-04-loss-chart.png` | loss chart zoom (Train/Validate) |

---

## 11. inference — 추론

폴더: `public/docs/inference/`

| 페이지 | # | 파일명 | 캡처 대상 |
|---|---|---|---|
| Select | - [ ] 01 | `select-01-grid.png` | inference 탭 체크포인트 카드 그리드 |
| Select | - [ ] 02 | `select-02-context.png` | 우클릭 메뉴 (Show Details / Edit / OOD / Export / Delete) |
| Select | - [ ] 03 | `select-03-info-dialog.png` | Show Details — CheckpointInfo 다이얼로그 |
| Run | - [ ] 01 | `run-01-header.png` | MonitoringWindow inference 헤더 (Hz + home toggle + Start) |
| Run | - [ ] 02 | `run-02-running.png` | 추론 중 화면 |
| Run | - [ ] 03 | `run-03-succeed-ood.png` | Succeed / OOD 배지 zoom |
| Run | - [ ] 04 | `run-04-stop.png` | Stop 버튼 |

---

## 12. planner — 플래너

폴더: `public/docs/planner/`

| 페이지 | # | 파일명 | 캡처 대상 |
|---|---|---|---|
| Create | - [ ] 01 | `create-01-select.png` | 상단 플래너 선택 드롭다운 펼친 상태 |
| Create | - [ ] 02 | `create-02-new.png` | 드롭다운 안 "+ 새 플래너 만들기" 항목 |
| Create | - [ ] 03 | `create-03-form.png` | 새 플래너 만들기 다이얼로그 |
| Create | - [ ] 04 | `create-04-edit-delete.png` | 플래너 항목 우측 edit/delete 아이콘 |
| Workspaces | - [ ] 01 | `workspaces-01-add-button.png` | 좌측 패널 "워크스페이스 추가" 버튼 |
| Workspaces | - [ ] 02 | `workspaces-02-multiselect.png` | 워크스페이스 선택 다이얼로그 (다중 선택) |
| Workspaces | - [ ] 03 | `workspaces-03-expanded.png` | 워크스페이스 expansion 펼침 — 센서/로봇 토글 |
| Workspaces | - [ ] 04 | `workspaces-04-remove.png` | 워크스페이스 헤더 X 버튼 |
| Blocks | - [ ] 01 | `blocks-01-new-button.png` | 가운데 패널 "새 블록" 버튼 |
| Blocks | - [ ] 02 | `blocks-02-type-select.png` | 블록 종류 select 펼침 (joint_position / checkpoint / timesleep) |
| Blocks | - [ ] 03 | `blocks-03-name.png` | 블록 이름 입력 필드 |
| Blocks | - [ ] 04 | `blocks-04-save.png` | 블록 다이얼로그 저장 버튼 |
| Blocks Joint | - [ ] 01 | `blocks-joint-01-workspace.png` | Joint Position — 워크스페이스 select |
| Blocks Joint | - [ ] 02 | `blocks-joint-02-pendant.png` | 다이얼로그 안 RobotPendant |
| Blocks Joint | - [ ] 03 | `blocks-joint-03-apply.png` | 저장된 자세 카드 + 현재 자세 적용 / 이동 버튼 |
| Blocks Ckpt | - [ ] 01 | `blocks-checkpoint-01-select.png` | Checkpoint — 워크스페이스/체크포인트 select |
| Blocks Ckpt | - [ ] 02 | `blocks-checkpoint-02-params.png` | Checkpoint 파라미터 (시간/Hz/re_inference_steps) |
| Blocks Sleep | - [ ] 01 | `blocks-sleep-01-duration.png` | Time Sleep 블록 시간 입력 |
| Blocks List | - [ ] 01 | `blocks-list-01-row.png` | 블록 행 + 우클릭 컨텍스트 메뉴 |
| Run | - [ ] 01 | `run-01-start.png` | 가운데 패널 "전체 실행" 버튼 |
| Run | - [ ] 02 | `run-02-running.png` | 실행 중 — 행 강조(파란 테두리) + 스피너 + 상태 텍스트 |
| Run | - [ ] 03 | `run-03-monitor.png` | 우측 모니터링 윈도우 — 실행 중 |
| Run | - [ ] 04 | `run-04-results.png` | 행 우측 결과 아이콘 (check_circle / cancel / error) |
| Run | - [ ] 05 | `run-05-stop.png` | "중지" 버튼 |

---

## 진행 요약

총 **약 186장**의 스크린샷이 필요합니다. 빠른 시작(섹션 2)이 44장으로 가장 큰 비중이며, 나머지는 reference에 가깝습니다.

캡처 우선순위 권장:
1. **installation (11장)** — 첫 사용자가 가장 먼저 보는 섹션
2. **quickstart (44장)** — 가장 가치 높은 가이드
3. **modules / training / inference** — 자주 참조됨
4. 나머지 reference 섹션은 점진적으로
