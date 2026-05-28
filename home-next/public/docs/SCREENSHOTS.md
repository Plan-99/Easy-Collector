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
| - [x] 01 | `01-deb-download.png` | 다운로드 페이지 또는 GitHub Releases — easytrainer_*.deb 항목 |
| - [x] 02 | `02-gui-install.png` | 파일 매니저에서 deb 더블클릭 후 뜨는 Ubuntu Software Install 다이얼로그 (Install 버튼 보이게) |
| - [x] 03 | `03-launcher-app-menu.png` | Activities/시작 메뉴에서 "Easy Trainer" 검색 결과 + 아이콘 |
| - [x] 04 | `04-login-dialog.png` | PyQt "Easy Trainer 로그인" 다이얼로그 (560×260) |
| - [x] 05 | `05-browser-oauth.png` | 브라우저 `/auth/device?code=...` Google OAuth 동의 화면 |
| - [x] 06 | `06-installer-eula.png` | 설치 마법사 page 0 — EULA + 동의함 체크박스 |
| - [x] 07 | `07-installer-disk.png` | page 1 — 저장 공간 확인 (현재 여유 공간 X.X GB) |
| - [x] 08 | `08-installer-mode.png` | page 2 — 학습 서버 옵션 라디오 (원격 기본값 / 로컬) |
| - [x] 09 | `09-installer-progress.png` | page 3 — 설치 중 progress bar + 로그 |
| - [x] 10 | `10-installer-done.png` | page 4 — 설치 완료 + "Easy Trainer 실행하기" 체크박스 |
| - [x] 11 | `11-launcher-main.png` | 메인 런처 floating pill bar 전체 (상태등 포함) |

---

## 2. quickstart — 빠른 시작

폴더: `public/docs/quickstart/`

### 튜토리얼 모드
| # | 파일명 | 캡처 대상 |
|---|---|---|
| - [x] 01 | `01-tutorial-toggle.png` | MainLayout 사이드바 Tutorial 토글 |
| - [x] 03 | `03-pipeline-guide.png` | PipelineGuideDialog — 6단계 stepper 펼친 상태 |

### 센서 확인
| # | 파일명 | 캡처 대상 |
|---|---|---|
| - [x] 04 | `04-sensors-overview.png` | SensorPage 전체 (sim 카메라 카드들) |
| - [x] 05 | `05-sensor-card.png` | 단일 센서 카드 zoom (ONLINE 상태 + 토글) |
| - [x] 06 | `06-sensor-preview.png` | 하단 WebRTC 라이브 프리뷰 |

### 로봇 움직임
| # | 파일명 | 캡처 대상 |
|---|---|---|
| - [x] 07 | `07-robots-overview.png` | RobotPage 전체 |
| - [x] 08 | `08-robot-online.png` | 로봇 카드 ONLINE 상태 |
| - [x] 09 | `09-robot-pendant-joint.png` | RobotPendant Joint 제어 영역 |
| - [x] 10 | `10-robot-pendant-ik.png` | RobotPendant IK 제어 (xyz/rpy) zoom |

### Assemble
| # | 파일명 | 캡처 대상 |
|---|---|---|
| - [x] 11 | `11-assemble-new-tab.png` | AssemblePage "New" 탭 진입 직후 |
| - [x] 12 | `12-assemble-form.png` | AssemblyForm 우측 로봇 선택 + Left/Right 버튼 |
| - [x] 13 | `13-assemble-canvas.png` | 좌측 Canvas — 그려진 조립 결과 |
| - [x] 14 | `14-assemble-name.png` | Assembly 이름 입력 (Save 직전 화면) |
| - [x] 15 | `15-assemble-card.png` | 그리드에 새로 생성된 Assembly 카드 |

### Teleoperation 키보드 테스트
| # | 파일명 | 캡처 대상 |
|---|---|---|
| - [x] 16 | `16-teleop-overview.png` | TeleoperationPage — 카드 mode indicator (키보드 ✅, 게임패드 ❌) |
| - [x] 17 | `17-teleop-setting-open.png` | 카드 ⚙️ 버튼 클릭 → TeleSettingDialog 열림 |
| - [x] 19 | `19-teleop-keyboard.png` | Keyboard 탭 — step size + 키 매핑 표 |
| - [x] 20 | `20-teleop-running.png` | TeleopConsole — Start Keyboard Teleop 활성 + 로그 |

### Workspace
| # | 파일명 | 캡처 대상 |
|---|---|---|
| - [x] 21 | `21-workspace-create.png` | "Create New Workspace +" select dropdown 펼친 상태 |
| - [x] 22 | `22-workspace-form.png` | Workspace 생성 폼 다이얼로그 |
| - [x] 23 | `23-workspace-sensor.png` | setting 탭 — Sensor Setting expansion (센서 추가 직후) |
| - [x] 24 | `24-workspace-sensor-crop.png` | sensor config 패널 — crop 캔버스 + Cropped Area 입력 |
| - [x] 25 | `25-workspace-robot.png` | Robot Setting expansion + Assembly select |
| - [x] 26 | `26-workspace-homepose.png` | robot config — Home Pose 입력 + sync/save 버튼 |
| - [x] 27 | `27-workspace-task.png` | Task Setting — Episode Length 입력 |
| - [x] 28 | `28-workspace-dataset-add.png` | data 탭 — Add Dataset Folder 폼 |

### 데이터 수집 (10 에피소드)
| # | 파일명 | 캡처 대상 |
|---|---|---|
| - [x] 29 | `29-monitoring-overview.png` | MonitoringWindow 전체 (sensor feed + robot state) |
| - [x] 30 | `30-record-setup.png` | 수집 setup row (Dataset/Instruction/Teleop type/Hz/REC) |
| - [x] 31 | `31-moving-home.png` | "Moving to Home Pose" 스피너 |
| - [x] 33 | `33-dataset-10-episodes.png` | data 탭 expansion — 10 에피소드 쌓인 모습 |

### 학습 (기본 ACT)
| # | 파일명 | 캡처 대상 |
|---|---|---|
| - [x] 35 | `35-train-step1.png` | TrainPage stepper 1단계 진입 |
| - [x] 36 | `36-train-dataset-select.png` | step1 — 데이터셋 카드 선택 + Continue |
| - [x] 37 | `37-train-step2-newpolicy.png` | step2 — "Create New Policy +" 선택 |
| - [x] 38 | `38-train-act-form.png` | Policy 타입 = ACT 파라미터 폼 |
| - [x] 40 | `40-train-server-ok.png` | Training Server URL 입력 + 헬스체크 OK |
| - [x] 41 | `41-training-running.png` | TrainingDialog — progress bar + loss chart + ProcessConsole |
| - [x] 42 | `42-train-done.png` | 학습 완료 — 우하단 floating button |

### 추론
| # | 파일명 | 캡처 대상 |
|---|---|---|
| - [x] 43 | `43-inference-list.png` | Workspace inference 탭 — 체크포인트 카드 |
| - [x] 44 | `44-inference-setup.png` | MonitoringWindow inference 헤더 — Hz + Start Inference |
| - [x] 45 | `45-inference-running.png` | 추론 중 — Succeed 배지 |

---

## 3. modules — 모듈 추가 (런처)

폴더: `public/docs/modules/`

| # | 파일명 | 캡처 대상 |
|---|---|---|
| - [x] 01 | `01-launcher-modules-button.png` | 런처 pill bar 🧩 모듈 관리 hover |
| - [x] 02 | `02-modules-dialog.png` | 모듈 관리 다이얼로그 — 3 카테고리 (로봇/센서/확장) |
| - [x] 03 | `03-module-installing.png` | 설치 진행 중 ⏳ |
| - [x] 04 | `04-module-installed.png` | 설치 완료 (v버전 + 초록색) |
| - [x] 05 | `05-module-paid.png` | 유료 모듈 — "결제" 버튼 + ₩가격 |
| - [x] 06 | `06-module-checkout.png` | "결제 진행" 다이얼로그 (URL + 복사) |
| - [x] 07 | `07-module-after-pay.png` | 결제 완료 후 자동 설치 |
| - [x] 08 | `08-module-removed.png` | "제거" 클릭 후 미설치 상태 |
| - [x] 09 | `09-launcher-modules-local-button.png` | 모듈 관리 다이얼로그 하단 — ➕ 로컬 모듈 만들기 버튼 |
| - [x] 10 | `10-wizard-open.png` | ModuleWizard 첫 화면 — 모듈 ID/이름/카테고리/버전 입력 |
| - [x] 11 | `11-wizard-variant.png` | ModuleWizard variant — ROS 패키지/SDK 폴더, joint, URDF, IK |
| - [x] 12 | `12-wizard-launch.png` | ModuleWizard — launch args / pre-launch / post-launch |
| - [x] 13 | `13-wizard-validate.png` | ModuleWizard — 검증 결과 (schema + controller.py 점검) |
| - [x] 14 | `14-wizard-install.png` | ModuleWizard — 설치 진행 로그 ([1/3] 패키징 → [2/3] 트리 설치 → [3/3] colcon build) |
| - [x] 15 | `15-wizard-installed.png` | 설치 완료 후 모듈 관리 목록에 새 로컬 모듈 등장 |
| - [ ] 16 | `16-module-credentials.png` | post_install.credentials 다이얼로그 — 안내 링크 + 시크릿 입력(예: SAM 3 HuggingFace 토큰) |

---

## 4. sensors — 센서 추가

폴더: `public/docs/sensors/`

| 페이지 | 파일명 | 캡처 대상 |
|---|---|---|
| 공통 (모든 타입에서 공유) | `add-sensor-button.png` | SensorPage — 우측 상단/카드 끝의 Add Sensor 버튼 영역 |

| 페이지 | # | 파일명 | 캡처 대상 |
|---|---|---|---|
| Realsense | - [x] 01 | `realsense-01-form.png` | SensorForm — Type=realsense, Serial Number 입력 |
| Realsense | - [x] 02 | `realsense-02-online.png` | Realsense 카드 ONLINE + 라이브 프리뷰 |
| Webcam | - [x] 01 | `webcam-01-form.png` | SensorForm — Type=webcam, Device Index |
| Webcam | - [x] 02 | `webcam-02-preview.png` | Webcam 라이브 프리뷰 |
| Custom | - [x] 01 | `custom-01-form.png` | SensorForm — Type=custom, Read Topic, Msg Type, Resolution |
| Custom | - [x] 02 | `custom-02-topic-on.png` | Custom 센서 TOPIC ON 상태 |

---

## 5. robots — 로봇 추가

폴더: `public/docs/robots/`

| 페이지 | 파일명 | 캡처 대상 |
|---|---|---|
| 공통 (모든 타입에서 공유) | `add-robot-button.png` | RobotPage — Add Robot 버튼 영역 |
| 공통 (모든 타입에서 공유) | `robot-form.png` | RobotForm — Robot Type 선택 + 연결 정보 입력 다이얼로그 |
| 공통 (모든 타입에서 공유) | `robot-online.png` | 로봇 카드 — 토글 ON, 상태 ONLINE |
| 공통 (모든 타입에서 공유) | `add-pose-button.png` | RobotPendant — ADD POSE 버튼 + 저장된 pose 버튼들 |

vendor별 페이지는 **하드웨어 케이블 / 컨트롤박스 / vendor 티치펜던트 화면 위주**로만 캡처합니다. EasyTrainer의 RobotPendant 패널·Add Robot 폼·ONLINE 카드 같은 공통 흐름은 위 공통 슬롯에서 처리합니다.

| 페이지 | # | 파일명 | 캡처 대상 |
|---|---|---|---|
| Piper | - [x] 01 | `piper-01-can-setup.png` | USB-CAN 어댑터 + Piper CAN 포트 연결 (+ 호스트 `ip link show can0`) |
| Fairino | - [x] 01 | `fairino-01-network.png` | Fairino 티치펜던트 — 네트워크 / IP 확인 화면 (필요 시 Remote 모드 토글 포함) |
| OMX | - [ ] 01 | `omx-01-hardware.png` | OMX 본체 + U2D2 보드 + 12 V 어댑터 + 호스트 `/dev/ttyACM*` 확인 |
| OMX | - [ ] 02 | `omx-02-online.png` | OMX 카드 ONLINE + RobotPendant 조인트 슬라이더 (5-DOF + gripper) |
| Custom | (스크린샷 없음 — 텍스트 가이드만) | — | 폼 화면은 공통 슬롯으로 충분, custom 페이지는 텍스트 단계만 |
| Custom | - [x] 01 | `custom-01-form.png` | RobotForm — Role / Joint Names / Bounds / Read·Write 토픽 / IK Settings JSON 한 화면 |
| Custom | - [ ] 02 | `custom-02-online.png` | Custom 로봇 TOPIC ON 상태 |

---

## 6. teleoperation — 텔레오퍼레이션

폴더: `public/docs/teleoperation/`

| 페이지 | # | 파일명 | 캡처 대상 |
|---|---|---|---|
| Assembly | - [x] 01 | `assembly-01-form.png` | AssemblyForm — 좌측 Canvas + 우측 로봇 카드 패널 |
| Assembly | - [ ] 02 | `assembly-02-role-buttons.png` | single_arm / tool 역할별 Left·Right 슬롯 버튼 |
| End-effector | - [ ] 01 | `ee-01-general-tab.png` | TeleSettingDialog General 탭 |
| End-effector | - [ ] 02 | `ee-02-offset-explained.png` | EE offset x/y/z 입력 + 설명 텍스트 |
| Keyboard | - [x] 01 | `keyboard-01-tab.png` | Keyboard 탭 — Step size 입력 + Left/Right arm 키 매핑 표 |
| Leader | - [x] 01 | `leader-01-tab.png` | Leader Robot 탭 — 좌측 Start Leader Robot 버튼 + 우측 Stepper |
| Leader | - [x] 02 | `leader-02-origin-mapping.png` | Origin Setting — DXL 카드 드래그 + Set as Gripper 버튼 |
| Testing | - [x] 01 | `testing-01-console.png` | TeleopConsole 초기 상태 |
| Testing | - [x] 02 | `testing-02-running.png` | Start Keyboard Teleop 활성 — 좌측 키보드 로그 + 우측 리더 로그 |

---

## 7. workspace — 워크스페이스

폴더: `public/docs/workspace/`

| 페이지 | # | 파일명 | 캡처 대상 |
|---|---|---|---|
| Create | - [x] 01 | `create-01-select.png` | 워크스페이스 select dropdown |
| Create | - [x] 02 | `create-02-form.png` | 생성 폼 다이얼로그 |
| Create | - [x] 03 | `create-03-edit-delete.png` | edit / delete 버튼 |
| Sensors | - [x] 01 | `sensors-01-expansion.png` | Sensor Setting expansion — Common Width/Height + Add Sensor 버튼 + 선택 폼 |
| Sensor Config | - [x] 01 | `sensor-cfg-01-crop.png` | Crop 패널 — 캔버스 드래그 + x1/y1/x2/y2 입력 + Reset crop + Rotation select |
| Robots | - [x] 01 | `robots-01-assembly-select.png` | Robot Assembly select |
| Robots | - [x] 02 | `robots-02-list.png` | Assembly 내부 로봇 리스트 |
| Home Pose | - [x] 01 | `home-01-section.png` | Home Pose 섹션 — 관절값 입력 칸 + sync / save 버튼 |
| Episode | - [x] 01 | `episode-01-task-setting.png` | Task Setting — Episode Length |
| Dataset | - [x] 01 | `dataset-01-add.png` | Add Dataset Folder 버튼 |
| Dataset | - [x] 02 | `dataset-02-form.png` | Dataset 생성 폼 |
| Dataset | - [x] 03 | `dataset-03-list.png` | 데이터셋 목록 expansion |
| Dataset | - [ ] 04 | `dataset-04-incompatible.png` | metadata remap 워닝 |

---

## 8. data-collection — 데이터 수집

폴더: `public/docs/data-collection/`

| 페이지 | # | 파일명 | 캡처 대상 |
|---|---|---|---|
| Teleop Mode | - [ ] 01 | `mode-01-select.png` | Teleop Type select 펼침 (keyboard/leader_robot/vive/...) |
| Teleop Mode | - [ ] 02 | `mode-02-keyboard.png` | keyboard 모드 — Step Size 입력 추가 |
| Teleop Mode | - [ ] 03 | `mode-03-leader.png` | leader_robot 모드 |
| Home Return | - [x] 01 | `home-01-toggle.png` | home 배지 — 파란색(ON) / 회색(OFF) 두 상태 |
| Record Flow | - [x] 01 | `record-01-rec.png` | REC 버튼 hover |
| Record Flow | - [x] 02 | `record-02-moving.png` | Moving to Home Pose → 수집 중 progress bar + SUCCESS/DONE/STOP → 100% 직전 |
| Buttons | - [x] 01 | `btn-01-toolbar.png` | MonitoringWindow 툴바 — SUCCESS / DONE / STOP 3개 버튼 |
| Replay | - [ ] 01 | `replay-01-select.png` | 워크스페이스 data 탭 — 에피소드 선택 |
| Replay | - [ ] 02 | `replay-02-running.png` | **통합 컷** (replay-panel + progress) — Replay 헤더(Action Type / Capture Episode / Target Dataset)와 Capturing X% progress가 동시에 보이는 상태 한 컷 |

---

## 9. datasets — 데이터셋 관리

폴더: `public/docs/datasets/`

| 페이지 | # | 파일명 | 캡처 대상 |
|---|---|---|---|
| Common | - [x] 00 | `context-menu.png` | **통합 컷** (edit-00 + export-01 + export-02 대체) — 데이터셋 우클릭 메뉴 전체 (Edit / Batch Edit / Merge / Export / Delete 항목이 모두 보이는 한 컷) — 예전 Augment / 다운샘플 항목이 Batch Edit 하나로 합쳐진 모습 |
| Edit | - [x] 01 | `edit-01-edit.png` | Edit Dataset 폼 |
| Edit | - [x] 02 | `edit-02-merge.png` | Merge Dataset 폼 |
| Edit | - [x] 03 | `edit-03-remap.png` | metadata remap (sensor/robot select) |
| Edit | - [x] 04 | `edit-04-delete.png` | Delete 확인 |
| Batch Edit | - [x] 01 | `batch-edit-01-panel.png` | BatchEditPanel 전체 — 데이터셋 폴더 라벨 클릭 후 우측 패널 + 출력 이름 입력 + Augmentation/Downsample/Crop/Rotate 4개 섹션 헤더 + 하단 Apply 버튼이 한 화면에 보이는 컷 |
| Batch Edit | - [x] 02 | `batch-edit-02-augmentation.png` | Augmentation 섹션 펼친 모습 — Repeat 입력 + Lightness / HSV / Disturbances / Salt & Pepper / Gaussian / Perspective 슬라이더 |
| Batch Edit | - [x] 03 | `batch-edit-03-crop.png` | Crop 섹션 — 카메라별 첫 프레임 + 드래그한 파란 박스 + Reset 버튼 (가능하면 카메라 2대 이상이 보이도록) |
| Batch Edit | - [x] 04 | `batch-edit-04-rotate.png` | Rotate 섹션 — 카메라별 0°/90°/180°/270° q-btn-toggle 행 |
| Batch Edit | - [x] 05 | `batch-edit-05-progress.png` | Apply 후 하단 Apply 영역이 progress bar로 바뀐 모습 (진행률 %  badge 노출) |
| Episode Edit | - [x] 01 | `episode-edit-01-panel.png` | **통합 컷** (episode-edit-03-layout 흡수) — 데이터셋 탭에서 에피소드 클릭 시 우측 EpisodePanel 전체 + 섹션 카드 분리 + 좌우 패널 높이 동기화 + 카메라 수별 이미지 크기 |
| Episode Edit | - [x] 02 | `episode-edit-02-trim.png` | Trim 슬라이더 + Apply 버튼 |
| Import / Export | - [ ] 03 | `export-03-save-dialog.png` | OS 네이티브 "다른 이름으로 저장" 다이얼로그 — 제안된 파일명 `<name>_<id>.tar.gz` 노출 |
| Import / Export | - [x] 04 | `import-01-button.png` | 데이터셋 탭(또는 WorkspacePage data 탭) 상단의 "데이터셋 불러오기" 버튼 |
| Import / Export | - [ ] 05 | `import-02-file-picker.png` | OS 네이티브 파일 선택 다이얼로그 — `.tar.gz` 아카이브 선택 |
| Import / Export | - [x] 06 | `import-03-listed.png` | 가져온 데이터셋이 데이터셋 목록에 새 행으로 추가된 상태 |

> 이전 슬롯 중 `augment-02-dialog.png`(DataAugmentationDialog 전체) 와 `downsample-01-dialog.png`(데이터셋 다운샘플 다이얼로그) 는 BatchEditPanel 통합으로 제거되었습니다. 해당 UI는 위 `batch-edit-02-augmentation.png` / Downsample 섹션 컷으로 대체됩니다.

---

## 10. training — 학습

폴더: `public/docs/training/`

| 페이지 | # | 파일명 | 캡처 대상 |
|---|---|---|---|
| Server | - [ ] 01 | `server-01-launcher-btn.png` | 런처 pill bar 🔥 학습 서버 hover |
| Server | - [ ] 02 | `server-02-dialog.png` | **통합 컷** (server-03-install-run 흡수) — 학습 서버 관리 다이얼로그 전체 + 설치 / 실행 버튼 + 진행 로그 + 실행 중 상태가 모두 보이는 한 컷 |
| Server | - [x] 04 | `server-04-frontend.png` | TrainPage step3 — Training Server URL 입력 + 헬스체크 OK + GPU Available 배지 |
| Dataset Select | - [x] 01 | `dataset-01-grid.png` | step1 데이터셋 카드 그리드 |
| Dataset Select | - [x] 02 | `dataset-02-selected.png` | 선택된 카드 (border primary) |
| Policy | - [ ] 01 | `policy-01-select.png` | step2 Policy select 펼침 |
| Policy | - [x] 02 | `policy-02-create-new.png` | "Create New Policy +" 클릭 후 폼 |
| Policy | - [x] 03 | `policy-03-params.png` | 정책 파라미터 폼 — 타입별 하이퍼파라미터 + pretrained_backbone_weights |
| Policy | - [ ] 04 | `policy-04-save-as.png` | "Save Policy As" 다이얼로그 |
| Parameters | - [ ] 01 | `params-01-form.png` | **통합 컷** (queue-01-start 흡수) — Step 3 Checkpoint Name + lr / batch_size / epochs 등 학습 폼 + "Start Training" 버튼이 한 화면에 보이는 컷 |
| Parameters | - [ ] 02 | `params-02-help-dialog.png` | 하이퍼파라미터 도움말 다이얼로그 — ? 버튼 클릭 시 쉬운 설명 / 자세한 설명 분리 |
| Start & Queue | - [x] 02 | `queue-02-buttons.png` | 우하단 Train Queue floating buttons |
| Start & Queue | - [x] 03 | `queue-03-dialog.png` | TrainingDialog 열린 모습 |
| Start & Queue | - [ ] 04 | `queue-04-loss-chart.png` | loss chart zoom (Train/Validate) |

---

## 11. inference — 추론

폴더: `public/docs/inference/`

| 페이지 | # | 파일명 | 캡처 대상 |
|---|---|---|---|
| Select | - [x] 01 | `select-01-grid.png` | inference 탭 체크포인트 카드 그리드 |
| Select | - [ ] 02 | `select-02-context.png` | 우클릭 메뉴 (Show Details / Edit / Export / Delete) |
| Select | - [ ] 03 | `select-03-info-dialog.png` | Show Details — CheckpointInfo 다이얼로그 |
| Run | - [x] 01 | `run-01-monitoring.png` | MonitoringWindow inference — Hz / Home 배지 / Start·Stop 버튼 + 추론 중 카메라 뷰 |
| Vision Map | - [ ] 01 | `vision-map-01-settings.png` | EpisodeViewer Vision Map 설정 다이얼로그 — Checkpoint select + Method(Attention / Grad-CAM) |
| Vision Map | - [ ] 02 | `vision-map-02-precompute.png` | EpisodeViewer 비전 맵 준비 중 — 카메라 영상 위 스피너 + `done / total` 진행 표시 |
| Vision Map | - [ ] 03 | `vision-map-03-overlay.png` | EpisodeViewer — 카메라 영상 위에 컬러 히트맵 오버레이가 적용된 모습 |
| Vision Map | - [ ] 04 | `vision-map-04-inference-dropdown.png` | 추론 중 MonitoringWindow — Vision Map 드롭다운 펼친 상태 (끔 / Attention / Grad-CAM) |
| Vision Map | - [ ] 05 | `vision-map-05-inference-overlay.png` | 추론 중 카메라 영상 위 실시간 히트맵 오버레이 |
| DAgger | - [ ] 01 | `dagger-01-button.png` | 추론 중 MonitoringWindow 상단 — 노란 DAgger 버튼 (school 아이콘) |
| DAgger | - [ ] 02 | `dagger-02-dialog.png` | DAgger 데이터 수집 다이얼로그 — Dataset / Language Instruction / Teleop Type / Hz |

---

## 12. planner — 플래너

폴더: `public/docs/planner/`

| 페이지 | # | 파일명 | 캡처 대상 |
|---|---|---|---|
| Create+Workspaces | - [x] 01 | `create-01-select.png` | 상단 플래너 선택 드롭다운 펼친 상태 |
| Create+Workspaces | - [x] 02 | `create-02-new.png` | 새 플래너 만들기 다이얼로그 (영문 캡처를 한·영 매뉴얼 공용으로 사용) |
| Create+Workspaces | - [x] 03 | `create-04-edit-delete.png` | 플래너 항목 우측 edit/delete 아이콘 |
| Create+Workspaces | - [x] 04 | `workspaces-01-add-button.png` | 좌측 패널 "워크스페이스 추가" 버튼 |
| Create+Workspaces | - [x] 05 | `workspaces-02-multiselect.png` | 워크스페이스 선택 다이얼로그 (다중 선택) |
| Create+Workspaces | - [x] 06 | `workspaces-03-expanded.png` | 워크스페이스 expansion 펼침 — 센서/로봇 토글 |
| Create+Workspaces | - [x] 07 | `workspaces-04-remove.png` | 워크스페이스 헤더 X 버튼 |
| Blocks+Run | - [x] 01 | `blocks-01-new-button.png` | 가운데 패널 "새 블록" 버튼 |
| Blocks+Run | - [x] 02 | `blocks-02-type-select.png` | 블록 종류 select 펼침 (joint_position / checkpoint / timesleep) |
| Blocks+Run | - [x] 03 | `blocks-03-name.png` | 블록 이름 입력 필드 |
| Blocks+Run | - [x] 04 | `blocks-04-save.png` | 블록 다이얼로그 저장 버튼 |
| Blocks+Run | - [x] 05 | `blocks-joint-01-workspace.png` | Joint Position — 워크스페이스 select |
| Blocks+Run | - [x] 06 | `blocks-joint-02-pendant.png` | 다이얼로그 안 RobotPendant |
| Blocks+Run | - [x] 07 | `blocks-joint-03-apply.png` | 저장된 자세 카드 + 현재 자세 적용 / 이동 버튼 |
| Blocks+Run | - [x] 08 | `blocks-checkpoint.png` | Checkpoint 블록 — 워크스페이스/체크포인트 select + 파라미터(시간/Hz/re_inference_steps) 통합 |
| Blocks+Run | - [x] 09 | `blocks-sleep-01-duration.png` | Time Sleep 블록 시간 입력 |
| Blocks+Run | - [x] 10 | `blocks-list-01-row.png` | 블록 행 + 우클릭 컨텍스트 메뉴 |
| Blocks+Run | - [x] 11 | `run.png` | 플랜 실행 중 — 좌측 진행 상태(전체 실행 / 실행 중 강조 / 결과 아이콘 / 중지 버튼) + 우측 모니터링 윈도우 통합 |
| Export | - [x] 01 | `export-01-button.png` | 플래너 헤더 — 내보내기 버튼 (다운로드 아이콘 + 라벨, 옆 전체 실행 버튼 일부) |

---
## 진행 요약

총 **약 155장**의 스크린샷이 필요합니다. 빠른 시작(섹션 2)이 44장으로 가장 큰 비중이며, 나머지는 reference에 가깝습니다. (reference 섹션의 일부 슬롯은 한 화면에 여러 UI를 같이 담는 통합 컷으로 정리되어 있습니다.)

캡처 우선순위 권장:
1. **installation (11장)** — 첫 사용자가 가장 먼저 보는 섹션
2. **quickstart (44장)** — 가장 가치 높은 가이드
3. **modules / training / inference** — 자주 참조됨
4. 나머지 reference 섹션은 점진적으로
