# Remote Training Server 분리

- **결정일**: 2026-04-17
- **상태**: 적용 중
- **영향 폴더**: [training_server/](../../training_server/), [backend/](../../backend/), [frontend/](../../frontend/)

## 결정

학습(Training) 기능을 메인 Easy-Collector에서 분리하여 독립 Docker 서비스로 운영한다.
포트는 **5100** (메인 backend API는 5000).

## 배경 (Why)

데이터 수집·로봇 제어는 로컬 PC에서 수행하지만, 학습은 GPU를 많이 쓰는 워크로드라
**원격의 학습 전용 머신**에 넘기고 싶었다. 단일 컨테이너에 학습을 포함하면 로컬 GPU 자원을
점유하고, 원격 GPU 머신을 활용할 수 없다.

## 구조

| 컴포넌트 | 위치 | 역할 |
|---------|------|------|
| Training Server | [training_server/](../../training_server/) | 독립 Docker 서비스, 포트 5100 |
| Remote train routes | `backend/api/routes/remote_train.py` | 로컬 backend → 원격 training server 프록시 |
| Frontend toggle | `frontend/src/pages/v2/TrainPage.vue` (Step 3) | "Use Remote Training Server" 스위치 |
| Build helper | `training_server/setup_deps.sh` | Docker 이미지 빌드 전 공용 코드 복사 |

## 모델 전달 메커니즘

학습 완료 모델은 **이중 경로**로 회수한다:

1. **Push**: training server가 메인 backend의 callback URL로 모델 업로드
2. **Pull**: 실패 시 WebSocket 이벤트 + 다운로드 엔드포인트로 폴백

WebSocket 진행 로그는 `task_log` 포맷에 `[TRAIN_LOG]` 프리픽스를 붙여 통일한다.

## 운영 시 주의

- training server 이미지 빌드 전 반드시 `setup_deps.sh` 실행 (그렇지 않으면 공용 코드 누락)
- `EASYTRAINER_DATA_DIR`는 training server 쪽에서도 동일하게 적용됨
- 신규 학습 API는 `remote_train.py`에 추가하고 frontend는 toggle 분기를 거치도록 통일

## 관련

- [Launcher 인증 & 모듈 entitlement](2026-04-28_launcher-auth.md)
