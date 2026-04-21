# EasyTrainer Remote Training Server

GPU 학습 서버를 분리하여 별도의 머신에서 모델을 학습시킬 수 있습니다.

## 설치 및 실행

### 1. 의존성 복사
```bash
cd training_server
bash setup_deps.sh
```
이 스크립트는 Easy-Collector에서 공유 코드(policies, lerobot, lerobot_io)를 복사합니다.

### 2. Docker 빌드 및 실행
```bash
docker compose up -d
```

서버는 **포트 5100**에서 실행됩니다.

## 사용 방법

1. Easy-Collector의 **Train 페이지**에서 Step 3 (Training Configuration)으로 이동
2. **"Use Remote Training Server"** 토글 활성화
3. 학습 서버 URL 입력 (예: `http://192.168.1.100:5100`)
4. 연결 확인 버튼 클릭 (GPU 상태 표시)
5. **Start Training** — 데이터셋과 모델이 자동으로 업로드되고 학습 시작
6. 학습 진행상황은 기존 TrainingDialog에서 실시간 모니터링
7. 학습 완료 시 모델이 자동으로 로컬 PC로 전송됨

## API 엔드포인트

| Method | Path | 설명 |
|--------|------|------|
| GET | `/api/health` | 서버 상태 및 GPU 확인 |
| POST | `/api/train/upload-dataset` | 데이터셋 업로드 (tar.gz) |
| POST | `/api/train/upload-model` | 베이스 모델 업로드 (finetuning) |
| POST | `/api/train/start` | 학습 시작 |
| GET | `/api/train/status/<job_id>` | 학습 상태 조회 |
| POST | `/api/train/stop/<job_id>` | 학습 중단 |
| GET | `/api/train/download/<job_id>` | 학습된 모델 다운로드 |
| GET | `/api/train/jobs` | 전체 작업 목록 |

## WebSocket 이벤트

- `task_log` — 학습 로그 (`[TRAIN_LOG]` JSON 포함)
- `train_status` — 학습 상태 변경 (training, finished, failed, delivered)

## 모델 자동 전송

학습 완료 시 두 가지 방식으로 모델이 로컬로 전달됩니다:
1. **Callback (Push)**: 학습 서버가 로컬 API의 `/api/remote-train/receive-model`로 POST
2. **Download (Pull)**: Callback 실패 시 WebSocket/폴링으로 완료 감지 후 자동 다운로드
