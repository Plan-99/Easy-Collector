# LeRobot 리뉴얼 대응 — 수행 완료 보고서

> 2026-04-06 수행 완료

## 개요

`src/backend/lerobot/`이 lerobot 전체 git repo로 교체됨에 따라, EasyTrainer의 학습/추론 파이프라인이 ACT, Diffusion, PI05에 대해 정상 동작하도록 수정 완료.

---

## 수정 내역

### Phase 1: Python 3.10 호환성 (vendored lerobot)

Docker 컨테이너가 Python 3.10인데, 리뉴얼된 lerobot이 Python 3.11~3.12 문법을 사용.

| 문제 | 해결 | 파일 |
|------|------|------|
| `from typing import Unpack` (3.11+) | `typing_extensions` 폴백 | pretrained.py, factory.py, modeling_pi05.py, modeling_pi0.py, modeling_pi0_fast.py, modeling_smolvla.py |
| `class Foo[T]:` (3.12+) | `datasets/__init__.py`에서 StreamingLeRobotDataset lazy import | datasets/__init__.py |
| `def foo[T: Bound]()` (3.12+) | `TypeVar` 방식으로 변환 | utils/io_utils.py |
| `type X = str \| int` (3.12+) | import chain 차단 (pretrained.py에서 TrainPipelineConfig 임포트 제거) | pretrained.py |
| Groot dataclass 3.10 비호환 | `policies/__init__.py`에서 lazy import | policies/__init__.py |
| pi05 processor 3.12 문법 의존 | `pi05/__init__.py`에서 lazy import | pi05/__init__.py |

### Phase 2: PYTHONPATH 설정

- `start_services.sh` — PYTHONPATH에 `/root/src/backend/lerobot/src` 추가
- `scripts/train.py` — sys.path에 `lerobot/src` 디렉토리 추가
- `start_services.sh` — 구형 `ensure_lerobot_dataset_init()` 함수 및 호출 제거

### Phase 3: 임포트 마이그레이션 (10개 파일)

모든 상대 임포트 (`from ..lerobot.X`, `from ...lerobot.X`, `from backend.lerobot.X`) → 절대 임포트 (`from lerobot.X`).

리뉴얼에서 함수 위치가 변경된 것들:
- `datasets.utils.get_hf_features_from_features` → `datasets.feature_utils`
- `datasets.utils.embed_images` → `datasets.io_utils`
- `datasets.utils.create_empty_dataset_info` → `datasets.feature_utils`
- `datasets.utils.dataset_to_policy_features` → `datasets.feature_utils`
- `DEFAULT_PARQUET_PATH` → 삭제 (미사용)

수정 파일: train.py, checkpoint_test.py, policies/utils.py, lerobot_io.py, augment_dataset.py, generate_ood_features.py, merge_dataset.py, oti_rl.py, test_vla.py, embedding_helper.py

### Phase 4: PI0 / VLAsEn 제거

- `train.py` — PI0Config/PI0Policy, VLAsEnConfig/VLAsEnPolicy 임포트 및 분기 삭제
- `checkpoint_test.py` — PI0Policy 분기 삭제
- `modelConfigs.js` — PI0, VLAsEn 설정 삭제
- `oti_rl.py`, `test_vla.py` — PI0 임포트 삭제

### Phase 5: 생성자 변경

```python
# Before
policy = ACTPolicy(cfg, dataset_stats=stats)
# After
policy = ACTPolicy(config=cfg)
```

ACT, Diffusion, PI05 모두 동일. 새 lerobot은 `dataset_stats`를 생성자에서 받지 않음 (`**kwargs`로 흡수되고 무시됨).

### Phase 6: PI05 토큰화 브릿지

PI05의 `forward()`와 `predict_action_chunk()`가 `observation.language.tokens`, `observation.language.attention_mask`를 기대하지만, EasyTrainer는 raw `language_instruction` 문자열만 제공.

`policies/utils.py`에 `prepare_pi05_language_tokens()` 함수 추가:
1. state를 min-max로 [-1,1] 정규화
2. 256 bin 이산화 (OpenPI 방식)
3. 프롬프트 구성: `"Task: {text}, State: {bins};\nAction: "`
4. PaliGemma tokenizer로 토큰화 (max_length=200)

연동:
- `forward_pass(batch, policy, norm_stats=None)` — PI05일 때 자동 토큰화
- `checkpoint_test.py` — 추론 시 `prepare_pi05_language_tokens()` 호출

### Phase 7: UI 수정

- `modelConfigs.js` — Diffusion noise_scheduler_type: `DPM-Solver` → `DDIM`

---

### Phase 8: PYTHONPATH 템플릿 수정 (2026-04-07)

`docker-compose.gpu.yml`, `docker-compose.cpu.yml`에 `/root/src/backend/lerobot/src` 추가.
런처가 `_apply_compose_variant()`로 템플릿을 `docker-compose.yml`로 복사하므로 템플릿 수정이 필수.
`docker-compose.yml`에 `name: project` 추가하여 compose 프로젝트 이름 고정.

### Phase 9: 런처 시작 로직 수정 (2026-04-07)

- `release/ui/service.py` `_ensure_service_running()` — `docker compose start` → `up -d` 변경
- `_clear_conflicting_containers()` — `easy_collector_service` 추가 (orphan 컨테이너 정리)

### Phase 10: 구형 잔재 파일 정리 (2026-04-07)

컨테이너 내 `/root/src/backend/lerobot/__init__.py`, `datasets/__init__.py` 삭제.
이전 `ensure_lerobot_dataset_init()`이 만든 파일이 `backend.lerobot` 서브패키지를 만들어 새 lerobot과 충돌.

### Phase 11: 데이터셋 저장 수정 (2026-04-07)

- `lerobot_io.py` `append_episode()` — `hf_datasets.Dataset.from_dict()` → `pyarrow` 직접 parquet 저장
- HF datasets `encode_nested_example`의 numpy scalar `len()` 에러 우회

### Phase 12: quick_apply.sh 수정 (2026-04-07)

rsync exclude에 `datasets/`, `.git/` 추가하여 Permission denied 해결.

### Phase 13: 이미지 전처리 수정 (2026-04-07)

- `policies/utils.py` `process_image()` — `transforms.Resize((224, 224))` 추가 (resnet18 분기)
- 데이터셋 이미지 크기가 에피소드마다 다를 수 있어 (224x224 vs 240x320) `torch.stack` 실패 방지
- numpy array → PIL Image 자동 변환 추가 (inference 시 numpy가 직접 전달되는 문제 해결)

### Phase 14: EpisodicDataset 텐서 shape 수정 (2026-04-07)

- 새 lerobot ACT는 이미지를 `[batch, C, H, W]` 4D로 기대
- 기존: `[n_obs_steps, C, H, W]` → collate 후 `[batch, n_obs_steps, C, H, W]` 5D → conv2d 에러
- 수정: `n_obs_steps=1`일 때 squeeze하여 `[C, H, W]` 반환
- state도 동일: `n_obs_steps=1`일 때 `qpos[0]` (1D)으로 반환
- `self.info` shape도 `val[0].shape` → `val.shape`로 수정

### Phase 15: Validation 모드 수정 (2026-04-07)

- `scripts/train.py` — validation에서 `policy.eval()` 제거
- 새 lerobot ACT의 VAE 인코더는 `self.training=True`일 때만 동작
- eval 모드에서 `mu=None`, `log_sigma_x2=None` 반환 → KLD 계산 TypeError
- `torch.inference_mode()`는 유지 (gradient 비활성화)

### Phase 16: 체크포인트 config.json에 type 키 추가 (2026-04-07)

- `scripts/train.py` — `save_pretrained()` 후 config.json에 `type` 키 수동 추가
- `draccus.dump()`가 `type` property를 직렬화하지 않음
- `from_pretrained()`는 `ChoiceRegistry` 서브클래스 판별에 `type` 키 필요 (예: `"type": "act"`)

### Phase 17: 학습 속도 최적화 (2026-04-07)

- `EpisodicDataset.__init__` — `tasks.jsonl`을 한 번만 로드하여 `_task_map`으로 캐시
- 기존: 매 `__getitem__`마다 `_read_jsonl()` 파일 I/O
- 에피소드 캐시 50개 제한 제거 → 전체 에피소드 메모리 캐시

---

## 검증 결과

- ACTPolicy, DiffusionPolicy, PI05Policy 임포트 성공 (Docker 컨테이너 내)
- Flask 백엔드 정상 기동 (포트 5000, ImportError 없음)
- 런처 "나중에 하기" → 서비스 정상 시작
- 데이터셋 에피소드 저장 정상 동작
- **ACT 학습 정상 완료** (5 epoch, val loss 42.45)
- **ACT inference 정상 동작** (checkpoint_test)
