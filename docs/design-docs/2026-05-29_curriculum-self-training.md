# Curriculum 자가 학습 (Self-Training Curriculum)

- **결정일**: 2026-05-29
- **상태**: 적용 중
- **영향 폴더**: [backend/](../../backend/), [frontend/](../../frontend/), [training_server/](../../training_server/)

## 결정

Planner 위에 **Curriculum(커리큘럼)** 계층을 얹어, 학습된 체크포인트를 베이스로 로봇이
**스스로 롤아웃하며 데이터를 수집하고, 난이도를 점진적으로 올리며 재학습**하는 자가 학습
루프를 만든다. 핵심 엔티티 계층은 다음과 같다.

```
Planner (기존)
 └─ Curriculum            플래너와 1:1. 얇은 컨테이너(planner_id, status).
     ├─ CheckpointGroup   성공/실패 판정 단위. 플래너 블록을 그룹에 배정해서 구성.
     │    - 멤버: checkpoint_ids(체크포인트 블록) + motion_block_ids(모션 블록)
     │    - color: 그룹 색상. 정책(mission). 체크포인트별 학습설정(checkpoint_settings).
     │    - block_noise: 모션 블록별 noise(rate+offset, x/y/z/ax/ay/az).
     │    └─ Stage        그룹에 종속. 정책으로부터 계산된 현재 난이도 스냅샷 + 상태.
     │                    생성 시 그룹 내 체크포인트마다 success/failure/dagger 3개 데이터셋 생성.
     └─ Rollout           플래너 1회 실행. 타겟 그룹들을 지정해 성공/실패 판정.
          └─ RolloutResult  (rollout × group) 단위 결과/저장된 에피소드 참조.
```

## 용어 (확정)

| 코드 식별자 | UI/한글 | 의미 |
|---|---|---|
| `Curriculum` | 커리큘럼 | 플래너와 1:1 자가학습 컨테이너. 플래너 선택 즉시 자동 생성·설정 |
| `CheckpointGroup` | 체크포인트 그룹 | 성공/실패 판정 단위. **플래너 블록 배정**으로 구성, 색상으로 구분 |
| `Stage` | Stage | 정책으로부터 계산된 현재 난이도(noise/criteria) 스냅샷 + 상태 |
| `Rollout` | 롤아웃 | 플래너 1회 실행 |
| `block_noise` | Rollout Noise | 모션 블록별 noise(rate+offset). 그룹에 저장, Stage가 escalate한 스냅샷 사용 |
| `success_criteria` | 판정 조건 | 체크포인트별 성공/실패 조건 (이전 "Condition") |
| `mission` | 그룹 정책 | 승급 목표·실패저장확률·난이도 상승률. **그룹**에 종속 |
| `checkpoint_settings` | 체크포인트별 설정 | base 데이터셋 + training 파라미터. **체크포인트별**(그룹 내 맵) |

## 설정의 종속 레벨 (확정)

- **커리큘럼**: `planner_id`, `status` 만. (base/train 없음 — 체크포인트별로 이동)
- **체크포인트별**(그룹 내 `checkpoint_settings[cp_id]`): `base_dataset_ids`, `train_settings`(epoch/batch/server_url).
- **체크포인트 그룹**: 멤버(`checkpoint_ids`, `motion_block_ids`), `color`, 정책 `mission`,
  `block_noise`(모션 블록별 rate/offset base), `checkpoint_settings`.
- **Stage**(그룹 종속): `rollout_noise`·`success_criteria` **현재 스냅샷** + `success_count`/`failure_count`/`trained_checkpoint_id`/`status`.

## 그룹 구성 / 색상 / 블록 배정

- 커리큘럼 최초 생성 시 **그룹 1개**를 자동 생성하고 플래너의 **모든 체크포인트를 그 그룹**에 넣는다.
- **플래너 탭**(좌측)에서 플래너 블록을 세로로 나열(읽기 전용). 체크포인트 블록은 **소속 그룹 색상**,
  모션 블록 중 그룹에 배정된 것은 **그 색상의 연한 톤**, 그 외/미배정 블록은 어두운 회색.
- 각 블록의 **설정 버튼** → 다이얼로그:
  - 체크포인트 블록: 그룹 **변경**(다른 그룹이 있을 때만) / **분리**(그룹에 나 혼자가 아닐 때만, 새 그룹 이름 입력)
    + 체크포인트별 설정(base 데이터셋, train_settings).
  - 모션 블록: 영향 줄 **그룹 선택** + noise **rate/offset**(x,y,z,ax,ay,az) 입력.
- **그룹 정책 탭**: 그룹 선택 → 정책(mission) 설정.

## 배경 (Why)

Planner(`backend/api/process/planner_run.py`)는 체크포인트 추론 + 모션을 파이프라인으로
엮어 **실행**할 수 있지만, ① 에피소드를 녹화하지 않고 ② 모션에 노이즈를 주지 않으며
③ 성공/실패를 판정하거나 그 결과로 데이터를 분류·재학습하지 않는다. 자가 학습을 위해선
이 셋이 필요하다. 학습 인프라(`remote_train.run_training_job` — 다중 데이터셋 병합 +
`load_model_id` 이어학습 + training_server 연동)는 이미 갖춰져 있어 그대로 재사용한다.

## 구조 / 적용

### 데이터 모델 (Peewee, `backend/database/models/`)
- `curriculum_model.py` — `Curriculum`: `planner_id`, `name`, `status`.
- `checkpoint_group_model.py` — `CheckpointGroup`: `curriculum_id`, `name`, `color`,
  `checkpoint_ids`(json), `motion_block_ids`(json),
  `checkpoint_settings`(json, `{cp_id: {base_dataset_ids, train_settings}}`),
  `block_noise`(json, `{block_id: {rate:{x,y,z,ax,ay,az}, offset:{...}}}`),
  `mission`(json, `{target_success_count, failure_save_prob, noise_escalation_var, condition_escalation_var}`).
- `stage_model.py` — `Stage`: `checkpoint_group_id`, `index`, `status`,
  `rollout_noise`(json 스냅샷, 블록별), `success_criteria`(json, 체크포인트별 `{max_steps, success_threshold, fallback_block}`),
  `success_count`, `failure_count`, `trained_checkpoint_id`. (mission 은 그룹으로 이동)
- `rollout_model.py` — `Rollout`: `curriculum_id`, `target_group_ids`(json), `noise_snapshot`(json), `status`.
- `rollout_result_model.py` — `RolloutResult`: `rollout_id`, `checkpoint_group_id`, `success`(bool), `episodes`(json).
- `dataset_model.py` 확장 — `stage_id`, `checkpoint_group_id`, `checkpoint_id`, `role`(`success|failure|dagger`),
  `origin`(`manual|curriculum`). 신규 모델은 `migrate.py`의 `ALL_MODELS`에 등록.

### 데이터셋 규칙
- Stage 생성 시 그룹 내 체크포인트마다 3개 데이터셋(success/failure/dagger)을 자동 생성.
  `task_id`는 체크포인트의 워크스페이스로 설정(워크스페이스 종속) + `stage_id`로 Stage 종속.
- `origin == 'curriculum'` 데이터셋은 워크스페이스 UI에서 **편집/병합/삭제 차단**.
  **커리큘럼 초기화(reset)** 또는 **실패 데이터 버리기(discard failure)** 로만 변경.
- Dagger 데이터셋은 현재 **생성만** 한다 (채우는 메커니즘은 후속 결정).

### 롤아웃 (`planner_run.py` 확장 + `curriculum_rollout.py`)
1. **노이즈**: 모션 블록(`joint_position`/`move_relative_ee`/`query_pose`)별 x,y,z,ax,ay,az delta.
   joint pos는 해당 joint의 EE pose에 노이즈 후 IK 재계산.
2. **녹화**: 타겟 그룹 체크포인트 블록 실행 중 obs+action 버퍼링(`record_episode` 재사용).
3. **판정**: 체크포인트별 `max_steps` 내 success token 미획득 → 실패, `fallback_block` 실행.
   그룹 성공 = 그룹 내 전 체크포인트 성공.
4. **저장**: 그룹 성공 → success 데이터셋. 실패 → 진행된 체크포인트 에피소드만 **저장 확률**로
   failure 데이터셋. 성공/실패 카운트는 확률과 무관하게 항상 갱신.

### 승급 & 학습
- Stage Mission(`target_success_count` 등) 충족 → (현 stage success + 이전 stage들 + base) 집계
  → `remote_train` enqueue(`load_model_id` = 현재 체크포인트). 완료 시 새 체크포인트가 그룹의
  현재 체크포인트가 되고 **다음 Stage 자동 생성** + 조건 업그레이드:
  `next_noise = 성공률 × var + 현재노이즈`, `next_criteria = 성공 에피소드 평균 길이 × var`.

### API / UI
- `backend/api/routes/curriculum.py` (신규 Blueprint, `app.py` 등록): Curriculum/Group/Stage CRUD,
  Rollout `:start`/`:stop`/`:status`, `:reset`, `:discard_failure`.
- `frontend/src/pages/v2/CurriculumPage.vue` (신규) + 라우트 등록.

## 운영 시 주의
- ORM은 **Peewee** (CLAUDE.md의 "Orator"는 오래된 기술). 신규 테이블은 `ALL_MODELS` 등록만 하면
  `migrate()`가 `create_tables` + `ALTER TABLE`(`_ensure_columns`)로 처리.
- 롤아웃은 ProcessManager **function task**(`task_control['stop']` 협력 종료)로 돌린다 —
  강제 종료(subprocess)가 아니라 로봇이 안전 지점에서 멈추도록.
- `origin=='curriculum'` 데이터셋을 워크스페이스에서 건드리면 카운트/성공률 정합성이 깨진다 —
  반드시 reset/discard 경로로만.
