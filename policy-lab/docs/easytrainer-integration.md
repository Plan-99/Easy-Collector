# EasyTrainer 통합 지도 (read-only)

policy-lab 가 EasyTrainer 내부에서 **읽고/쓰는 지점**의 단일 지도. 경로는 EasyTrainer 루트
(`$EASYTRAINER_ROOT = $(git rev-parse --show-toplevel)`) 기준. **랩은 EasyTrainer 코드를 read 하고,
아래 "쓰기 지점"에만 write 한다.** `backend/lerobot/` 는 절대 수정 금지 (read-only).

## 새 정책을 등록하려면 건드리는 6곳 (policy-researcher 레시피)

| # | 파일 (EasyTrainer 루트 기준) | 역할 |
|---|------|------|
| a | `training_server/policies/{type_lower}/{__init__,configuration_*,modeling_*}.py` | 정책 구현 (lerobot import+composition) |
| b | `training_server/train_worker.py` | dispatch `elif policy_obj['type'] == '<Type>'` 추가 |
| c | `backend/tools/model_tester/tutorial_evaluator.py` | `_load_policy_by_config()` 에 `elif` 추가 (relative_ee_pos 면 IK 경로) |
| d | `backend/policies/utils.py` | `make_easytrainer_processors` 분기 (필요시) |
| e | `backend/database/models/policy_model.py` | `POLICY_CONFIGS` 항목 (UI 노출 디폴트) |
| f | `frontend/src/configs/modelConfigs.js` | `POLICY_CONFIGS` + `TRAIN_CONFIGS` (키/디폴트는 e 와 일치) |

## 토너먼트/평가 도구 (랩이 호출, 수정 금지)

- `backend/tools/model_tester/tournament.py` — 토너먼트 run / declare-champion.
- `backend/tools/model_tester/_run_mini_train.py` — 1~N epoch 학습 (dataset-dir/checkpoint-dir 인자).
- `backend/tools/model_tester/tutorial_evaluator.py` — ground-truth 평가 (`--sensor-map`, `_MujocoIK`).
- `backend/tools/model_tester/register_run.py` — DB 행(Task/Dataset/Policy/Checkpoint) prefab (UI 노출).
- `backend/tools/model_tester/assemble_dataset.py` — raw → LeRobot v2.1.

## 시뮬 / 평가 브릿지 (ros2 컨테이너, `ROS_DOMAIN_ID=1` + cyclonedds)

- `ros2 launch mujoco_world tutorial_pegtask.launch.py` → `/tutorial/randomize|run_episode|check_success`.
- `ros2 run mujoco_world tutorial_eval_bridge --port 7799` → `/health`, `/step` (DLS IK 변환).
- 콜렉터: `ros2 run mujoco_world tutorial_collector`.

## 데이터 / 체크포인트 (런타임, `/opt/easytrainer/...`)

- 데이터셋: `/opt/easytrainer/datasets/<id>/` (LeRobot v2.1). 기본 토너먼트 dataset-id = 15.
- 체크포인트: `/opt/easytrainer/training_data/checkpoints/<machine_id>/<id>/` (`resolve_checkpoint_dir`).
- 챔피언: `/opt/easytrainer/training_data/model_tester_champion.json` (strictly-higher 갱신).
- 진화 런타임 산출물: `/opt/easytrainer/training_data/evolve/<run_id>/`.
- 평가 리포트: `/opt/easytrainer/training_data/model_tester_reports/`.

## 컨테이너 / 공유 스크립트 (랩이 재사용)

- 컨테이너: `easytrainer_backend`(:5000), `easytrainer_ros2`, `easytrainer_frontend`(:5173 dev).
- `$EASYTRAINER_ROOT/scripts/smoke.sh` (readiness), `quick_apply.sh` (소스→런타임), `db_snapshot.sh`,
  `overnight_summary.sh` (야간 summary). 랩 `/lab-overnight` 가 이들을 재사용한다.

## 관련 메모리 (영속)
- `lerobot_readonly.md`, `feedback_collector_eepos.md`, `feedback_obs_state_keys_empty.md`,
  `feedback_epoch_budget_by_policy.md`, `project_round1_anchor.md`, `harness_philosophy.md`.

> 이 지도가 EasyTrainer 의 구조 변경으로 어긋나면 갱신한다. EasyTrainer 의 폴더 책임은
> `$EASYTRAINER_ROOT/FOLDERS.md`, 하네스 지도는 `$EASYTRAINER_ROOT/docs/HARNESS.md`.
