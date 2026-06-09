# demo/ — 행사용 시뮬레이션 데모 카탈로그

로봇 관련 행사를 다니면서 하드웨어 없이 보여줄 수 있는 **시뮬레이션 데모**를 모아두는 공간이다.
각 데모는 하나의 하위 폴더이며, `manifest.json` 으로 자기 자신을 기술한다. 상단 nav bar 의
**Sim Activation** 버튼(GPU 버튼 옆)이 이 폴더를 읽어 데모 목록을 보여주고, 선택 후 "실행하기" 를
누르면 해당 환경이 켜진다.

## 폴더 규약

```
demo/
└── <demo_id>/
    ├── manifest.json        # 데모 메타데이터 + launch/collector 정의 (필수)
    ├── env/                 # 시뮬레이션 환경 소스 (참조용 단일 진실원천 복사본)
    └── collect/             # 자동 데이터 수집 코드 (모델 토너먼트와 동일)
```

`manifest.json` 스키마:

| 키 | 의미 |
|----|------|
| `id` | 데모 식별자 (폴더명과 동일) |
| `name` / `name_en` | UI 표시명 (한/영) |
| `description` / `description_en` | 설명 (한/영) |
| `topic_prefix` | sim 이 publish 하는 ROS 토픽 prefix |
| `cameras` | 카메라 슬러그 목록 |
| `launch.process_id` | 브릿지가 관리할 프로세스 id |
| `launch.package` / `launch.launch_file` | `ros2 launch` 대상 (ros2 컨테이너 install 공간에 설치돼 있어야 함) |
| `launch.args` | launch 인자 (예: `show_viewer`) |
| `collector.*` | 자동 데이터 수집 정보 |
| `services.*` | randomize / run_episode / check_success 등 표준 Trigger 서비스 이름 |

## 백엔드 API

`backend/api/routes/demo.py` 가 이 폴더를 읽는다.

- `GET  /api/demo/list`    — manifest 목록
- `GET  /api/demo/status`  — 현재 실행 중인 데모 sim 상태 (process + topics)
- `POST /api/demo:start`   — `{ "demo_id": "peg_in_hole" }` 로 환경 기동
- `POST /api/demo:stop`    — 실행 중인 데모 sim 종료

## 데모 목록

### peg_in_hole — Peg-in-Hole 정밀 삽입
모델 토너먼트(`/model-test`, `/policy-tournament`) 벤치마크와 **동일한** MuJoCo 환경.
빨간 peg 를 노란 hole 에 삽입한다. wrist 2-cam (wrist_cam, wrist_cam_down) 셋업.

- 환경: [peg_in_hole/env/scene_peg.xml](peg_in_hole/env/scene_peg.xml) + `tutorial_pegtask.launch.py`
  (MuJoCo world `mujoco_world_node` + `tutorial_planner_node`)
- 자동 수집: [peg_in_hole/collect/tutorial_collector_node.py](peg_in_hole/collect/tutorial_collector_node.py)
  — `randomize → run_episode → check_success` 를 반복하며 **성공한 에피소드만** 디스크에 저장.
- 파이프라인(수집 → LeRobot 어셈블 → ACT 학습 → 평가)은 `backend/tools/model_tester/` 가 담당한다.

## 자동 수집 → 학습 → 평가 (모델 토너먼트와 동일)

데모 환경을 켠 뒤, 모델 토너먼트와 동일한 흐름으로 100개 데이터를 모으고 ACT 로 학습한 뒤
추론 성공률을 기록한다. 자세한 절차는 리포지토리 루트의 `/model-test` 스킬 문서를 따른다.

```bash
# 1) 데모 환경 기동 (UI 의 Sim Activation 버튼 또는 API)
curl -s -X POST localhost:5000/api/demo:start -H 'Content-Type: application/json' \
     -d '{"demo_id":"peg_in_hole"}'

# 2) 자동 수집 100개 (ros2 컨테이너) — bridge sim 은 ROS_DOMAIN_ID=0 에 뜬다 (스킬 문서의 1은 구버전)
docker exec easytrainer_ros2 bash -lc '
  source /opt/ros/humble/setup.bash && source /root/ros2_ws/install/setup.bash &&
  export ROS_DOMAIN_ID=0 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp &&
  ros2 run mujoco_world tutorial_collector_node --num-episodes 100 \
      --cameras top_cam,front_cam,wrist_cam \
      --output-dir /opt/easytrainer/training_data/raw_episodes'

# 3) 어셈블 + ACT 학습 + 평가 → backend/tools/model_tester (register_run / assemble_dataset /
#    _run_mini_train / tutorial_evaluator). 결과는 RESULTS.md 에 기록.
```

> ⚠️ 현재 상태: sim 기동·서비스(randomize/check_success)·수집기 plumbing 은 복원돼 동작하지만,
> scripted planner 의 peg 삽입 성공률이 0% 라 100개 수집이 완료되지 않는다. 근본원인(물리-렌더
> 단일스레드 starvation)과 재현/측정은 [RESULTS.md](RESULTS.md) 에 정리돼 있다.
