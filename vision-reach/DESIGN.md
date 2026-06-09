# DESIGN — `visual_reach` (grounded in 실제 코드 앵커)

> Explore 로 확인한 실제 통합점만 사용(추측 금지). 경로는 EasyTrainer 루트 기준.

## 데이터 흐름 (실행 시)
```
[R1] 관찰자세 이동  →  [R4] 타겟 선택(SAM3 text/box→mask)  →  마스크 centroid 픽셀 (u,v)
   →  wrist depth[v,u] = Z  →  카메라 intrinsics + extrinsics 로 (u,v,Z) → world XYZ_target
[R5] 핑거팁 3D: sim=FK(ee_site/finger geom) 투영, 실물=depth 대칭검출/사용자점
[R6] delta = (XYZ_target + [0,0,+HOVER]) − XYZ_eetip  →  agent.move_ee_to(target_ee)  (IK)
```

## 재사용 (이미 존재 — 만들지 않음)
- **블록 디스패치(backend):** `backend/api/process/planner_run.py:969` `_HANDLERS`; 핸들러 시그니처는
  기존 `_run_move_relative_ee`(:279) 패턴 따른다. EE 제어: `agent.get_ee_position()`(:342),
  `agent.move_ee_to(target_ee, duration)`. IK 는 agent(ROS2 gRPC `MoveEETo` / sim `_MujocoIK`)가 담당.
- **블록 설정(backend):** `backend/api/routes/planner.py:18` `BLOCK_CONFIGS` 에 항목 추가 →
  `GET /planner/block_configs` 가 프론트로 전달.
- **타겟 선택(SAM3):** `POST /sensor/<id>/sam3_preview`(`backend/api/routes/sensor.py:159`), body
  `{text_prompts, boxes, mode, color}` → base64 PNG. helper `backend/utils/sam3_helper.py`
  (`preview_mask`). **마스크에서 centroid 추출은 신규(아래).** SAM3 모듈 게이트: `modulesStore.has('sam3')`.
- **라이브 스트림(frontend):** `frontend/src/components/v2/WebRtcVideo.vue`(`:topic`,`:msgType`,`:sensorId`).
- **MuJoCo IK:** `backend/tools/model_tester/tutorial_evaluator.py:106` `_MujocoIK`(fk/solve, `ee_site`).
- **블록 폼/카드/디테일(frontend):** `PlannerPage.vue`(폼 510–699 패턴, 디테일 345–423),
  `components/v2/PlannerBlockCard.vue`, `FormDialog.vue`. 규칙: `frontend/DESIGN.md` (no-CSS, i18n 양쪽, 8-step 체크리스트).

## 신규 구현 (gap)
1. **wrist depth 카메라 (sim) — 대부분 이미 존재.** `scene_peg.xml` 에 `wrist_cam`(전방, fovy 65,
   `gripper_base` 부착) + `wrist_cam_down`(top-down, fovy 55)가 **이미 있다**. MuJoCo depth 렌더는
   `Renderer(...).enable_depth_rendering()` + `MUJOCO_GL=egl` 로 ros2 컨테이너에서 **검증됨**.
   M1 검증은 `wrist_cam`(전방)이 peg+hole+핑거팁을 다 봐서 최적(RESULTS.md). 남은 일은 실시간
   publish(M2/M3) 뿐 — sim 자산 추가는 불필요. `scene.xml`(큐브 씬)에는 wrist cam 없음(필요시 추가).
2. **perception: prompt/box → 3D centroid:** 마스크(SAM3 결과 or sim 합성)에서 centroid 픽셀 →
   depth 샘플(중앙값) → intrinsics 역투영 → camera frame XYZ → wrist extrinsics(FK) 로 world XYZ.
   sim 에선 카메라 intrinsics 를 MuJoCo cam fovy/해상도로 계산.
3. **핑거팁 위치:** sim = `ee_site`(또는 `finger_left/right` geom 중점)를 FK 로 world→wrist 투영(정확).
   실물 = depth 에서 대칭 두 돌출 검출, 실패 시 사용자 클릭 1점(블록 설정에 저장).
4. **`visual_reach` 핸들러 + 블록 type:** 위 흐름을 backend 핸들러로. sim 경로는 `_MujocoIK` 재사용.

## 카메라 모델 (정확도 핵심)
- intrinsics: sim 은 `fy = (H/2)/tan(fovy/2)`, `fx=fy*(W/H) 또는 fovx`, `cx=W/2, cy=H/2`.
- 역투영: `X=(u-cx)*Z/fx, Y=(v-cy)*Z/fy, Zc=Z` (카메라 광축 +Z, MuJoCo 카메라는 -Z 전방이므로 부호 주의).
- extrinsics: wrist cam pose 는 FK(`ee_site` 기준 고정 오프셋). world XYZ = T_world_cam · [X,Y,Z,1].
- **검증 가능성:** sim 에선 ground-truth 타겟 body / `ee_site` 를 알므로 perception 추정치를 직접 대조 가능(별도 오라클).

## 실물 경로(설계만, M4)
- 카메라: RealSense (`backend/lerobot/src/lerobot/cameras/realsense/`) aligned depth. intrinsics 는 드라이버 제공.
- extrinsics: wrist→EE 캘리브레이션(hand-eye) 필요. 핑거팁: depth 대칭검출 또는 사용자 1-클릭.
- detector: SAM3 text/box. EE 이동: 동일 `agent.move_ee_to`(실물 IK).

## 좌표/단위 규약
- 길이 m, world frame, EE pose `[x,y,z,rx,ry,rz]`(planner 규약, `move_relative_ee` 와 동일).
- HOVER 기본 +0.06 m (타겟 top 위). 접근만 — 회전은 관찰자세 유지(타겟 위 수직 하강 준비 자세).
