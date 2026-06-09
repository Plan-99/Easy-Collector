# SPEC — Planner 블록 `visual_reach` (Visual Reach, wrist-cam)

> foundry `/forge` 로 만든 이 기능 전용 sub-harness 의 계약(contract). 구현 전 "done" 을 못박는다.
> 검증 오라클은 **MuJoCo ground-truth** (verify/oracle.md). 설계는 [DESIGN.md](DESIGN.md).

## 목표 (한 문장)
플래너 블록 하나로: 손목(wrist) depth 카메라가 장면을 잘 보는 자세로 이동 → 사람이 타겟 오브젝트를
쉽게 지정 → wrist depth 로 **EE(그리퍼) 핑거팁과 타겟의 상대 3D 위치차**를 계산 → EE 를 타겟
**바로 위(약간 높게)** 로 이동시킨다.

## 블록 이름
- type id: `visual_reach`  / label: **Visual Reach** (ko: "비전 접근(wrist)") / icon: `my_location` / color: `pink`.

## 요구사항 → 수용 기준 (Acceptance, 사용자 7개 항목 매핑)

1. **[R1] 사전 관찰 자세로 이동.** 블록에 사용자 지정 "관찰 자세"(joint 또는 EE pose)가 있고, 실행
   시 먼저 그 자세로 이동한다. — 검증: 블록 실행 1단계 후 로봇이 그 자세(±tol)에 도달.
2. **[R2] wrist 카메라 지정(depth 필수).** 블록 설정에서 wrist 카메라(sensor)를 고른다. 그 센서는
   **depth 가능**(realsense 등; sim 은 depth 렌더 wrist cam)이어야 하고, 아니면 저장이 막힌다. —
   검증: depth 미지원 센서 선택 시 폼이 거부.
3. **[R3] 설정에서 wrist 실시간 스트리밍.** 블록 설정 다이얼로그가 wrist 카메라를 라이브로 보여준다.
   — 검증: 설정 다이얼로그에 `WebRtcVideo` 스트림이 뜬다.
4. **[R4] 타겟 선택(쉽게).** 사람이 **바운딩박스** 또는 **랭귀지 프롬프트**로 타겟을 지정. 프롬프트가
   최대한 쉬워야 함(텍스트 한 줄 = 최우선). 기술은 SAM3 재사용(text/box → mask). — 검증: 텍스트
   "red cube" 한 줄 또는 박스 1개로 마스크/타겟이 잡힌다.
5. **[R5] wrist 뷰에 핑거팁 가시 + 위치.** wrist 뷰에서 EE 핑거팁이 보이고 그 위치를 안다. 자동
   감지 우선(대칭 두 돌출), 안 되면 사용자 지정. — 검증: sim 에선 FK 투영으로 핑거팁 픽셀/3D 가
   ground-truth 와 일치(±tol); 실물은 depth 대칭 검출 또는 사용자 클릭.
6. **[R6] 실행: depth 로 상대위치차 계산 → 타겟 위 약간 높게 이동.** wrist depth 에서 타겟 3D 와
   핑거팁(=EE) 3D 를 얻어 delta 를 만들고 EE 를 `target_xyz + [0,0,+hover]` 로 IK 이동. — 검증:
   verify/oracle (아래).
7. **[R7] MuJoCo 로 먼저 구축·검증.** sim 에서 실제 실행해 타겟으로 잘 가는지 확인한 뒤 실물 경로.

## 검증 오라클 (done 의 단일 판정 — ground-truth, memory `feedback_success_check`)
MuJoCo 는 타겟 body 와 `ee_site` 의 ground-truth pose 를 안다. 블록 실행 후:
- **수평 오차** `‖ee_xy − target_xy‖ < TOL_XY` (기본 0.03 m)
- **높이** `ee_z ∈ [target_top_z + HOVER − TOL_Z, target_top_z + HOVER + TOL_Z]` (HOVER 기본 0.06 m, TOL_Z 0.02 m)
- N 회 랜덤 타겟 위치 시도 중 **성공률 ≥ TARGET_RATE**(기본 0.8).
세부·스크립트: [verify/oracle.md](verify/oracle.md).

## 비-목표 (Out of Scope — 이번 사이클에서 안 함)
- 잡기(grasp)/삽입까지: 이 블록은 **"타겟 위로 접근"까지만**. 하강·집기는 별도 블록.
- 실물 RealSense 하드웨어 연동 검증: sim-first. 실물 경로는 설계만 두고 하드웨어 검증은 후속.
- 자유 free-text 오픈보캡 detector 신규 도입: SAM3(text/box) 재사용으로 충분. DINO/YOLO 추가 안 함.
- closed-loop visual servo(수렴 반복): 1-shot 추정→이동. 반복 servo 는 후속 가능.

## 구현 슬라이스(검증 가능한 순서)
- **M1 (sim core, 헤드리스):** scene 에 wrist depth cam 추가 → 합성 perception(prompt→mask→centroid→depth→3D) + 핑거팁 FK 투영 → IK 이동 → oracle 측정. **여기서 "타겟으로 잘 간다"를 먼저 증명.**
- **M2 (block 배선):** backend `BLOCK_CONFIGS` + `_HANDLERS` 에 `visual_reach` 추가, sim 경로 호출.
- **M3 (frontend):** PlannerPage 블록 폼(카메라 선택 + 라이브 스트림 + 타겟 텍스트/박스 + 핑거팁) + 카드/디테일 + i18n. DESIGN.md 준수.
- **M4 (실물 설계 검증):** RealSense aligned-depth 경로 + 실물 핑거팁 검출 설계 확인(하드웨어 있으면 실측).
