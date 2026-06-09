# RESULTS — `visual_reach` 검증 로그

## M1 (sim core, headless MuJoCo) — ✅ VERIFIED (2026-06-01)

러너: `verify/run_oracle.py` (ros2 컨테이너, `MUJOCO_GL=egl`, scene_peg.xml, `wrist_cam`).

**L1 (perception 정확도):** 마스크 centroid → depth → 핀홀 역투영 + MuJoCo extrinsics 로 world XYZ.
- XY 추정 오차 ~1–8 mm (밀리미터급). Z = peg 가시 top 면(0.065 m)과 일치 → "위로 접근" 기준으로 정확.
- L1 평균오차 0.0147 m (대부분 <0.01 m; 단일픽셀 depth 이상치 2건이 평균을 끌어올림).

**L2 (실제 도달):** perceive → world target=(est_xy, peg_top+hover) → DLS-IK(`ee_site`) → 최종 측정.
- `--n 20 --hover 0.06 --tol-xy 0.03 --tol-z 0.02` → **17/20 성공 (0.85) ≥ target 0.8, exit 0.**
- 성공 예: reach_xy 0.0–7 mm, hover_z 0.060(목표 0.06)에 정확.
- 실패 3건: i7/i9 = 단일픽셀 depth 이상치(est가 ~3.5 cm 빗나가 tol 직초과), i18 = peg 가 FOV 가장자리(미검출).

**증명된 것:** EGL depth 렌더 / `wrist_cam` 가 peg+hole+핑거팁(`ee_site`)을 봄 / 핀홀+extrinsics
좌표변환 / depth 역투영 / DLS-IK 도달 = perceive→3D→move 체인이 **실제 타겟 위로 이동**.

**다음 라운드 개선(M2 에서):**
- robustness: 단일 픽셀 depth → **마스크 영역 depth 중앙값**으로 (이상치 2건 제거 → 0.95+ 예상).
- 관찰 자세(R1): home 키프레임은 일부 영역만 봄. 작업영역 전체를 보는 observe pose 를 블록 기본값으로.
- 색 검출은 M1 한정(기하 증명용). 실사용은 SAM3(text/box) 마스크로 교체(M3).

## M2 (block 배선) — ✅ VERIFIED (2026-06-01)
`backend/api/routes/planner.py` `BLOCK_CONFIGS` + `_validate_plan_blocks` 분기 + `backend/api/process/planner_run.py`
`_run_visual_reach` + `_HANDLERS`. 핸들러: 관찰자세(옵션) → ROSProxy `/tutorial/wrist_rgbd` fetch →
타겟 마스크(SAM3 text/box if installed, else box, else 색상) → median-depth centroid → 3D 역투영 → `move_ee_to(타겟+hover)`.
검증: 백엔드 컨테이너에서 실제 데이터 경로 호출 → EST `[0.3057, 0.0808, 0.0381]` (큐브와 6mm).

## M3 (frontend) — ✅ VERIFIED (Playwright)
`PlannerPage.vue` visual_reach 폼(카메라 select + 라이브 `WebRtcVideo` + 텍스트 프롬프트 + 박스 + hover/duration/settle)
+ blockForm init/open/edit/save + 카드 subtitle/색상 + 블록 디테일 + ko/en i18n. DESIGN.md(no-CSS) 준수.
검증: Playwright로 폼 렌더 11필드 + 콘솔 에러 0. **함정**: 프론트 컨테이너는 `/opt/easytrainer/project/frontend`
(런타임)을 마운트하므로 호스트 편집 후 **반드시 `quick_apply`** 해야 반영됨(초기 빈 폼의 진짜 원인).

## M-E2E — ✅ VERIFIED (live tutorial, 실제 planner run)
`/tutorial:start` → wrist 센서(id 4) 시드 → API로 planner+group+visual_reach 블록 생성 → robot agent 기동 →
`:start_run`. 결과: EE home `[0.300,0.000,0.110]` → after `[0.3056,0.0805,0.0982]` (타겟 0.30/0.08/0.10).
**수평 ~5.5mm, 높이 ~2mm.** wrist 화면에 큐브가 정중앙 = EE가 큐브 위. 색상/텍스트프롬프트 두 경로 모두 통과.

## SAM3 — ✅ FUNCTIONAL (2026-06-02, 사용자 토큰으로 가중치 다운로드)

사용자가 모듈 UI에서 SAM3 설치 + HF 토큰 입력 후, 다음으로 **실제 동작 확정**:
- `sam3` pip 패키지가 backend 컨테이너에 없었음(모듈 install.sh가 `--break-system-packages` 없이 PEP668에
  막혀 `|| true`로 조용히 실패 + 컨테이너에 git 없음). → tarball 설치 + 누락 deps(iopath/timm/pycocotools/ftfy).
  torch 무결성 유지(2.12+cu130). **install.sh/module.json 수정(v0.1.3)으로 재설치 시 자동 해결.**
- **dtype 버그**: SAM3 혼합정밀도(bf16 backbone + f32 head) → `mat1/mat2 dtype mismatch`. runner.detect_one을
  `torch.autocast(bf16)`로 감싸 해결(모듈 runner.py, v0.1.2).
- **동작 검증**: "red cube" → 12983px, "white circle" → 38338px 정확 분할. 토큰으로 gated 가중치 자동 다운로드 OK.
- **"white plate" 처리**: 튜토리얼의 추상 흰 디스크를 SAM3가 "plate"가 아닌 "circle"로 인식 → 핸들러에
  **색+형태 프리미티브 프롬프트 확장 fallback** 추가("white plate" empty → "white circle" 매치, 로그로 투명).
- **E2E 라이브**: 사용자 블록(text "white plate") 실행 → SAM3→fallback "white circle" → target [0.312,-0.102,0.001]
  → EE **[0.3123,-0.1019,0.0608]** = 흰 접시(GT 0.30,-0.10) 위 hover. home(y=0)→접시(y=-0.10) 이동, 빨간 큐브와 구분. wrist 화면에 접시 정중앙(시각 확인). nightwatch UI: 폼 11필드+프롬프트값+라이브스트림+콘솔에러 0.

### (참고) 이전 상태 — 통합·설치·활성
확장 코드 런타임 설치(`/root/backend/extensions/sam3`) → `is_extension_installed: True`. visual_reach 핸들러가
text/box 프롬프트 시 SAM3 `detect_one` 시도 → 가중치/패키지 없으면 ModuleNotFoundError catch → box/색상 fallback
(검증: 로그 `SAM3 path unavailable, falling back` + EE 정상 이동). **실제 텍스트 분할 활성화 = gated
`facebook/sam3` 가중치 다운로드용 사용자 HuggingFace 토큰 1개**(module.json post_install credential `hf_token`).
토큰 입력 시 자동 동작 — 코드 변경 불필요.

## M4 (실물) — 설계만 (하드웨어 대기)
RealSense aligned-depth + hand-eye extrinsics + 실물 핑거팁 검출. 동일 핸들러에 `rgbd_service`만 실물 소스로 교체.
