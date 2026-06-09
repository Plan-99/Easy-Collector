# Verify Oracle — `visual_reach` (MuJoCo ground-truth)

> "타겟으로 잘 가는가"를 사람 눈이 아니라 **결정적 측정**으로 판정. memory `feedback_success_check`
> (성공은 scene state 로). 생성≠평가: 구현이 "됐다"고 말해도 이 오라클만 믿는다.

## 두 층위의 검증

### L1 — perception 정확도 (블록 이동 전, 추정치 vs ground-truth)
sim 은 타겟 body 와 핑거팁(`ee_site`) world XYZ 를 안다. perception 파이프라인의 추정치를 직접 대조:
- `‖XYZ_target_est − XYZ_target_gt‖ < 0.02 m`
- `‖XYZ_eetip_est  − XYZ_eetip_gt‖ < 0.02 m`
L1 이 깨지면 카메라 모델(intrinsics/extrinsics/부호)이 틀린 것 → IK 이동 전에 잡는다.

### L2 — 도달 성공 (블록 실행 후, EE 최종 pose)
블록 실행 완료 후 `ee_site` 의 ground-truth 로:
- **수평** `‖ee_xy − target_xy‖ < TOL_XY` (기본 0.03 m)
- **높이** `|ee_z − (target_top_z + HOVER)| < TOL_Z` (HOVER 0.06, TOL_Z 0.02 m)

## 통계 (flaky 방지)
- 타겟을 작업영역 내 **랜덤 N 위치**(기본 N=20)에 배치하고 각 1회 실행.
- 성공률 = L2 통과 수 / N. **합격선: ≥ TARGET_RATE(기본 0.8).**
- 실패 케이스는 (perception 실패 / IK 미수렴 / depth NaN) 로 분류 기록.

## 러너 (M1 에서 구현 예정)
`vision-reach/verify/run_oracle.py` (헤드리스 MuJoCo, ros2 컨테이너 안에서 실행):
```
for i in range(N):
    randomize_target()
    obs_rgb, obs_depth = wrist_camera.render()
    tgt_xyz_est = perceive(obs_rgb, obs_depth, prompt_or_box)   # 신규 perception
    tip_xyz_est = fingertip(obs_depth or FK)                    # 신규
    ee_target = tgt_xyz_est + [0,0,HOVER]
    mujoco_ik_move(ee_target)                                   # _MujocoIK 재사용
    record(L1: est vs gt, L2: ee_site vs target gt)
print {N, success_rate, l1_pass, failures_by_reason}
```
종료코드: 성공률 ≥ TARGET_RATE 면 0, 아니면 1 (CI/슬라이스 게이트로 사용).

## 사용 (M1 검증)
```bash
EASYTRAINER_ROOT="$(git rev-parse --show-toplevel)"
docker exec easytrainer_ros2 bash -lc '
  source /opt/ros/humble/setup.bash && source /root/ros2_ws/install/setup.bash &&
  export ROS_DOMAIN_ID=1 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp &&
  python3 /opt/easytrainer/project/vision-reach/verify/run_oracle.py --n 20 --hover 0.06'
```
> 스크린샷 시각 확인도 병행(wrist RGB + 추정 centroid/핑거팁 오버레이 PNG 저장 → Read 로 눈으로 확인).
