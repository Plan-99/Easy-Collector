"""
Joint trajectory interpolator with scheduled waypoint queue.

DexUMI 의 PoseTrajectoryInterpolator (real_env/common/pose_trajectory_interpolator.py)
를 joint-space 로 단순화한 버전. EasyTrainer 의 interpolation_node 가 이 클래스를
사용해, 백엔드가 보낸 (target_joints, t_absolute) 스케줄 waypoint 들을 시간순으로
유지하면서 매 control tick 마다 현재 시각에 해당하는 보간된 joint 명령을 산출한다.

핵심 동작:
  - schedule_waypoint(joints, t_target): (시각, joints) 한 점을 큐 끝에 append.
    이미 큐에 있는 마지막 시각보다 작으면 무시 (역방향 스케줄 방지).
  - __call__(t): t 시각의 보간된 joints 반환. t < 첫 waypoint → 첫 값 hold.
    t > 마지막 waypoint → 마지막 값 hold. 사이는 linear interp.
  - trim_before(t): t 이전 waypoint 제거 (메모리 관리). 단, t 직전 한 점은
    interp 기준점으로 유지.

스레드 안전성은 호출자(interpolation_node) 가 lock 으로 처리한다.
"""

from __future__ import annotations

from typing import List, Optional

import numpy as np


class JointTrajectoryInterpolator:
    def __init__(self):
        # times: (N,) np.float64 absolute seconds (epoch 또는 monotonic — 호출자가 결정)
        # poses: (N, D) joint positions
        self._times: List[float] = []
        self._poses: List[np.ndarray] = []
        self._dim: Optional[int] = None
        self._names: Optional[List[str]] = None  # 마지막에 받은 joint names

    # ─── Properties ────────────────────────────────────────────────────────
    @property
    def empty(self) -> bool:
        return len(self._times) == 0

    @property
    def first_time(self) -> Optional[float]:
        return self._times[0] if self._times else None

    @property
    def last_time(self) -> Optional[float]:
        return self._times[-1] if self._times else None

    @property
    def names(self) -> Optional[List[str]]:
        return self._names

    @property
    def num_waypoints(self) -> int:
        return len(self._times)

    # ─── Mutation ──────────────────────────────────────────────────────────
    def schedule_waypoint(
        self,
        joints: np.ndarray,
        t_target: float,
        names: Optional[List[str]] = None,
    ) -> bool:
        """큐 끝에 (t_target, joints) 추가.

        Returns:
            True if added; False if rejected (e.g. t_target <= last_time).
        """
        joints = np.asarray(joints, dtype=np.float64).flatten()
        if self._dim is None:
            self._dim = joints.shape[0]
        elif joints.shape[0] != self._dim:
            return False

        # 역방향 / 중복 시각 거부 (단조 증가 유지)
        if self._times and t_target <= self._times[-1]:
            return False

        self._times.append(float(t_target))
        self._poses.append(joints)
        if names:
            self._names = list(names)
        return True

    def reset(self):
        self._times.clear()
        self._poses.clear()
        self._dim = None
        # names 는 유지 (last known) — caller 가 직접 reset 가능

    def trim_before(self, t: float):
        """t 이전 waypoint 들 제거. 단 t 직전 한 점은 interp 기준으로 유지."""
        if len(self._times) <= 1:
            return
        # t 이전의 마지막 idx 찾기
        keep_start = 0
        for i, ti in enumerate(self._times):
            if ti < t:
                keep_start = i
            else:
                break
        if keep_start > 0:
            self._times = self._times[keep_start:]
            self._poses = self._poses[keep_start:]

    # ─── Query ─────────────────────────────────────────────────────────────
    def __call__(self, t: float) -> Optional[np.ndarray]:
        """t 시각의 보간된 joints 반환. 큐 비어 있으면 None."""
        if not self._times:
            return None
        if len(self._times) == 1 or t <= self._times[0]:
            return self._poses[0].copy()
        if t >= self._times[-1]:
            return self._poses[-1].copy()

        # binary-search-free: 보통 큐 짧음 (수~수십개), linear 충분.
        for i in range(1, len(self._times)):
            if self._times[i] >= t:
                t0, t1 = self._times[i - 1], self._times[i]
                p0, p1 = self._poses[i - 1], self._poses[i]
                alpha = (t - t0) / (t1 - t0) if t1 > t0 else 1.0
                return p0 + alpha * (p1 - p0)
        return self._poses[-1].copy()
