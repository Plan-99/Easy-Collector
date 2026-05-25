"""Joint-space trajectory interpolator with a scheduled-waypoint queue.

Mirrors EasyTrainer's in-container
``ros2_bridge.joint_trajectory_interpolator.JointTrajectoryInterpolator``.

The interpolation node holds one of these per robot. Callers push
``(t_absolute, target_joints)`` waypoints; the node's high-rate timer asks
``__call__(t_now)`` for the linearly-interpolated joint vector at the current
wall-clock time and writes it to the robot. Time bases must be consistent —
this module uses whatever ``t`` callers pass in (the rest of the bundle uses
``time.time()`` epoch seconds).

Thread safety is the caller's job: the node locks around mutations / queries.
"""
from __future__ import annotations

from typing import List, Optional

import numpy as np


class JointTrajectoryInterpolator:
    def __init__(self):
        self._times: List[float] = []
        self._poses: List[np.ndarray] = []
        self._dim: Optional[int] = None
        self._names: Optional[List[str]] = None  # last-seen joint names

    # ── Properties ──────────────────────────────────────────────────────────
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

    # ── Mutation ────────────────────────────────────────────────────────────
    def schedule_waypoint(
        self,
        joints: np.ndarray,
        t_target: float,
        names: Optional[List[str]] = None,
    ) -> bool:
        """Append a waypoint. Rejects out-of-order timestamps and dim mismatches."""
        joints = np.asarray(joints, dtype=np.float64).flatten()
        if self._dim is None:
            self._dim = joints.shape[0]
        elif joints.shape[0] != self._dim:
            return False

        # Reject non-monotonic / duplicate timestamps.
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
        # Keep `_names` — caller can clear it explicitly if needed.

    def trim_before(self, t: float):
        """Drop waypoints older than ``t``, keeping the most recent prior point
        as the interpolation anchor."""
        if len(self._times) <= 1:
            return
        keep_start = 0
        for i, ti in enumerate(self._times):
            if ti < t:
                keep_start = i
            else:
                break
        if keep_start > 0:
            self._times = self._times[keep_start:]
            self._poses = self._poses[keep_start:]

    # ── Query ───────────────────────────────────────────────────────────────
    def __call__(self, t: float) -> Optional[np.ndarray]:
        """Return the interpolated joint vector at time ``t``. Holds the first
        / last value when ``t`` falls outside the queue's time range."""
        if not self._times:
            return None
        if len(self._times) == 1 or t <= self._times[0]:
            return self._poses[0].copy()
        if t >= self._times[-1]:
            return self._poses[-1].copy()

        # Queue is typically short (handful of waypoints); linear scan beats
        # the overhead of binary search.
        for i in range(1, len(self._times)):
            if self._times[i] >= t:
                t0, t1 = self._times[i - 1], self._times[i]
                p0, p1 = self._poses[i - 1], self._poses[i]
                alpha = (t - t0) / (t1 - t0) if t1 > t0 else 1.0
                return p0 + alpha * (p1 - p0)
        return self._poses[-1].copy()
