# -*- coding: utf-8 -*-
"""dual_arm_assembly_test — keyboard / IK control verification (two robots).

Two independent role='single_arm' robots on separate topic prefixes, sharing
one MuJoCo sim, combined via an Assembly. Verifies the multi-robot 양팔 path:

  1. POST /api/dual_arm_assembly_test:start -> seed + launch, wait for topics
  2. subscribe BOTH robots (left_arm on /da_asm_left, right_arm on /da_asm_right)
  3. drive each robot's single 'ee' via move_robot_ee_delta and assert:
       - the targeted arm's EE + joints move
       - the OTHER robot stays put (independent topics => no cross-talk)

Assumes backend + ROS2 bridge are up. See backend/tests/README.md.

    python -m backend.tests.sim_dual_arm_assembly.test_keyboard_ik
    python -m backend.tests.sim_dual_arm_assembly.test_keyboard_ik --keep
"""
import argparse
import sys

from ..helpers.api import Backend, start_sim_test, stop_sim_test, subscribe_robot
from ..helpers.control import (
    RobotLink, pulse_ee_delta, max_abs_pos_change, joint_subset_change,
)

ENV = 'dual_arm_assembly_test'
ARM_IDX = [0, 1, 2, 3, 4, 5]   # each robot: [j1..j6, gripper]
MOVE_THRESH = 0.01


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--backend', default='http://127.0.0.1:5000')
    ap.add_argument('--keep', action='store_true')
    args = ap.parse_args()

    api = Backend(args.backend)
    results = []

    def check(name, ok, detail=''):
        results.append((name, ok, detail))
        print(f'  [{"PASS" if ok else "FAIL"}] {name}  {detail}', flush=True)

    print('=== dual_arm_assembly_test keyboard/IK control test ===', flush=True)
    start = start_sim_test(api, ENV, show_viewer=False)
    left_id = start['robot_ids']['left_arm']
    right_id = start['robot_ids']['right_arm']
    subscribe_robot(api, left_id)
    subscribe_robot(api, right_id)

    left = RobotLink(args.backend, left_id).connect()
    right = RobotLink(args.backend, right_id).connect()
    try:
        ljs0, lee0 = left.wait_for_state(timeout=20)
        rjs0, ree0 = right.wait_for_state(timeout=20)
        check('left robot: 7 joints + ee', ljs0 is not None and len(ljs0) == 7 and bool(lee0) and 'ee' in lee0,
              f'(len={len(ljs0) if ljs0 else None}, ee={list(lee0.keys()) if lee0 else None})')
        check('right robot: 7 joints + ee', rjs0 is not None and len(rjs0) == 7 and bool(ree0) and 'ee' in ree0,
              f'(len={len(rjs0) if rjs0 else None}, ee={list(ree0.keys()) if ree0 else None})')

        # --- Drive LEFT robot only --------------------------------------
        ljsA, _ = left.snapshot()
        rjsA, _ = right.snapshot()
        lb, la = pulse_ee_delta(left, 'ee', axis=2, step=0.01, n=15)
        ljsB, _ = left.snapshot()
        rjsB, _ = right.snapshot()
        l_ee_moved = max_abs_pos_change(lb, la)
        l_jchg = joint_subset_change(ljsA, ljsB, ARM_IDX)
        r_jchg_during_left = joint_subset_change(rjsA, rjsB, ARM_IDX)
        check('LEFT ee delta moves left robot EE', l_ee_moved > MOVE_THRESH, f'(|Δee|={l_ee_moved:.3f})')
        check('LEFT ee delta moves left robot joints', l_jchg > MOVE_THRESH, f'(|Δq|={l_jchg:.3f})')
        check('LEFT ee delta leaves right robot still', r_jchg_during_left < MOVE_THRESH,
              f'(|Δq_right|={r_jchg_during_left:.3f})')

        # --- Drive RIGHT robot only -------------------------------------
        ljsA, _ = left.snapshot()
        rjsA, _ = right.snapshot()
        rb, ra = pulse_ee_delta(right, 'ee', axis=2, step=0.01, n=15)
        ljsB, _ = left.snapshot()
        rjsB, _ = right.snapshot()
        r_ee_moved = max_abs_pos_change(rb, ra)
        r_jchg = joint_subset_change(rjsA, rjsB, ARM_IDX)
        l_jchg_during_right = joint_subset_change(ljsA, ljsB, ARM_IDX)
        check('RIGHT ee delta moves right robot EE', r_ee_moved > MOVE_THRESH, f'(|Δee|={r_ee_moved:.3f})')
        check('RIGHT ee delta moves right robot joints', r_jchg > MOVE_THRESH, f'(|Δq|={r_jchg:.3f})')
        check('RIGHT ee delta leaves left robot still', l_jchg_during_right < MOVE_THRESH,
              f'(|Δq_left|={l_jchg_during_right:.3f})')
    finally:
        left.close()
        right.close()
        if not args.keep:
            stop_sim_test(api, ENV)

    passed = sum(1 for _, ok, _ in results if ok)
    total = len(results)
    print(f'\n=== dual_arm_assembly_test: {passed}/{total} checks passed ===', flush=True)
    sys.exit(0 if passed == total else 1)


if __name__ == '__main__':
    main()
