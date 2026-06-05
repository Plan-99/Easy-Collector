# -*- coding: utf-8 -*-
"""dual_arm_test — keyboard / IK control verification (single-topic dual_arm).

Verifies the humanoid-style 양팔 control path against the bundled MuJoCo
dual-arm world:

  1. POST /api/dual_arm_test:start  -> seed rows + launch sim, wait for topics
  2. subscribe the role='dual_arm' robot (14 joints on one topic)
  3. drive each end-effector via move_robot_ee_delta (the SAME socket event
     keyboard teleop uses) and assert:
       - L_ee delta moves the LEFT EE and the left arm joints, not the right
       - R_ee delta moves the RIGHT EE and the right arm joints, not the left
  4. drive a single joint via move_robot_joint_delta and assert it moves

This is the authoritative check that dual-arm keyboard control + dual-EE IK
are wired correctly. RobotPendant (UI) IK is covered separately by
test_pendant.js (Playwright).

Assumes backend + ROS2 bridge are up. See backend/tests/README.md.

    python -m backend.tests.sim_dual_arm.test_keyboard_ik
    python -m backend.tests.sim_dual_arm.test_keyboard_ik --keep   # leave sim running
"""
import argparse
import sys

from ..helpers.api import Backend, start_sim_test, stop_sim_test, subscribe_robot
from ..helpers.control import (
    RobotLink, pulse_ee_delta, max_abs_pos_change, joint_subset_change,
)

ENV = 'dual_arm_test'
# 14-joint layout: [L1..L6, Lgrip, R1..R6, Rgrip]
LEFT_ARM_IDX = [0, 1, 2, 3, 4, 5]
RIGHT_ARM_IDX = [7, 8, 9, 10, 11, 12]
MOVE_THRESH = 0.01   # 1 cm EE / 0.01 rad joint counts as "moved"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--backend', default='http://127.0.0.1:5000')
    ap.add_argument('--keep', action='store_true', help='leave sim running on exit')
    args = ap.parse_args()

    api = Backend(args.backend)
    results = []

    def check(name, ok, detail=''):
        results.append((name, ok, detail))
        print(f'  [{"PASS" if ok else "FAIL"}] {name}  {detail}', flush=True)

    print(f'=== dual_arm_test keyboard/IK control test ===', flush=True)
    start = start_sim_test(api, ENV, show_viewer=False)
    robot_id = start['robot_ids']['dual_arm']
    subscribe_robot(api, robot_id)

    link = RobotLink(args.backend, robot_id).connect()
    try:
        js0, ee0 = link.wait_for_state(timeout=20)
        check('joint_states has 14 entries', js0 is not None and len(js0) == 14,
              f'(len={len(js0) if js0 else None})')
        check('ee_pos exposes L_ee and R_ee',
              bool(ee0) and 'L_ee' in ee0 and 'R_ee' in ee0,
              f'(keys={list(ee0.keys()) if ee0 else None})')

        # --- Left EE: +z ------------------------------------------------
        jsA, _ = link.snapshot()
        lb, la = pulse_ee_delta(link, 'L_ee', axis=2, step=0.01, n=15)
        jsB, _ = link.snapshot()
        l_ee_moved = max_abs_pos_change(lb, la)
        left_jchg = joint_subset_change(jsA, jsB, LEFT_ARM_IDX)
        right_jchg = joint_subset_change(jsA, jsB, RIGHT_ARM_IDX)
        check('L_ee delta moves LEFT end-effector', l_ee_moved > MOVE_THRESH,
              f'(|Δee|={l_ee_moved:.3f})')
        check('L_ee delta moves left arm joints', left_jchg > MOVE_THRESH,
              f'(|Δq_left|={left_jchg:.3f})')
        check('L_ee delta does NOT drive right arm', right_jchg < left_jchg,
              f'(|Δq_right|={right_jchg:.3f} < |Δq_left|={left_jchg:.3f})')

        # --- Right EE: +z -----------------------------------------------
        jsA, _ = link.snapshot()
        rb, ra = pulse_ee_delta(link, 'R_ee', axis=2, step=0.01, n=15)
        jsB, _ = link.snapshot()
        r_ee_moved = max_abs_pos_change(rb, ra)
        left_jchg = joint_subset_change(jsA, jsB, LEFT_ARM_IDX)
        right_jchg = joint_subset_change(jsA, jsB, RIGHT_ARM_IDX)
        check('R_ee delta moves RIGHT end-effector', r_ee_moved > MOVE_THRESH,
              f'(|Δee|={r_ee_moved:.3f})')
        check('R_ee delta moves right arm joints', right_jchg > MOVE_THRESH,
              f'(|Δq_right|={right_jchg:.3f})')
        check('R_ee delta does NOT drive left arm', left_jchg < right_jchg,
              f'(|Δq_left|={left_jchg:.3f} < |Δq_right|={right_jchg:.3f})')

        # --- Joint delta on a single left joint -------------------------
        jsA, _ = link.wait_for_state()
        delta = [0.0] * 14
        delta[0] = 0.15  # left_joint1
        for _ in range(8):
            link.send_joint_delta(delta)
            import time as _t
            _t.sleep(0.1)
        import time as _t
        _t.sleep(0.6)
        jsB, _ = link.snapshot()
        j0chg = abs(jsB[0] - jsA[0]) if jsB and jsA else 0.0
        check('move_robot_joint_delta moves left_joint1', j0chg > MOVE_THRESH,
              f'(|Δq0|={j0chg:.3f})')
    finally:
        link.close()
        if not args.keep:
            stop_sim_test(api, ENV)

    passed = sum(1 for _, ok, _ in results if ok)
    total = len(results)
    print(f'\n=== dual_arm_test: {passed}/{total} checks passed ===', flush=True)
    sys.exit(0 if passed == total else 1)


if __name__ == '__main__':
    main()
