# -*- coding: utf-8 -*-
"""dual_arm_test API routes — single-topic role='dual_arm' MuJoCo world.

Thin wrapper: the SPEC lives in configs/dual_arm_defaults.py and all the
seeding/control logic is the shared sim_test_common factory.
"""
from ...configs.dual_arm_defaults import SPEC
from .sim_test_common import make_sim_test_blueprint

dual_arm_test_bp = make_sim_test_blueprint(SPEC)
