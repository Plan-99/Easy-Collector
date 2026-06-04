# -*- coding: utf-8 -*-
"""dual_arm_assembly_test API routes — two single_arm robots on separate
topics sharing one MuJoCo sim, combined via an Assembly.

Thin wrapper: SPEC lives in configs/dual_arm_assembly_defaults.py; all logic
is the shared sim_test_common factory.
"""
from ...configs.dual_arm_assembly_defaults import SPEC
from .sim_test_common import make_sim_test_blueprint

dual_arm_assembly_test_bp = make_sim_test_blueprint(SPEC)
