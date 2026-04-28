# Copyright (c) 2022-2026, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

import isaaclab.sim as sim_utils
from isaaclab.assets import DeformableObjectCfg
from isaaclab.managers import EventTermCfg as EventTerm
from isaaclab.managers import SceneEntityCfg
from isaaclab.utils import configclass

import isaaclab_tasks.manager_based.manipulation.lift.mdp as mdp

from .ik_rel_env_cfg import FrankaCubeLiftEnvCfg


@configclass
class FrankaDeformableCubeLiftIKRelEnvCfg(FrankaCubeLiftEnvCfg):
    def __post_init__(self):
        super().__post_init__()

        # Deformable cube
        self.scene.object = DeformableObjectCfg(
            prim_path="{ENV_REGEX_NS}/Object",
            init_state=DeformableObjectCfg.InitialStateCfg(pos=(0.5, 0, 0.055), rot=(1, 0, 0, 0)),
            spawn=sim_utils.MeshCuboidCfg(
                size=(0.08, 0.08, 0.08),
                deformable_props=sim_utils.DeformableBodyPropertiesCfg(
                    kinematic_enabled=False,
                    self_collision=True,
                    simulation_hexahedral_resolution=10,
                    solver_position_iteration_count=20,
                    rest_offset=0.0,
                ),
                visual_material=sim_utils.PreviewSurfaceCfg(
                    diffuse_color=(0.8, 0.2, 0.2),
                ),
                physics_material=sim_utils.DeformableBodyMaterialCfg(
                    youngs_modulus=100000.0,
                    poissons_ratio=0.4,
                    dynamic_friction=0.5,
                    elasticity_damping=0.01,
                ),
            ),
        )

        # Soften gripper for deformable handling
        self.scene.robot.actuators["panda_hand"].effort_limit_sim = 50.0
        self.scene.robot.actuators["panda_hand"].stiffness = 40.0
        self.scene.robot.actuators["panda_hand"].damping = 10.0

        # Disable replicate physics (doesn't work with deformable objects)
        self.scene.replicate_physics = False

        # Use deformable-compatible reset event
        self.events.reset_object_position = EventTerm(
            func=mdp.reset_nodal_state_uniform,
            mode="reset",
            params={
                "position_range": {"x": (-0.1, 0.1), "y": (-0.25, 0.25), "z": (0.0, 0.0)},
                "velocity_range": {},
                "asset_cfg": SceneEntityCfg("object"),
            },
        )

        # Disable terms incompatible with deformable objects
        self.terminations.object_dropping = None
        self.rewards.reaching_object = None
        self.rewards.lifting_object = None
        self.rewards.object_goal_tracking = None
        self.rewards.object_goal_tracking_fine_grained = None
        self.observations.policy.object_position = None
