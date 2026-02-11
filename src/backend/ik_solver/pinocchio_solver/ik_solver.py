# -*- coding: utf-8 -*-
#
# Copyright 2024 Pukyung National University
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import numpy as np
import pinocchio as pin
from pink import solve_ik
from pink.tasks import FrameTask
from pink.configuration import Configuration
import logging_mp
logger_mp = logging_mp.get_logger(__name__)

# ---------------------------------------------------------------------------
# Helper Functions (Largely unchanged, as they are used for pose representation)
# ---------------------------------------------------------------------------

def xyzrpy_to_se3(xyzrpy):
    """
    Converts a list of [x, y, z, roll, pitch, yaw] to a pin.SE3 object.
    """
    xyz = np.array(xyzrpy[:3])
    rpy = np.array(xyzrpy[3:])
    rotation_matrix = pin.rpy.rpyToMatrix(rpy)
    return pin.SE3(rotation_matrix, xyz)

def se3_to_xyzrpy(se3_matrix):
    """
    Converts a pin.SE3 object to a list of [x, y, z, roll, pitch, yaw].
    """
    xyz = se3_matrix.translation
    rotation_matrix = se3_matrix.rotation
    rpy = pin.rpy.matrixToRpy(rotation_matrix)
    return np.concatenate([xyz, rpy]).tolist()

def se3_to_xyzaxayaz(se3_matrix):
    """
    Converts a pin.SE3 object to a list of [x, y, z, ax, ay, az] (rotation vector).
    """
    xyz = se3_matrix.translation
    rotation_matrix = se3_matrix.rotation
    axayaz = pin.log3(rotation_matrix)
    return np.concatenate([xyz, axayaz]).tolist()

def xyzaxayaz_to_se3(xyzaxayaz):
    """
    Converts a list of [x, y, z, ax, ay, az] (rotation vector) to a pin.SE3 object.
    """
    xyz = np.array(xyzaxayaz[:3])
    axayaz = np.array(xyzaxayaz[3:])
    rotation_matrix = pin.exp3(axayaz)
    return pin.SE3(rotation_matrix, xyz)

# ---------------------------------------------------------------------------
# ## 🚀 Main Class: IK_Solver (using pink)
# ---------------------------------------------------------------------------
class IK_Solver:
    def __init__(self, 
                 urdf_path, 
                 joints_to_lock, 
                 ee_definitions,
                 dt=0.01, # Timestep for IK integration
                 solver='proxqp',
                 **kwargs): # Absorb unused parameters
        
        np.set_printoptions(precision=5, suppress=True, linewidth=200)
        
        # 1. Load and reduce the robot model using Pinocchio
        full_model = pin.buildModelFromUrdf(urdf_path)
        joints_to_lock_ids = [full_model.getJointId(jname) for jname in joints_to_lock if full_model.existJointName(jname)]
        q_reference = pin.neutral(full_model) # Use neutral configuration as reference
        
        self.model = pin.buildReducedModel(full_model, joints_to_lock_ids, q_reference)
        self.data = self.model.createData()
        
        self.nq = self.model.nq
        self.nv = self.model.nv
        self.dt = dt
        self.solver = solver

        # 2. Dynamically add EE frames if they don't exist
        self.ee_names = []
        self.ee_frame_names = {}
        for name, parent_or_existing_frame, offset in ee_definitions:
            self.ee_names.append(name)
            frame_name_to_use = name # Use the task name as the frame name by default

            if offset is not None:
                # This logic creates a new frame in the model
                parent_joint_name = parent_or_existing_frame
                if not self.model.existJointName(parent_joint_name):
                    raise ValueError(f"Joint '{parent_joint_name}' not found.")
                parent_joint_id = self.model.getJointId(parent_joint_name)
                
                new_frame = pin.Frame(frame_name_to_use, parent_joint_id, pin.SE3(np.eye(3), np.array(offset)), pin.FrameType.OP_FRAME)
                
                if not self.model.existFrame(frame_name_to_use):
                    self.model.addFrame(new_frame)
                
            else:
                # Use an existing frame
                frame_name_to_use = parent_or_existing_frame
                if not self.model.existFrame(frame_name_to_use):
                    raise ValueError(f"Frame '{frame_name_to_use}' not found.")

            self.ee_frame_names[name] = frame_name_to_use

        # Re-create data object after model modifications
        self.data = self.model.createData()

        # 3. Create pink FrameTasks for each end-effector
        self.tasks = {
            name: FrameTask(
                self.ee_frame_names[name],
                position_cost=1.0, # High cost for position
                orientation_cost=1.0 # High cost for orientation
            ) for name in self.ee_names
        }

        # 4. Set initial state
        self.q = pin.neutral(self.model)

    def solve_ik(self, target_poses: dict, current_lr_arm_motor_q=None, current_lr_arm_motor_dq=None):
        """
        Solves the inverse kinematics problem using pink.
        
        :param target_poses: Dictionary mapping EE names to their target poses in xyzaxayaz format.
        :param current_lr_arm_motor_q: Current joint configuration (optional).
        :return: Tuple of (solved joint positions, feedforward torques).
        """
        if current_lr_arm_motor_q is not None:
            self.q = np.array(current_lr_arm_motor_q)

        # 1. Update task targets
        for name, target_pose_vec in target_poses.items():
            if name in self.tasks:
                target_pose_se3 = xyzaxayaz_to_se3(target_pose_vec)
                self.tasks[name].set_target(target_pose_se3)

        # 2. Create configuration and solve IK
        configuration = Configuration(self.model, self.data, self.q)
        
        # The solve_ik function computes the velocity `v` needed to move towards the targets
        try:
            velocity = solve_ik(configuration, self.tasks.values(), self.dt, solver=self.solver)
        except Exception as e:
            logger_mp.error(f"ERROR in pink.solve_ik: {e}")
            # Return current state if solver fails
            velocity = np.zeros(self.nv)

        # 3. Integrate velocity to get the new joint configuration
        self.q = configuration.integrate(velocity, self.dt)
        
        # 4. Compute feedforward torques (optional, but good to have)
        # We use the computed velocity and assume zero acceleration for RNEA
        tau_ff = pin.rnea(self.model, self.data, self.q, velocity, np.zeros(self.nv))
        
        return self.q, tau_ff

    def get_ee_position(self, q_numeric):
        """
        Computes forward kinematics for a given joint configuration `q`.
        Returns a dictionary of EE poses in xyzaxayaz format.
        """
        pin.forwardKinematics(self.model, self.data, np.array(q_numeric))
        pin.updateFramePlacements(self.model, self.data)
        
        poses = {}
        for name, frame_name in self.ee_frame_names.items():
            frame_id = self.model.getFrameId(frame_name)
            se3_pose = self.data.oMf[frame_id]
            poses[name] = se3_to_xyzaxayaz(se3_pose)
            
        return poses

    def reset_state(self, current_q):
        """
        Resets the internal state of the solver to a given joint configuration.
        """
        self.q = np.array(current_q).flatten()

    def compute_delta_target(self, name, current_q, delta_xyzaxayaz, frame='global'):
        """
        Computes the next target pose based on a delta from the current pose.
        """
        # 1. FK to get current pose
        pin.forwardKinematics(self.model, self.data, current_q)
        pin.updateFramePlacements(self.model, self.data)
        
        frame_id = self.model.getFrameId(self.ee_frame_names[name])
        current_oMf = self.data.oMf[frame_id]
        
        # 2. Calculate new pose based on delta
        d_xyz = np.array(delta_xyzaxayaz[:3])
        d_axayaz = np.array(delta_xyzaxayaz[3:])
        delta_R = pin.exp3(d_axayaz)
        
        if frame == 'global':
            new_R = delta_R @ current_oMf.rotation
        else: # local frame
            new_R = current_oMf.rotation @ delta_R
            
        new_t = current_oMf.translation + d_xyz
        
        new_se3 = pin.SE3(new_R, new_t)
        return se3_to_xyzaxayaz(new_se3)