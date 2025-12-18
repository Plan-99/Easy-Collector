import casadi                                                                       
import meshcat.geometry as mg
import numpy as np
import pinocchio as pin                             
import time
from pinocchio import casadi as cpin    
from pinocchio.visualize import MeshcatVisualizer   
import os
import sys
import logging_mp
logger_mp = logging_mp.get_logger(__name__)
parent2_dir = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.append(parent2_dir)


def xyzrpy_to_se3(xyzrpy):
    """
    (x, y, z, roll, pitch, yaw) 형식의 리스트를 SE3 객체로 변환합니다.
    """
    xyz = np.array(xyzrpy[:3])
    rpy = np.array(xyzrpy[3:])
    rotation_matrix = pin.rpy.rpyToMatrix(rpy)
    se3_pose = pin.SE3(rotation_matrix, xyz)
    return se3_pose.homogeneous


def se3_to_xyzrpy(se3_matrix):
    """
    SE3 객체를 (x, y, z, roll, pitch, yaw) 형식의 리스트로 변환합니다.
    """
    xyz = se3_matrix.translation
    rotation_matrix = se3_matrix.rotation
    rpy = pin.rpy.matrixToRpy(rotation_matrix)
    return np.concatenate([xyz, rpy]).tolist()


# ---------------------------------------------------------------------------
# 헬퍼 클래스 (변경 없음)
# ---------------------------------------------------------------------------
class WeightedMovingFilter:
    def __init__(self, weights, data_size = 14):
        self._window_size = len(weights)
        self._weights = np.array(weights)
        assert np.isclose(np.sum(self._weights), 1.0), "[WeightedMovingFilter] the sum of weights list must be 1.0!"
        self._data_size = data_size
        self._filtered_data = np.zeros(self._data_size)
        self._data_queue = []

    def _apply_filter(self):
        if len(self._data_queue) < self._window_size:
            return self._data_queue[-1]

        data_array = np.array(self._data_queue)
        temp_filtered_data = np.zeros(self._data_size)
        for i in range(self._data_size):
            temp_filtered_data[i] = np.convolve(data_array[:, i], self._weights, mode='valid')[-1]
        
        return temp_filtered_data

    def add_data(self, new_data):
        assert len(new_data) == self._data_size

        if len(self._data_queue) > 0 and np.array_equal(new_data, self._data_queue[-1]):
            return  # skip duplicate data
        
        if len(self._data_queue) >= self._window_size:
            self._data_queue.pop(0)

        self._data_queue.append(new_data)
        self._filtered_data = self._apply_filter()

    @property
    def filtered_data(self):
        return self._filtered_data

# ---------------------------------------------------------------------------
# ## 🚀 상위 클래스: IK_Solver (General-Purpose)
# ---------------------------------------------------------------------------
class IK_Solver:
    def __init__(self, 
                 urdf_path, 
                 package_dir, 
                 joints_to_lock, 
                 ee_definitions, # <--- 변경: l_ee_def, r_ee_def 대신 리스트 사용
                 cost_weights,
                 use_scaling=False,
                 Visualization=False):
        
        np.set_printoptions(precision=5, suppress=True, linewidth=200)
        self.Visualization = Visualization
        self.use_scaling = use_scaling 
        self.ee_names = []
        self.ee_ids = {}
        self.ee_params = {}     # 최적화용 파라미터 (목표치)
        self.ee_sym_vars = {}   # 심볼릭 변수 (수식용)

        # 1. 로봇 로드
        self.robot = pin.RobotWrapper.BuildFromURDF(urdf_path, package_dir)

        # 2. 모델 축소
        self.reduced_robot = self.robot.buildReducedRobot(
            list_of_joints_to_lock=joints_to_lock,
            reference_configuration=np.array([0.0] * self.robot.model.nq),
        )

        # 3. EE 프레임 동적 추가
        # ee_definitions 형식: [ ('L_ee', 'joint_name', offset_array), 
        #                      ('R_ee', 'joint_name', offset_array) ]
        for name, parent_or_existing_frame, offset in ee_definitions:
            self.ee_names.append(name)
            
            if offset is not None:
                # --- 1. '새 프레임 추가' 로직 (기존과 동일) ---
                parent_joint_name = parent_or_existing_frame
                logger_mp.debug(f"Adding new EE frame: '{name}' relative to '{parent_joint_name}'")
                
                self.reduced_robot.model.addFrame(
                    pin.Frame(name,
                              self.reduced_robot.model.getJointId(parent_joint_name),
                              pin.SE3(np.eye(3), offset),
                              pin.FrameType.OP_FRAME)
                )
                self.ee_ids[name] = self.reduced_robot.model.getFrameId(name)
            
            else:
                # --- 2. '기존 프레임 사용' 로직 (새 로직) ---
                existing_frame_name = parent_or_existing_frame
                logger_mp.debug(f"Using existing URDF frame: '{existing_frame_name}' as EE '{name}'")

                # reduced_robot 모델에 해당 프레임이 존재하는지 확인
                if not self.reduced_robot.model.existFrame(existing_frame_name):
                    raise ValueError(f"'{existing_frame_name}' 프레임이 reduced_robot.model에 존재하지 않습니다. "
                                     "URDF나 buildReducedRobot 로직을 확인하세요.")
                
                # 'name' (예: 'L_ee')을 키로, 'existing_frame_name' (예: 'left_palm_center')의 ID를 저장
                self.ee_ids[name] = self.reduced_robot.model.getFrameId(existing_frame_name)


        self.reduced_robot.data = self.reduced_robot.model.createData()

        # 4. Casadi 모델 생성
        self.cmodel = cpin.Model(self.reduced_robot.model)
        self.cdata = self.cmodel.createData()

        # 5. 심볼릭 변수 생성
        self.cq = casadi.SX.sym("q", self.reduced_robot.model.nq, 1) 
        for name in self.ee_names:
            self.ee_sym_vars[name] = casadi.SX.sym(f"tf_{name}", 4, 4)
        
        cpin.framesForwardKinematics(self.cmodel, self.cdata, self.cq)

        # 6. 오차 함수 동적 정의
        trans_errors = []
        rot_errors = []
        sym_vars_list = [self.cq] + list(self.ee_sym_vars.values())

        for name in self.ee_names:
            ee_id = self.ee_ids[name]
            sym_var = self.ee_sym_vars[name]
            
            trans_error = self.cdata.oMf[ee_id].translation - sym_var[:3, 3]
            rot_error = cpin.log3(self.cdata.oMf[ee_id].rotation @ sym_var[:3, :3].T)
            
            trans_errors.append(trans_error)
            rot_errors.append(rot_error)

        self.translational_error = casadi.Function(
            "translational_error",
            sym_vars_list,
            [casadi.vertcat(*trans_errors)]
        )
        self.rotational_error = casadi.Function(
            "rotational_error",
            sym_vars_list,
            [casadi.vertcat(*rot_errors)]
        )

        # 7. 최적화 문제 정의
        self.opti = casadi.Opti()
        self.var_q = self.opti.variable(self.reduced_robot.model.nq)
        self.var_q_last = self.opti.parameter(self.reduced_robot.model.nq)
        
        for name in self.ee_names:
            self.ee_params[name] = self.opti.parameter(4, 4)

        param_list = [self.var_q] + list(self.ee_params.values())

        self.translational_cost = casadi.sumsqr(self.translational_error(*param_list))
        self.rotation_cost = casadi.sumsqr(self.rotational_error(*param_list))
        self.regularization_cost = casadi.sumsqr(self.var_q)
        self.smooth_cost = casadi.sumsqr(self.var_q - self.var_q_last)

        # 8. 제약 조건 및 비용 함수 설정
        self.opti.subject_to(self.opti.bounded(
            self.reduced_robot.model.lowerPositionLimit,
            self.var_q,
            self.reduced_robot.model.upperPositionLimit)
        )
        w = cost_weights
        self.opti.minimize(
            w['trans'] * self.translational_cost + 
            w['rot'] * self.rotation_cost + 
            w['reg'] * self.regularization_cost + 
            w['smooth'] * self.smooth_cost
        )

        # 9. 솔버 설정
        opts = { 'ipopt':{ 'print_level':0, 'max_iter':50, 'tol':1e-6 }, 'print_time':False, 'calc_lam_p':False }
        self.opti.solver("ipopt", opts)

        # 10. 필터 및 시각화 초기화
        self.init_data = np.zeros(self.reduced_robot.model.nq)
        self.smooth_filter = WeightedMovingFilter(np.array([0.4, 0.3, 0.2, 0.1]), self.reduced_robot.model.nq)
        self.vis = None

        if self.Visualization:
            self.vis = MeshcatVisualizer(self.reduced_robot.model, self.reduced_robot.collision_model, self.reduced_robot.visual_model)
            self.vis.initViewer(open=True) 
            self.vis.loadViewerModel("pinocchio") 
            self.vis.displayFrames(True, frame_ids=list(self.ee_ids.values()), axis_length = 0.15, axis_width = 5)
            self.vis.display(pin.neutral(self.reduced_robot.model))

            frame_viz_names = [f'{name}_target' for name in self.ee_names]
            FRAME_AXIS_POSITIONS = (np.array([[0, 0, 0], [1, 0, 0],[0, 0, 0], [0, 1, 0],[0, 0, 0], [0, 0, 1]]).astype(np.float32).T)
            FRAME_AXIS_COLORS = (np.array([[1, 0, 0], [1, 0.6, 0],[0, 1, 0], [0.6, 1, 0],[0, 0, 1], [0, 0.6, 1]]).astype(np.float32).T)
            axis_length = 0.1
            axis_width = cost_weights.get('viz_axis_width', 10)
            
            for frame_viz_name in frame_viz_names:
                self.vis.viewer[frame_viz_name].set_object(
                    mg.LineSegments(
                        mg.PointsGeometry(position=axis_length * FRAME_AXIS_POSITIONS, color=FRAME_AXIS_COLORS),
                        mg.LineBasicMaterial(linewidth=axis_width, vertexColors=True),
                    )
                )

    # --- 공통 메서드 ---
    def scale_arms(self, human_left_pose, human_right_pose, human_arm_length=0.60, robot_arm_length=0.75):
        # 이 함수는 H1 케이스 전용으로 남겨두되, solve_ik에서 호출을 관리합니다.
        scale_factor = robot_arm_length / human_arm_length
        robot_left_pose = human_left_pose.copy()
        robot_right_pose = human_right_pose.copy()
        robot_left_pose[:3, 3] *= scale_factor
        robot_right_pose[:3, 3] *= scale_factor
        return robot_left_pose, robot_right_pose

    def solve_ik(self, target_poses: dict, current_lr_arm_motor_q = None, current_lr_arm_motor_dq = None):
        if current_lr_arm_motor_q is not None:
            self.init_data = current_lr_arm_motor_q
        self.opti.set_initial(self.var_q, self.init_data)

        # target_poses 딕셔너리를 기반으로 파라미터 설정
        for name, pose in target_poses.items():
            if name in self.ee_params:
                se3_pose = xyzrpy_to_se3(pose)
                self.opti.set_value(self.ee_params[name], se3_pose)
                if self.Visualization:
                    self.vis.viewer[f'{name}_target'].set_transform(se3_pose)
            else:
                logger_mp.warn(f"Target pose for '{name}' ignored (not in ee_params).")

        self.opti.set_value(self.var_q_last, self.init_data)

        try:
            sol = self.opti.solve()
            sol_q = self.opti.value(self.var_q)
            self.smooth_filter.add_data(sol_q)
            sol_q = self.smooth_filter.filtered_data

            if current_lr_arm_motor_dq is not None: v = current_lr_arm_motor_dq * 0.0
            else: v = (sol_q - self.init_data) * 0.0

            self.init_data = sol_q
            sol_tauff = pin.rnea(self.reduced_robot.model, self.reduced_robot.data, sol_q, v, np.zeros(self.reduced_robot.model.nv))

            if self.Visualization: self.vis.display(sol_q)
            return sol_q, sol_tauff
        
        except Exception as e:
            logger_mp.error(f"ERROR in convergence, plotting debug info.{e}")
            sol_q = self.opti.debug.value(self.var_q)
            self.smooth_filter.add_data(sol_q)
            sol_q = self.smooth_filter.filtered_data

            if current_lr_arm_motor_dq is not None: v = current_lr_arm_motor_dq * 0.0
            else: v = (sol_q - self.init_data) * 0.0

            self.init_data = sol_q
            sol_tauff = pin.rnea(self.reduced_robot.model, self.reduced_robot.data, sol_q, v, np.zeros(self.reduced_robot.model.nv))

            logger_mp.error(f"sol_q:{sol_q} \nmotorstate: \n{current_lr_arm_motor_q} \ntarget_poses: \n{target_poses}")
            if self.Visualization: self.vis.display(sol_q)
            return current_lr_arm_motor_q, np.zeros(self.reduced_robot.model.nv)
        
    
    def get_ee_position(self, q_numeric):
        """
        주어진 관절 각도(q)에 대한 모든 EE의 현재 포즈(SE3)를 딕셔너리로 반환합니다.
        """
        pin.forwardKinematics(self.reduced_robot.model, self.reduced_robot.data, np.array(q_numeric))
        pin.updateFramePlacements(self.reduced_robot.model, self.reduced_robot.data)
        
        poses = {}
        for name, ee_id in self.ee_ids.items():
            se3_pose = self.reduced_robot.data.oMf[ee_id]

            poses[name] = se3_to_xyzrpy(se3_pose)
            
        return poses


