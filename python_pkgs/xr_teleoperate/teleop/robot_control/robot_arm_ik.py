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
# ## 🚀 상위 클래스: IK_Solver
# 
# 모든 IK 클래스들의 공통 로직을 담고 있는 부모 클래스입니다.
# ---------------------------------------------------------------------------
class IK_Solver:
    def __init__(self, 
                 urdf_path, 
                 package_dir, 
                 joints_to_lock, 
                 l_ee_def, 
                 r_ee_def, 
                 cost_weights,
                 use_scaling=False,
                 Visualization=False):
        
        np.set_printoptions(precision=5, suppress=True, linewidth=200)
        self.Visualization = Visualization
        self.use_scaling = use_scaling # solve_ik에서 scale_arms를 사용할지 여부

        # 1. 로봇 로드 (자식 클래스에서 경로를 받아옴)
        self.robot = pin.RobotWrapper.BuildFromURDF(urdf_path, package_dir)

        # 2. 모델 축소 (자식 클래스에서 관절 리스트를 받아옴)
        self.reduced_robot = self.robot.buildReducedRobot(
            list_of_joints_to_lock=joints_to_lock,
            reference_configuration=np.array([0.0] * self.robot.model.nq),
        )

        # 3. EE 프레임 추가 (자식 클래스에서 정의를 받아옴)
        l_ee_joint_name, l_ee_offset = l_ee_def
        r_ee_joint_name, r_ee_offset = r_ee_def
        
        self.reduced_robot.model.addFrame(
            pin.Frame('L_ee',
                      self.reduced_robot.model.getJointId(l_ee_joint_name),
                      pin.SE3(np.eye(3), l_ee_offset),
                      pin.FrameType.OP_FRAME)
        )
        self.reduced_robot.model.addFrame(
            pin.Frame('R_ee',
                      self.reduced_robot.model.getJointId(r_ee_joint_name),
                      pin.SE3(np.eye(3), r_ee_offset),
                      pin.FrameType.OP_FRAME)
        )

        # 4. Casadi 모델 생성 (공통)
        self.cmodel = cpin.Model(self.reduced_robot.model)
        self.cdata = self.cmodel.createData()

        # 5. 심볼릭 변수 생성 (공통)
        self.cq = casadi.SX.sym("q", self.reduced_robot.model.nq, 1) 
        self.cTf_l = casadi.SX.sym("tf_l", 4, 4)
        self.cTf_r = casadi.SX.sym("tf_r", 4, 4)
        cpin.framesForwardKinematics(self.cmodel, self.cdata, self.cq)

        # 6. 오차 함수 정의 (공통)
        self.L_hand_id = self.reduced_robot.model.getFrameId("L_ee")
        self.R_hand_id = self.reduced_robot.model.getFrameId("R_ee")

        self.translational_error = casadi.Function(
            "translational_error",
            [self.cq, self.cTf_l, self.cTf_r],
            [
                casadi.vertcat(
                    self.cdata.oMf[self.L_hand_id].translation - self.cTf_l[:3,3],
                    self.cdata.oMf[self.R_hand_id].translation - self.cTf_r[:3,3]
                )
            ],
        )
        self.rotational_error = casadi.Function(
            "rotational_error",
            [self.cq, self.cTf_l, self.cTf_r],
            [
                casadi.vertcat(
                    cpin.log3(self.cdata.oMf[self.L_hand_id].rotation @ self.cTf_l[:3,:3].T),
                    cpin.log3(self.cdata.oMf[self.R_hand_id].rotation @ self.cTf_r[:3,:3].T)
                )
            ],
        )

        # 7. 최적화 문제 정의 (공통)
        self.opti = casadi.Opti()
        self.var_q = self.opti.variable(self.reduced_robot.model.nq)
        self.var_q_last = self.opti.parameter(self.reduced_robot.model.nq)   # for smooth
        self.param_tf_l = self.opti.parameter(4, 4)
        self.param_tf_r = self.opti.parameter(4, 4)
        self.translational_cost = casadi.sumsqr(self.translational_error(self.var_q, self.param_tf_l, self.param_tf_r))
        self.rotation_cost = casadi.sumsqr(self.rotational_error(self.var_q, self.param_tf_l, self.param_tf_r))
        self.regularization_cost = casadi.sumsqr(self.var_q)
        self.smooth_cost = casadi.sumsqr(self.var_q - self.var_q_last)

        # 8. 제약 조건 및 비용 함수 설정 (가중치는 자식 클래스에서 받아옴)
        self.opti.subject_to(self.opti.bounded(
            self.reduced_robot.model.lowerPositionLimit,
            self.var_q,
            self.reduced_robot.model.upperPositionLimit)
        )
        
        w = cost_weights # 자식 클래스에서 정의한 가중치 딕셔너리
        self.opti.minimize(
            w['trans'] * self.translational_cost + 
            w['rot'] * self.rotation_cost + 
            w['reg'] * self.regularization_cost + 
            w['smooth'] * self.smooth_cost
        )

        # 9. 솔버 설정 (공통)
        opts = {
            'ipopt':{
                'print_level':0,
                'max_iter':50,
                'tol':1e-6
            },
            'print_time':False,# print or not
            'calc_lam_p':False 
        }
        self.opti.solver("ipopt", opts)

        # 10. 필터 및 시각화 초기화 (공통)
        self.init_data = np.zeros(self.reduced_robot.model.nq)
        self.smooth_filter = WeightedMovingFilter(np.array([0.4, 0.3, 0.2, 0.1]), self.reduced_robot.model.nq)
        self.vis = None

        if self.Visualization:
            self.vis = MeshcatVisualizer(self.reduced_robot.model, self.reduced_robot.collision_model, self.reduced_robot.visual_model)
            self.vis.initViewer(open=True) 
            self.vis.loadViewerModel("pinocchio") 
            # L_ee, R_ee의 ID를 동적으로 가져와서 표시
            self.vis.displayFrames(True, frame_ids=[self.L_hand_id, self.R_hand_id], axis_length = 0.15, axis_width = 5)
            self.vis.display(pin.neutral(self.reduced_robot.model))

            frame_viz_names = ['L_ee_target', 'R_ee_target']
            FRAME_AXIS_POSITIONS = (
                np.array([[0, 0, 0], [1, 0, 0],
                          [0, 0, 0], [0, 1, 0],
                          [0, 0, 0], [0, 0, 1]]).astype(np.float32).T
            )
            # 색상도 cost_weights에서 가져오거나 통일
            FRAME_AXIS_COLORS = (
                np.array([[1, 0, 0], [1, 0.6, 0],
                          [0, 1, 0], [0.6, 1, 0],
                          [0, 0, 1], [0, 0.6, 1]]).astype(np.float32).T
            )
            # 축 두께도 cost_weights에서 가져옴
            axis_length = 0.1
            axis_width = cost_weights.get('viz_axis_width', 10) # 기본값 10
            
            for frame_viz_name in frame_viz_names:
                self.vis.viewer[frame_viz_name].set_object(
                    mg.LineSegments(
                        mg.PointsGeometry(
                            position=axis_length * FRAME_AXIS_POSITIONS,
                            color=FRAME_AXIS_COLORS,
                        ),
                        mg.LineBasicMaterial(
                            linewidth=axis_width,
                            vertexColors=True,
                        ),
                    )
                )

    # --- 공통 메서드 ---
    def scale_arms(self, human_left_pose, human_right_pose, human_arm_length=0.60, robot_arm_length=0.75):
        scale_factor = robot_arm_length / human_arm_length
        robot_left_pose = human_left_pose.copy()
        robot_right_pose = human_right_pose.copy()
        robot_left_pose[:3, 3] *= scale_factor
        robot_right_pose[:3, 3] *= scale_factor
        return robot_left_pose, robot_right_pose

    def solve_ik(self, left_wrist, right_wrist, current_lr_arm_motor_q = None, current_lr_arm_motor_dq = None):
        if current_lr_arm_motor_q is not None:
            self.init_data = current_lr_arm_motor_q
        self.opti.set_initial(self.var_q, self.init_data)

        # self.use_scaling 플래그에 따라 scale_arms 호출 여부 결정
        if self.use_scaling:
            left_wrist, right_wrist = self.scale_arms(left_wrist, right_wrist)
            
        if self.Visualization:
            self.vis.viewer['L_ee_target'].set_transform(left_wrist)
            self.vis.viewer['R_ee_target'].set_transform(right_wrist)

        self.opti.set_value(self.param_tf_l, left_wrist)
        self.opti.set_value(self.param_tf_r, right_wrist)
        self.opti.set_value(self.var_q_last, self.init_data)

        try:
            sol = self.opti.solve()
            sol_q = self.opti.value(self.var_q)
            self.smooth_filter.add_data(sol_q)
            sol_q = self.smooth_filter.filtered_data

            if current_lr_arm_motor_dq is not None:
                v = current_lr_arm_motor_dq * 0.0
            else:
                v = (sol_q - self.init_data) * 0.0

            self.init_data = sol_q
            sol_tauff = pin.rnea(self.reduced_robot.model, self.reduced_robot.data, sol_q, v, np.zeros(self.reduced_robot.model.nv))

            if self.Visualization:
                self.vis.display(sol_q)

            return sol_q, sol_tauff
        
        except Exception as e:
            logger_mp.error(f"ERROR in convergence, plotting debug info.{e}")
            sol_q = self.opti.debug.value(self.var_q)
            self.smooth_filter.add_data(sol_q)
            sol_q = self.smooth_filter.filtered_data

            if current_lr_arm_motor_dq is not None:
                v = current_lr_arm_motor_dq * 0.0
            else:
                v = (sol_q - self.init_data) * 0.0

            self.init_data = sol_q
            sol_tauff = pin.rnea(self.reduced_robot.model, self.reduced_robot.data, sol_q, v, np.zeros(self.reduced_robot.model.nv))

            logger_mp.error(f"sol_q:{sol_q} \nmotorstate: \n{current_lr_arm_motor_q} \nleft_pose: \n{left_wrist} \nright_pose: \n{right_wrist}")
            if self.Visualization:
                self.vis.display(sol_q)

            return current_lr_arm_motor_q, np.zeros(self.reduced_robot.model.nv)

# ---------------------------------------------------------------------------
# ## 🔩 자식 클래스 1: G1_29_ArmIK
#
# IK_Solver를 상속받고, G1 29-dof 모델에 특화된 설정값만 정의합니다.
# ---------------------------------------------------------------------------
class G1_29_ArmIK(IK_Solver):
    def __init__(self, Unit_Test=False, Visualization=False):
        # 1. 경로 정의
        if not Unit_Test:
            urdf_path = '../assets/g1/g1_body29_hand14.urdf'
            package_dir = '../assets/g1/'
        else:
            urdf_path = '../../assets/g1/g1_body29_hand14.urdf'
            package_dir = '../../assets/g1/'
        
        # 2. 잠글 관절 정의
        joints_to_lock = [
            "left_hip_pitch_joint", "left_hip_roll_joint", "left_hip_yaw_joint", "left_knee_joint", "left_ankle_pitch_joint", "left_ankle_roll_joint",
            "right_hip_pitch_joint", "right_hip_roll_joint", "right_hip_yaw_joint", "right_knee_joint", "right_ankle_pitch_joint", "right_ankle_roll_joint",
            "waist_yaw_joint", "waist_roll_joint", "waist_pitch_joint",
            "left_hand_thumb_0_joint", "left_hand_thumb_1_joint", "left_hand_thumb_2_joint", "left_hand_middle_0_joint", "left_hand_middle_1_joint", "left_hand_index_0_joint", "left_hand_index_1_joint",
            "right_hand_thumb_0_joint", "right_hand_thumb_1_joint", "right_hand_thumb_2_joint", "right_hand_index_0_joint", "right_hand_index_1_joint", "right_hand_middle_0_joint", "right_hand_middle_1_joint"
        ]
        
        # 3. EE 프레임 정의 (기준 관절, 오프셋)
        l_ee_def = ('left_wrist_yaw_joint', np.array([0.05, 0, 0]).T)
        r_ee_def = ('right_wrist_yaw_joint', np.array([0.05, 0, 0]).T)
        
        # 4. 비용함수 가중치 정의
        cost_weights = {
            'trans': 50,
            'rot': 1.0,  # G1_29, H1_2는 1.0
            'reg': 0.02,
            'smooth': 0.1,
            'viz_axis_width': 20
        }
        
        # 5. 부모 클래스 생성자 호출
        super().__init__(
            urdf_path=urdf_path,
            package_dir=package_dir,
            joints_to_lock=joints_to_lock,
            l_ee_def=l_ee_def,
            r_ee_def=r_ee_def,
            cost_weights=cost_weights,
            use_scaling=False, # G1 모델은 scale_arms 사용 안 함
            Visualization=Visualization
        )

# ---------------------------------------------------------------------------
# ## 🔩 자식 클래스 2: G1_23_ArmIK
# ---------------------------------------------------------------------------
class G1_23_ArmIK(IK_Solver):
    def __init__(self, Unit_Test=False, Visualization=False):
        if not Unit_Test:
            urdf_path = '../assets/g1/g1_body23.urdf'
            package_dir = '../assets/g1/'
        else:
            urdf_path = '../../assets/g1/g1_body23.urdf'
            package_dir = '../../assets/g1/'

        joints_to_lock = [
            "left_hip_pitch_joint", "left_hip_roll_joint", "left_hip_yaw_joint", "left_knee_joint", "left_ankle_pitch_joint", "left_ankle_roll_joint",
            "right_hip_pitch_joint", "right_hip_roll_joint", "right_hip_yaw_joint", "right_knee_joint", "right_ankle_pitch_joint", "right_ankle_roll_joint",
            "waist_yaw_joint"
        ]
        
        l_ee_def = ('left_wrist_roll_joint', np.array([0.20, 0, 0]).T)
        r_ee_def = ('right_wrist_roll_joint', np.array([0.20, 0, 0]).T)
        
        cost_weights = {
            'trans': 50,
            'rot': 0.5, # G1_23, H1은 0.5
            'reg': 0.02,
            'smooth': 0.1,
            'viz_axis_width': 20
        }
        
        super().__init__(
            urdf_path=urdf_path,
            package_dir=package_dir,
            joints_to_lock=joints_to_lock,
            l_ee_def=l_ee_def,
            r_ee_def=r_ee_def,
            cost_weights=cost_weights,
            use_scaling=False,
            Visualization=Visualization
        )

# ---------------------------------------------------------------------------
# ## 🔩 자식 클래스 3: H1_2_ArmIK
# ---------------------------------------------------------------------------
class H1_2_ArmIK(IK_Solver):
    def __init__(self, Unit_Test=False, Visualization=False):
        if not Unit_Test:
            urdf_path = '../assets/h1_2/h1_2.urdf'
            package_dir = '../assets/h1_2/'
        else:
            urdf_path = '../../assets/h1_2/h1_2.urdf'
            package_dir = '../../assets/h1_2/'

        joints_to_lock = [
            "left_hip_yaw_joint", "left_hip_pitch_joint", "left_hip_roll_joint", "left_knee_joint", "left_ankle_pitch_joint", "left_ankle_roll_joint",
            "right_hip_yaw_joint", "right_hip_pitch_joint", "right_hip_roll_joint", "right_knee_joint", "right_ankle_pitch_joint", "right_ankle_roll_joint",
            "torso_joint",
            "L_index_proximal_joint", "L_index_intermediate_joint", "L_middle_proximal_joint", "L_middle_intermediate_joint", "L_pinky_proximal_joint", "L_pinky_intermediate_joint", "L_ring_proximal_joint", "L_ring_intermediate_joint", "L_thumb_proximal_yaw_joint", "L_thumb_proximal_pitch_joint", "L_thumb_intermediate_joint", "L_thumb_distal_joint",
            "R_index_proximal_joint", "R_index_intermediate_joint", "R_middle_proximal_joint", "R_middle_intermediate_joint", "R_pinky_proximal_joint", "R_pinky_intermediate_joint", "R_ring_proximal_joint", "R_ring_intermediate_joint", "R_thumb_proximal_yaw_joint", "R_thumb_proximal_pitch_joint", "R_thumb_intermediate_joint", "R_thumb_distal_joint"
        ]
        
        l_ee_def = ('left_wrist_yaw_joint', np.array([0.05, 0, 0]).T)
        r_ee_def = ('right_wrist_yaw_joint', np.array([0.05, 0, 0]).T)
        
        cost_weights = {
            'trans': 50,
            'rot': 1.0, # G1_29, H1_2는 1.0
            'reg': 0.02,
            'smooth': 0.1,
            'viz_axis_width': 10
        }
        
        super().__init__(
            urdf_path=urdf_path,
            package_dir=package_dir,
            joints_to_lock=joints_to_lock,
            l_ee_def=l_ee_def,
            r_ee_def=r_ee_def,
            cost_weights=cost_weights,
            use_scaling=True, # H1 모델은 scale_arms 사용
            Visualization=Visualization
        )

# ---------------------------------------------------------------------------
# ## 🔩 자식 클래스 4: H1_ArmIK
# ---------------------------------------------------------------------------
class H1_ArmIK(IK_Solver):
    def __init__(self, Unit_Test=False, Visualization=False):
        if not Unit_Test:
            urdf_path = '../assets/h1/h1_with_hand.urdf'
            package_dir = '../assets/h1/'
        else:
            urdf_path = '../../assets/h1/h1_with_hand.urdf'
            package_dir = '../../assets/h1/'

        joints_to_lock = [
            "right_hip_roll_joint", "right_hip_pitch_joint", "right_knee_joint", "left_hip_roll_joint", "left_hip_pitch_joint", "left_knee_joint", "torso_joint", "left_hip_yaw_joint", "right_hip_yaw_joint",
            "left_ankle_joint", "right_ankle_joint",
            "L_index_proximal_joint", "L_index_intermediate_joint", "L_middle_proximal_joint", "L_middle_intermediate_joint", "L_ring_proximal_joint", "L_ring_intermediate_joint", "L_pinky_proximal_joint", "L_pinky_intermediate_joint", "L_thumb_proximal_yaw_joint", "L_thumb_proximal_pitch_joint", "L_thumb_intermediate_joint", "L_thumb_distal_joint",
            "R_index_proximal_joint", "R_index_intermediate_joint", "R_middle_proximal_joint", "R_middle_intermediate_joint", "R_ring_proximal_joint", "R_ring_intermediate_joint", "R_pinky_proximal_joint", "R_pinky_intermediate_joint", "R_thumb_proximal_yaw_joint", "R_thumb_proximal_pitch_joint", "R_thumb_intermediate_joint", "R_thumb_distal_joint",
            "left_hand_joint", "right_hand_joint"
        ]
        
        l_ee_def = ('left_elbow_joint', np.array([0.2605 + 0.05, 0, 0]).T)
        r_ee_def = ('right_elbow_joint', np.array([0.2605 + 0.05, 0, 0]).T)
        
        cost_weights = {
            'trans': 50,
            'rot': 0.5, # G1_23, H1은 0.5
            'reg': 0.02,
            'smooth': 0.1,
            'viz_axis_width': 10
        }
        
        super().__init__(
            urdf_path=urdf_path,
            package_dir=package_dir,
            joints_to_lock=joints_to_lock,
            l_ee_def=l_ee_def,
            r_ee_def=r_ee_def,
            cost_weights=cost_weights,
            use_scaling=True, # H1 모델은 scale_arms 사용
            Visualization=Visualization
        )
        