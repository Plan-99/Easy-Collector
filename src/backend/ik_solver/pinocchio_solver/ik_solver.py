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

def se3_to_xyzaxayaz(se3_matrix):
    """
    SE3 객체를 (x, y, z, ax, ay, az) 형식의 리스트로 변환합니다.
    ax, ay, az는 회전 벡터(Rotation Vector)입니다.
    """
    # 1. 위치 추출 (x, y, z)
    xyz = se3_matrix.translation
    
    # 2. 회전 행렬을 회전 벡터로 변환 (ax, ay, az)
    # pin.log3는 SO(3) -> so(3) 매핑을 수행합니다.
    rotation_matrix = se3_matrix.rotation
    axayaz = pin.log3(rotation_matrix)
    
    return np.concatenate([xyz, axayaz]).tolist()

def xyzaxayaz_to_se3(xyzaxayaz):
    """
    (x, y, z, ax, ay, az) 형식의 리스트를 SE3 객체로 변환합니다.
    ax, ay, az는 회전 벡터(Rotation Vector)입니다.
    """
    xyz = np.array(xyzaxayaz[:3])
    axayaz = np.array(xyzaxayaz[3:])
    
    # 회전 벡터를 회전 행렬로 변환 (지수 사상)
    rotation_matrix = pin.exp3(axayaz)
    
    se3_pose = pin.SE3(rotation_matrix, xyz)
    return se3_pose.homogeneous

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
            w['rot'] * self.rotation_cost + # rotation_cost 변수명 확인
            w['reg'] * self.regularization_cost + 
            w['smooth'] * self.smooth_cost
        )

        # 9. 솔버 설정
        opts = {
            'ipopt': {
                'print_level': 0,
                'max_iter': 200,       # 반복 횟수를 과감히 줄임 (Warm start 믿고 가기)
                'tol': 1e-8,          # 현실적인 오차 허용
                'warm_start_init_point': 'yes',
                'mu_strategy': 'adaptive', # 수렴 속도 향상
            },
            'print_time': False
        }
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
                # [수정] xyzrpy_to_se3 대신 xyzaxayaz_to_se3 사용
                se3_pose = xyzaxayaz_to_se3(pose)
                
                self.opti.set_value(self.ee_params[name], se3_pose)
                if self.Visualization:
                    self.vis.viewer[f'{name}_target'].set_transform(se3_pose)
            else:
                logger_mp.warn(f"Target pose for '{name}' ignored (not in ee_params).")

        self.opti.set_value(self.var_q_last, self.init_data)

        try:
            sol = self.opti.solve()
            sol_q = self.opti.value(self.var_q)

            # 속도 및 토크 계산 (v는 현재 0으로 설정되어 있음)
            if current_lr_arm_motor_dq is not None: 
                v = current_lr_arm_motor_dq * 0.0
            else: 
                v = (sol_q - self.init_data) * 0.0

            self.init_data = sol_q
            sol_tauff = pin.rnea(self.reduced_robot.model, self.reduced_robot.data, sol_q, v, np.zeros(self.reduced_robot.model.nv))

            if self.Visualization: self.vis.display(sol_q)
            return sol_q, sol_tauff
        
        except Exception as e:
            logger_mp.error(f"ERROR in convergence, plotting debug info. {e}")
            sol_q = self.opti.debug.value(self.var_q)
            self.smooth_filter.add_data(sol_q)
            sol_q = self.smooth_filter.filtered_data

            if current_lr_arm_motor_dq is not None: v = current_lr_arm_motor_dq * 0.0
            else: v = (sol_q - self.init_data) * 0.0

            self.init_data = sol_q
            sol_tauff = pin.rnea(self.reduced_robot.model, self.reduced_robot.data, sol_q, v, np.zeros(self.reduced_robot.model.nv))

            # 에러 로그 출력 시에도 형식이 바뀐 것을 명시
            logger_mp.error(f"sol_q:{sol_q} \ntarget_poses(xyzaxayaz): \n{target_poses}")
            if self.Visualization: self.vis.display(sol_q)
            
            # 실패 시 현재 상태를 그대로 유지하도록 반환
            return current_lr_arm_motor_q, np.zeros(self.reduced_robot.model.nv)
        
    
    # --- IK_Solver 클래스 내부 메서드 수정 ---
    def get_ee_position(self, q_numeric):
        """
        주어진 관절 각도(q)에 대한 모든 EE의 현재 포즈를 
        (x, y, z, ax, ay, az) 딕셔너리로 반환합니다.
        """
        # 1. 순기구학 계산
        pin.forwardKinematics(self.reduced_robot.model, self.reduced_robot.data, np.array(q_numeric))
        pin.updateFramePlacements(self.reduced_robot.model, self.reduced_robot.data)
        
        poses = {}
        for name, ee_id in self.ee_ids.items():
            # 2. 각 EE의 SE3 포즈 가져오기
            se3_pose = self.reduced_robot.data.oMf[ee_id]

            # 3. 새로운 헬퍼 함수를 사용하여 변환
            poses[name] = se3_to_xyzaxayaz(se3_pose)
            
        return poses
    
    def reset_state(self, current_q):
        """IK Solver의 필터와 최적화기 내부 파라미터를 현재 상태로 강제 동기화"""
        # 1. 데이터를 넘파이 배열로 변환
        q_target = np.array(current_q).flatten()
        self.init_data = q_target

        # 2. [가장 중요] CasADi 파라미터 및 초기값 리셋
        # = 가 아니라 set_value와 set_initial을 사용해야 합니다.
        self.opti.set_value(self.var_q_last, q_target) # 이전 위치를 현재 위치로 세팅 (smooth_cost 기준점)
        self.opti.set_initial(self.var_q, q_target)    # 솔버의 계산 시작점을 현재 위치로 세팅

        # 3. 필터 큐 비우기 및 현재 값으로 채우기
        self.smooth_filter._data_queue = []
        for _ in range(self.smooth_filter._window_size):
            self.smooth_filter.add_data(q_target)
            
        logger_mp.info(f"IK Solver state reset complete. Q: {q_target.tolist()}")

    def compute_delta_target(self, name, current_q, delta_xyzaxayaz, frame='global'):
        """
        현재 관절 상태(q)와 변화량(delta)을 받아 차기 목표(xyzaxayaz)를 계산합니다.
        delta_xyzaxayaz: [dx, dy, dz, dax, day, daz]
        frame: 'global' (베이스 기준) 또는 'local' (EE 기준)
        """
        # 1. 현재 포즈 계산 (FK)
        pin.forwardKinematics(self.reduced_robot.model, self.reduced_robot.data, current_q)
        pin.updateFramePlacements(self.reduced_robot.model, self.reduced_robot.data)
        
        ee_id = self.ee_ids[name]
        current_oMf = self.reduced_robot.data.oMf[ee_id]
        
        # 2. 변화량 분리
        d_xyz = np.array(delta_xyzaxayaz[:3])
        d_axayaz = np.array(delta_xyzaxayaz[3:])
        
        # 3. 미세 회전 행렬 생성 (Exponential Map)
        delta_R = pin.exp3(d_axayaz)
        
        # 4. 행렬 연산으로 차기 자세 계산
        if frame == 'global':
            # 베이스 축 기준 회전
            new_R = delta_R @ current_oMf.rotation 
        else:
            # 로봇 손(EE) 축 기준 회전
            new_R = current_oMf.rotation @ delta_R 
            
        new_t = current_oMf.translation + d_xyz
        
        # 5. 결과를 다시 xyzaxayaz 형식으로 변환
        new_se3 = pin.SE3(new_R, new_t)
        return se3_to_xyzaxayaz(new_se3)

