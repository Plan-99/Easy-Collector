import numpy as np
from .ik_solver import IK_Solver


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
        l_ee_def = ('L_ee', 'left_wrist_yaw_joint', np.array([0.05, 0, 0]).T)
        r_ee_def = ('R_ee', 'right_wrist_yaw_joint', np.array([0.05, 0, 0]).T)
        ee_definitions = [l_ee_def, r_ee_def]
        
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
            ee_definitions=ee_definitions,
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
        
        ee_definitions = [
            ('L_ee', 'left_wrist_roll_joint', np.array([0.20, 0, 0]).T),
            ('R_ee', 'right_wrist_roll_joint', np.array([0.20, 0, 0]).T)
        ],  
        
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
            ee_definitions=ee_definitions,
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
        
        ee_definitions = [
            ('L_ee', 'left_wrist_yaw_joint', np.array([0.05, 0, 0]).T),
            ('R_ee', 'right_wrist_yaw_joint', np.array([0.05, 0, 0]).T)
        ],
        
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
            ee_definitions=ee_definitions,
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
        
# --- 👇 변경된 부분 ---
        ee_definitions = [
            ('L_ee', 'left_elbow_joint', np.array([0.2605 + 0.05, 0, 0]).T),
            ('R_ee', 'right_elbow_joint', np.array([0.2605 + 0.05, 0, 0]).T)
        ]
        
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
            ee_definitions=ee_definitions,
            cost_weights=cost_weights,
            use_scaling=True, # H1 모델은 scale_arms 사용
            Visualization=Visualization
        )
