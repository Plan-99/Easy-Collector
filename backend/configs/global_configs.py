import os


# Datasets are persistent runtime data and must NOT live under the bind-mounted
# source tree (/root/src is bind-mounted from the dev checkout). Resolve under
# EASYTRAINER_DATA_DIR when available so writes land on the persistent volume.
_data_root = os.environ.get('EASYTRAINER_DATA_DIR')
DATASET_DIR = os.path.join(_data_root, 'datasets') if _data_root else '/root/src/backend/datasets'
CHECKPOINT_DIR = os.path.join(_data_root, 'checkpoints') if _data_root else '/root/src/backend/checkpoints'


def resolve_checkpoint_dir(checkpoint_id):
    """Locate the on-disk checkpoint directory for a given id.

    Tries, in order:
      1. ``$EASYTRAINER_DATA_DIR/checkpoints/<id>`` — canonical install location
         (where remote_train extracts the downloaded model).
      2. ``$TRAINING_SERVER_DATA_DIR/checkpoints/<machine_id>/<id>`` — raw output
         of local in-container training.
      3. ``/root/src/backend/checkpoints/<id>`` — legacy path from when the
         project lived under ``src/``.

    Returns the first existing directory, or the canonical path (1) if none
    exist so callers attempting writes still get a sensible target.
    """
    cid = str(checkpoint_id)
    candidates = [os.path.join(CHECKPOINT_DIR, cid)]

    training_data = os.environ.get('TRAINING_SERVER_DATA_DIR')
    if training_data:
        try:
            from ..utils.machine_id import machine_id
            candidates.append(os.path.join(training_data, 'checkpoints', machine_id(), cid))
        except Exception:
            pass

    legacy = os.path.join('/root/src/backend/checkpoints', cid)
    if legacy not in candidates:
        candidates.append(legacy)

    for path in candidates:
        if os.path.isdir(path):
            return path
    return candidates[0]

# Trained checkpoints — training_server가 이 경로에 직접 저장하고 (host volume
# share), remote 학습 시 _download_and_install_model이 동일한 경로에 풀어준다.
# machine_id namespace는 lazy하게 평가해야 하므로 함수로 노출.
_checkpoint_base = os.path.join(
    _data_root or '/opt/easytrainer', 'training_data', 'checkpoints'
)


def get_checkpoint_dir(checkpoint_id) -> str:
    """checkpoint_id에 해당하는 로컬 체크포인트 디렉터리 절대경로."""
    from ..utils.machine_id import machine_id
    return os.path.join(_checkpoint_base, machine_id(), str(checkpoint_id))

_ALL_ROBOTS = [
    {
        'name': 'test_arm',
        'role': 'single_arm',
        'company': 'Test',
        'joint_dim': 7,
        'joint_names': ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "gripper"],
        'joint_lower_bounds': [-2.618, 0, -2.618, -1.745, -1.22, -2.094, 0],
        'joint_upper_bounds': [2.618, 2.618, 0, 1.745, 1.22, 2.094, 0.087],
        'read_topic': '/joint_states_single',
        'read_topic_msg': 'sensor_msgs/JointState',
        'write_type': 'topic',
        'write_topic': '/joint_states',
        'write_topic_msg': 'sensor_msgs/JointState',
        'interpolation': True,
        'tool_inner': True,
        'tool_index': [6],
        'ik_available': True,
        'custom_fields': [],
    },
    {
        'name': 'piper',
        'module_id': 'robot_piper',
        'role': 'single_arm',
        'company': 'Piper',
        'joint_dim': 7,
        'joint_names': ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "gripper"],
        'joint_lower_bounds': [-2.618, 0, -2.618, -1.745, -1.22, -2.094, 0],
        'joint_upper_bounds': [2.618, 2.618, 0, 1.745, 1.22, 2.094, 0.087],
        'interpolation': True,
        'sdk_control': True,
        'sdk_type': 'piper',
        'tool_inner': True,
        'tool_index': [6],
        'ik_available': True,
        'custom_fields': ['can_port'],
    },
    
    {
        'name': 'piper(no gripper)',
        'module_id': 'robot_piper',
        'role': 'single_arm',
        'company': 'Piper',
        'joint_dim': 6,
        'joint_names': ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"],
        'joint_lower_bounds': [-2.618, 0, -2.618, -1.745, -1.22, -2.094],
        'joint_upper_bounds': [2.618, 2.618, 0, 1.745, 1.22, 2.094],
        'interpolation': True,
        'sdk_control': True,
        'sdk_type': 'piper',
        'tool_inner': False,
        'tool_index': [],
        'ik_available': True,
        'custom_fields': ['can_port'],
    },
    {
        'name': 'tm_12',
        'module_id': 'robot_techman',
        'role': 'single_arm',
        'company': 'OMRON',
        'joint_dim': 6,
        'joint_names': ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"],
        'joint_lower_bounds': [-6.283, -6.283, -2.827, -6.283, -6.283, -6.283],
        'joint_upper_bounds': [6.283, 6.283, 2.827, 6.283, 6.283, 6.283],
        'read_topic': '/joint_states',
        'read_topic_msg': 'sensor_msgs/JointState',
        'write_type': 'service',
        'write_topic': '/send_script',
        'write_topic_msg': 'tm_msgs/srv/SendScript',
        'tool_inner': False,
        'tool_index': [],
        'ik_available': True,
        'custom_fields': ['ip_address'],
    },
    {
        'name': 'tm_12s',
        'module_id': 'robot_techman',
        'role': 'single_arm',
        'company': 'OMRON',
        'joint_dim': 6,
        'joint_names': ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"],
        'joint_lower_bounds': [-6.283, -6.283, -2.827, -6.283, -6.283, -6.283],
        'joint_upper_bounds': [6.283, 6.283, 2.827, 6.283, 6.283, 6.283],
        'read_topic': '/joint_states',
        'read_topic_msg': 'sensor_msgs/JointState',
        'write_type': 'service',
        'write_topic': '/send_script',
        'write_topic_msg': 'tm_msgs/srv/SendScript',
        'tool_inner': False,
        'tool_index': [],
        'ik_available': True,
        'custom_fields': ['ip_address'],
    },
    {
        'name': 'tm_12_robotiq',
        'module_id': 'robot_techman',
        'role': 'single_arm',
        'company': 'OMRON',
        'joint_dim': 7,
        'joint_names': ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6", "gripper"],
        'joint_lower_bounds': [-6.283, -6.283, -2.827, -6.283, -6.283, -6.283, 0.0],
        'joint_upper_bounds': [6.283, 6.283, 2.827, 6.283, 6.283, 6.283, 0.85],
        'read_topic': '/feedback_states',
        'read_topic_msg': 'tm_msgs/msg/FeedbackState',
        'write_type': 'service',
        'write_topic': '/send_script',
        'write_topic_msg': 'tm_msgs/srv/SendScript',
        'tool_inner': True,
        'tool_index': [6],
        'ik_available': True,
        'custom_fields': ['ip_address'],
    },
    {
        'name': 'rb3_730es_u',
        'module_id': 'robot_rbpodo',
        'role': 'single_arm',
        'company': 'Rainbow Robotics',
        'joint_dim': 6,
        'joint_names': ["base", "shoulder", "elbow", "wrist1", "wrist2", "wrist3"],
        'joint_lower_bounds': [-3.14, -3.14, -3.14, -3.14, -3.14, -3.14],
        'joint_upper_bounds': [3.14, 3.14, 3.14, 3.14, 3.14, 3.14],
        'read_topic': '/rbpodo/joint_states',
        'read_topic_msg': 'sensor_msgs/JointState',
        'write_type': 'topic',
        'write_topic': '/position_controllers/commands',
        'write_topic_msg': 'std_msgs/Float64MultiArray',
        'tool_inner': False,
        'tool_index': [],
        'ik_available': True,
        'custom_fields': ['ip_address'],
    },
    {
        'name': 'rb5_850e',
        'module_id': 'robot_rbpodo',
        'role': 'single_arm',
        'company': 'Rainbow Robotics',
        'joint_dim': 6,
        'joint_names': ["base", "shoulder", "elbow", "wrist1", "wrist2", "wrist3"],
        'joint_lower_bounds': [-3.14, -3.14, -3.14, -3.14, -3.14, -3.14],
        'joint_upper_bounds': [3.14, 3.14, 3.14, 3.14, 3.14, 3.14],
        'read_topic': '/rbpodo/joint_states',
        'read_topic_msg': 'sensor_msgs/JointState',
        'write_type': 'topic',
        'write_topic': '/position_controllers/commands',
        'write_topic_msg': 'std_msgs/Float64MultiArray',
        'tool_inner': False,
        'tool_index': [],
        'custom_fields': ['ip_address'],
    },
    {
        'name': 'kinova_gen3_7dof_robotiq_2f_85',
        'module_id': 'robot_kinova',
        'role': 'single_arm',
        'company': 'Kinova',
        'joint_dim': 7,
        'joint_names': ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6", "joint_7"],
        'joint_lower_bounds': [-3.14, -3.14, -3.14, -3.14, -3.14, -3.14, -3.14],
        'joint_upper_bounds': [3.14, 3.14, 3.14, 3.14, 3.14, 3.14, 3.14],
        'read_topic': '/joint_trajectory_controller/controller_state',
        'read_topic_msg': 'control_msgs/JointTrajectoryControllerState',
        'write_topic': '/joint_trajectory_controller/joint_trajectory',
        'write_topic_msg': 'trajectory_msgs/JointTrajectory',
        'tool_inner': False,
        'tool_index': [],
        'ik_available': True,
        'custom_fields': ['ip_address'],
    },
    {
        'name': 'fairino_fr5',
        'module_id': 'robot_fairino',
        'role': 'single_arm',
        'company': 'Fairino',
        'joint_dim': 6,
        'joint_names': ["j1", "j2", "j3", "j4", "j5", "j6"],
        'joint_lower_bounds': [-3.0543, -4.6251, -2.8274, -4.6251, -3.0543, -3.0543],
        'joint_upper_bounds': [3.0543, 1.4835, 2.8274, 1.4835, 3.0543, 3.0543],
        'interpolation': True,
        'sdk_control': True,
        'sdk_type': 'fairino',
        'tool_inner': False,
        'tool_index': [],
        'ik_available': True,
        'custom_fields': ['ip_address'],
    },
    {
        'name': 'jaka_zu12',
        'module_id': 'robot_jaka',
        'role': 'single_arm',
        'company': 'JAKA',
        'joint_dim': 6,
        'joint_names': ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"],
        'joint_lower_bounds': [-6.28, -1.48, -3.05, -1.48, -6.28, -6.28],
        'joint_upper_bounds': [6.28, 4.62, 3.05, 4.62, 6.28, 6.28],
        'read_topic': '/jaka_driver/joint_position',
        'read_topic_msg': 'sensor_msgs/JointState',
        'write_type': 'service',
        'write_topic': '/jaka_driver/servo_j',
        'write_topic_msg': 'jaka_msgs/srv/ServoMove',
        'tool_inner': False,
        'tool_index': [],
        'ik_available': True,
        'custom_fields': ['ip_address'],
    },
    {
        'name': 'robotiq_2f_85',
        'module_id': 'gripper_robotiq',
        'role': 'tool',
        'company': 'Robotiq',
        'joint_dim': 1,
        'joint_names': ['robotiq_85_left_knuckle_joint'],
        'joint_lower_bounds': [0.0],
        'joint_upper_bounds': [0.85],
        'read_topic': '/joint_states',
        'read_topic_msg': 'sensor_msgs/JointState',
        'write_type': 'action',
        'write_topic': '/robotiq_gripper_controller/gripper_cmd',
        'write_topic_msg': 'control_msgs/action/GripperCommand',
        'tool_inner': False,
        'tool_index': [],
        'custom_fields': ['serial_port'],
    },
    {
        'name': '2FG7',
        'module_id': 'gripper_onrobot',
        'role': 'tool',
        'company': 'OnRobot',
        'joint_dim': 1,
        'joint_names': ['gripper_pos'],
        'joint_lower_bounds': [0.1],
        'joint_upper_bounds': [0.48],
        'read_topic': '/joint_states',
        'read_topic_msg': 'sensor_msgs/JointState',
        'write_type': 'topic',
        'write_topic': '/command',
        'write_topic_msg': 'sensor_msgs/JointState',
        'tool_inner': False,
        'tool_index': [],
        'custom_fields': ['ip_address', 'port'],
    }
]

_ALL_SENSORS = [
    {
        'name': 'realsense_d435_color',
        'company': 'Intel',
        'role': 'rgb_camera',
        'topic_type': 'color',
        'read_topic': '/camera/color/image_raw/compressed',
        'read_topic_msg': 'sensor_msgs/CompressedImage',
        'custom_fields': ['serial_no'],
        'resolution': [1280, 720],
        'module_id': 'sensor_realsense',
    },
    {
        'name': 'realsense_d405_color',
        'company': 'Intel',
        'role': 'rgb_camera',
        'topic_type': 'color',
        'read_topic': '/camera/color/image_rect_raw/compressed',
        'read_topic_msg': 'sensor_msgs/CompressedImage',
        'custom_fields': ['serial_no'],
        'resolution': [848, 480],
        'module_id': 'sensor_realsense',
    },
    {
        'name': 'webcam_color',
        'company': 'Logitec',
        'role': 'rgb_camera',
        'topic_type': 'color',
        'read_topic': '/image_raw/compressed',
        'read_topic_msg': 'sensor_msgs/CompressedImage',
        'custom_fields': ['device_index'],
        'resolution': [600, 480],
        'module_id': 'sensor_webcam',
    },
    {
        'name': 'kinova_vision_color',
        'company': 'Kinova',
        'role': 'rgb_camera',
        'topic_type': 'color',
        'read_topic': '/color/image_raw/compressed',
        'read_topic_msg': 'sensor_msgs/CompressedImage',
        'custom_fields': ['ip_address'],
        'resolution': [1980, 1080],
        'module_id': 'robot_kinova',
    }
]

def _get_installed_module_ids():
    """설치된 모듈의 ID 목록을 반환한다.

    Single source of truth: /opt/easytrainer/project/modules/*.json
    런처에서 모듈 설치/제거 시 이 폴더에 manifest를 생성/삭제한다.
    """
    import json
    data_root = os.environ.get('EASYTRAINER_DATA_DIR', '/opt/easytrainer')
    modules_dir = os.path.join(data_root, 'project', 'modules')
    installed = set()

    if os.path.isdir(modules_dir):
        for fname in os.listdir(modules_dir):
            if not fname.endswith('.json'):
                continue
            fpath = os.path.join(modules_dir, fname)
            try:
                with open(fpath) as f:
                    meta = json.load(f)
                mid = meta.get('id', fname[:-5])  # fallback to filename without .json
                installed.add(mid)
            except Exception:
                pass

    return installed


def _get_support_robots():
    installed = _get_installed_module_ids()
    return [r for r in _ALL_ROBOTS if not r.get('module_id') or r['module_id'] in installed]

def _get_support_sensors():
    installed = _get_installed_module_ids()
    return [s for s in _ALL_SENSORS if not s.get('module_id') or s['module_id'] in installed]

# 하위호환: 기존 코드에서 SUPPORT_ROBOTS / SUPPORT_SENSORS를 직접 참조하는 곳 대응
class _DynamicList:
    """매 접근 시 함수를 호출하여 최신 목록을 반환하는 프록시."""
    def __init__(self, fn):
        self._fn = fn
    def __iter__(self):
        return iter(self._fn())
    def __len__(self):
        return len(self._fn())
    def __getitem__(self, idx):
        return self._fn()[idx]
    def __contains__(self, item):
        return item in self._fn()
    def __bool__(self):
        return bool(self._fn())

SUPPORT_ROBOTS = _DynamicList(_get_support_robots)
SUPPORT_SENSORS = _DynamicList(_get_support_sensors)

def get_robot_by_name(name):
    return next((robot for robot in _get_support_robots() if robot.get('name') == name), None)

def get_sensor_by_name(name):
    return next((sensor for sensor in _get_support_sensors() if sensor.get('name') == name), None)