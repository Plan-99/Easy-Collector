DATASET_DIR = '/root/src/backend/datasets'

from ..ik_solver.pinocchio_solver.piper_arm_ik import Piper_ArmIK

SUPPORT_ROBOTS = [
    {
        'name': 'piper',
        'role': 'single_arm',
        'company': 'Piper',
        'joint_dim': 7,
        'joint_names': ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "gripper"],
        'joint_lower_bounds': [-2.618, 0, -2.618, -1.745, -1.22, -2.094, 0],
        'joint_upper_bounds': [2.618, 2.618, 0, 1.745, 1.22, 2.094, 0.087],
        'read_topic': '/joint_states_single',
        'read_topic_msg': 'sensor_msgs/JointState',
        'write_topic': '/joint_states',
        'write_topic_msg': 'sensor_msgs/JointState',
        'tool_inner': True,
        'network_interface': 'can',
    },
    {
        'name': 'piper(no gripper)',
        'role': 'single_arm',
        'company': 'Piper',
        'joint_dim': 6,
        'joint_names': ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"],
        'joint_lower_bounds': [-2.618, 0, -2.618, -1.745, -1.22, -2.094],
        'joint_upper_bounds': [2.618, 2.618, 0, 1.745, 1.22, 2.094],
        'read_topic': '/joint_states_single',
        'read_topic_msg': 'sensor_msgs/JointState',
        'write_topic': '/joint_states',
        'write_topic_msg': 'sensor_msgs/JointState',
        'tool_inner': False,
        'network_interface': 'can',
    },
    {
        'name': 'rb3-730',
        'role': 'single_arm',
        'company': 'Rainbow Robotics',
    },
    {
        'name': 'ur3',
        'role': 'single_arm',
        'company': 'Universal Robots',
    },
    {
        'name': 'ur5',
        'role': 'single_arm',
        'company': 'Universal Robots',
    },
    {
        'name': 'ur10',
        'role': 'single_arm',
        'company': 'Universal Robots',
    },
    {
        'name': 'ur5e',
        'role': 'single_arm',
        'company': 'Universal Robots',
    },
    {
        'name': 'h1',
        'role': 'dual_arm',
        'company': 'Unitree',
    },
    {
        'name': 'rg2',
        'role': 'tool',
        'company': 'OnRobot',
    },
    {
        'name': 'robotiq_2f_85',
        'role': 'tool',
        'company': 'Robotiq',
    },
]