from flask import Blueprint, request, current_app
from flask_socketio import Namespace, emit
from ...database.models.robot_model import Robot as RobotModel
import time
import os
import subprocess
from ...env.agent import Agent
from ..process.subscribe_robot import subscribe_robot_topic
from ...configs.global_configs import SUPPORT_ROBOTS
from ..utils.runtime import attach_robot_runtime

# 1. Blueprint 생성
# 이 블루프린트는 카메라와 관련된 'HTTP' 라우트를 관리합니다.
robot_bp = Blueprint('robot', __name__)


class RobotNamespace(Namespace):
    def __init__(self, namespace, process_manager):
        super().__init__(namespace)
        self.pm = process_manager

    # def on_connect(self):
    #     print(f'Client connected to /robot namespace: {request.sid}')
    #     emit('response', {'data': '카메라 제어 채널에 연결되었습니다.'})

    # def on_disconnect(self):
    #     print(f'Client disconnected from /robot namespace: {request.sid}')


@robot_bp.route('/robots', methods=['GET'])
def get_robots():
    robots = RobotModel.with_('leader_robot_preset').where('hide', False).get()
    robots = [robot.to_dict() for robot in robots]

    processes = set(current_app.pm.list_processes())
    for robot in robots:
        attach_robot_runtime(robot, processes)

    return {
        'status': 'success',
        'robots': robots
    }, 200

@robot_bp.route('/robots:supporting', methods=['GET'])
def get_supporting_robots():
    support_robot_json = []
    for robot in SUPPORT_ROBOTS:
        robot_copy = robot.copy()
        robot_copy.pop('ik_setting', None)
        support_robot_json.append(robot_copy)
    return {
        'status': 'success',
        'robots': support_robot_json
    }, 200

@robot_bp.route('/robot/<id>', methods=['GET'])
def get_robot(id):
    robot = RobotModel.find(id)
    if robot:
        processes = set(current_app.pm.list_processes())
        robot = attach_robot_runtime(robot.to_dict(), processes)
        return {
            'status': 'success',
            'robot': robot
        }, 200
    else:
        return {
            'status': 'error',
            'message': 'Robot not found'
        }, 404

@robot_bp.route('/robot:start', methods=['POST'])
def start_robot():
    data = request.json
    process_id = data.get('process_id')
    id = data.get('id')
    type = data.get('type')
    company = data.get('company', '')
    settings = data.get('settings', {})

    # Stop lingering processes from previous attempts to allow clean restart
    try:
        current_app.pm.stop_process(process_id)
    except Exception:
        pass
    try:
        current_app.pm.stop_function('subscribe_robot_' + str(id))
    except Exception:
        pass

    command = ''
    if company == 'Piper':
        script_path = os.path.expanduser('~/ros2_ws/src/piper_ros/can_activate_main.sh')
        current_app.pm.start_process(
            name='can_config',
            command=['bash', script_path],
            log_emit_id = process_id
        )
        time.sleep(1)

        gripper_exist = 'true' if type == 'piper' else 'false'
        can_port = settings.get("can_port", "can0")
        if can_port.startswith("can_"):
            can_port = "can" + can_port[4:]
        can_port = _ensure_can_interface(can_port)
        # If still missing, fail fast
        if not os.path.exists(f"/sys/class/net/{can_port}"):
            return {'status': 'error', 'message': f'CAN interface {can_port} not found'}, 400
        command = ['ros2', 'launch', 'piper', 'start_single_piper.launch.py', 
                   f'namespace:=ec_robot_{id}', 
                   f'can_port:={can_port}', 
                   'auto_enable:=true', 'rviz_ctrl_flag:=false', 
                   f'gripper_exist:={gripper_exist}']

    if company == 'Rainbow Robotics':
        # moveit_command = ['ros2', 'launch', 'rbpodo_bringup', 'moveit.launch.py', f'namespace:=ec_robot_{id}', f'robot_ip:={settings.get("ip", "10.0.2.27")}', 'use_fake_hardware:=false']
        
        command = ['ros2', 'launch', 'rbpodo_bringup', 'rbpodo.launch.py', f'namespace:=ec_robot_{id}', f'robot_ip:={settings.get("ip", "10.0.2.27")}', 'use_fake_hardware:=false']
        
    if company == 'OnRobot':
        command = ['ros2', 'launch', 'onrobot_rg_control', 'bringup.launch.py',
                   f'namespace:=ec_robot_{id}',
                   f'gripper:={type}',
                   f'ip:={settings.get("ip_address", "10.0.2.27")}',
                   f'port:={settings.get("port", 41414)}',
                   f'changer_addr:={settings.get("changer_address", 5)}'
        ]

    if company == 'Robotiq':
        command = ['ros2', 'launch', 'robotiq_description', 'robotiq_control.launch.py',
                   f'namespace:=ec_robot_{id}',
                   f'com_port:={settings.get("serial_port", "/dev/ttyUSB0")}'
        ]

    if company == "Kinova":
        command = ['ros2', 'launch', f'{type}_moveit_config', 'robot.launch.py',
                f'namespace:=ec_robot_{id}',
                f'robot_ip:={settings.get("ip_address", "192.168.1.10")}',
                'launch_rviz:=false'
        ]

    if company == 'OMRON':
        command = ['ros2', 'launch', 'tm_driver', 'tm_bringup.launch.py', 
                   f'namespace:=ec_robot_{id}', 
                   f'robot_ip:={settings.get("ip_address", "192.168.1.10")}']
        
    if company == 'JAKA':
        command = ['ros2', 'launch', 'jaka_driver', 'robot_start.launch.py', 
                   f'namespace:=ec_robot_{id}', 
                   f'ip:={settings.get("ip_address", "192.168.1.10")}']

    print(f"Attempting to start robot")

    process = current_app.pm.start_process(
        name=process_id,
        command=command,
    )

    if company == 'JAKA':
        # Give the driver a moment to start up before enabling servo mode
        time.sleep(5)
        
        from jaka_msgs.srv import ServoMoveEnable
        
        # The service name in the driver has a leading '/', so it's absolute
        service_name = '/jaka_driver/servo_move_enable'
        servo_client = current_app.node.create_client(ServoMoveEnable, service_name)
        
        if not servo_client.wait_for_service(timeout_sec=5.0):
            current_app.logger.error(f"JAKA servo_move_enable service not available.")
            current_app.pm.stop_process(process_id)
            return {'status': 'error', 'message': f'Failed to find JAKA servo_move_enable service'}, 500

        req = ServoMoveEnable.Request()
        req.enable = True
        
        # Fire and forget the service call
        future = servo_client.call_async(req)
        current_app.logger.info("Attempted to enable JAKA servo mode.")

    # agent = Agent(current_app.node, data)

    # current_app.agents[id] = agent

    if process:
        return {
            'status': 'success',
            'message': f'Robot process started',
            'pid': process.pid
        }, 200
    else:
        return {'status': 'error', 'message': f'Failed to start robot'}, 500
    

@robot_bp.route('/robot:stop', methods=['POST'])
def stop_robot():
    robot_id = request.json.get('id')
    process_id = request.json.get('process_id')
    current_app.pm.stop_process(process_id)
    current_app.pm.stop_process('leader_teleoperation')
    current_app.pm.stop_function('subscribe_robot_' + str(robot_id))
    current_app.agents.pop(int(robot_id), None)
    return {'status': 'success', 'message': 'Robot process stopped'}, 200


@robot_bp.route('/robot', methods=['POST'])
def create_robot():
    name = request.json.get('name')
    type = request.json.get('type')
    homepose = request.json.get('homepose', [])
    role = ''

    settings = {}

    if type == 'custom':
        role = request.json.get('role', '')
        settings = {
            'role': request.json.get('role', ''),
            'read_topic': request.json.get('read_topic', ''),
            'read_topic_msg': request.json.get('read_topic_msg', ''),
            'write_type': request.json.get('write_type', ''),
            'write_topic': request.json.get('write_topic', ''),
            'write_topic_msg': request.json.get('write_topic_msg', ''),
            'joint_names': request.json.get('joint_names', []),
            'joint_lower_bounds': request.json.get('joint_lower_bounds', []),
            'joint_upper_bounds': request.json.get('joint_upper_bounds', []),
            'tool_index': request.json.get('tool_index', []),
            'tool_inner': len(request.json.get('tool_index', [])) > 0,
        }

    custom_fields = ['can_port', 'ip_address', 'port', 'changer_address', 'serial_port']
    for field in custom_fields:
        if field in request.json:
            settings[field] = request.json.get(field)

    # Normalize CAN port names (can0/can1) in case underscores are provided
    if 'can_port' in settings and settings['can_port'].startswith('can_'):
        settings['can_port'] = 'can' + settings['can_port'][4:]

    RobotModel.create(
        name=name,
        type=type,
        role=role,
        settings=settings,
        homepose=homepose
    )
    
    return {'status': 'success', 'message': 'Robot Created'}, 200


@robot_bp.route('/robot/<id>', methods=['PUT'])
def update_robot(id):
    name = request.json.get('name')
    type = request.json.get('type')

    robot = RobotModel.find(id)
    robot.name = name
    robot.type = type

    if 'homepose' in request.json:
        robot.homepose = request.json.get('homepose')

    if 'role' in request.json:
        robot.role = request.json.get('role')

    settings = robot.settings if robot.settings else {}
    if type == 'custom':
        settings = {
            'role': request.json.get('role', ''),
            'read_topic': request.json.get('read_topic', ''),
            'read_topic_msg': request.json.get('read_topic_msg', ''),
            'write_type': request.json.get('write_type', ''),
            'write_topic': request.json.get('write_topic', ''),
            'write_topic_msg': request.json.get('write_topic_msg', ''),
            'joint_names': request.json.get('joint_names', []),
            'joint_lower_bounds': request.json.get('joint_lower_bounds', []),
            'joint_upper_bounds': request.json.get('joint_upper_bounds', []),
            'tool_index': request.json.get('tool_index', []),
            'tool_inner': len(request.json.get('tool_index', [])) > 0,
        }

    custom_fields = ['can_port', 'ip_address', 'port', 'changer_address', 'serial_port']
    for field in custom_fields:
        if field in request.json:
            settings[field] = request.json.get(field)

    robot.settings = settings
    robot.save()
    return {'status': 'success', 'message': 'Robot Updated'}, 200


@robot_bp.route('/robot/<id>', methods=['DELETE'])
def delete_robot(id):
    robot = RobotModel.find(id)
    robot.hide = True
    robot.save()
    return {'status': 'success', 'message': 'Robot Deleted'}, 200


@robot_bp.route('/robot/<id>/:move_to', methods=['POST'])
def move_robot(id):
    data = request.json
    robot = RobotModel.find(id)
    if not robot:
        return {'status': 'error', 'message': 'Robot not found'}, 404

    goal_pos = data.get('goal_pos')
    step_size = data.get('step_size', 0.1)

    if not goal_pos:
        return {'status': 'error', 'message': 'Action is required'}, 400
    
    print(current_app.agents)
    agent = current_app.agents[int(id)]

    agent.move_to(goal_pos, step_size)
    time.sleep(3)
    return {'status': 'success', 'message': 'Robot moved'}, 200


@robot_bp.route('/robot/<id>/:subscribe_robot', methods=['POST'])
def subscribe_robot(id):
    int_id = int(id)
    func_name = 'subscribe_robot_' + str(id)

    # 기존 Agent가 있으면 재사용 (rclpy.spin 중 destroy_subscription → segfault 방지)
    existing_agent = current_app.agents.get(int_id)
    if existing_agent is not None:
        # background task만 재시작
        if func_name in current_app.pm.processes:
            return {'status': 'success', 'message': 'Already subscribed'}, 200
        current_app.pm.start_function(
            name=func_name,
            node=current_app.node,
            func=subscribe_robot_topic,
            socketio_instance=current_app.pm.socketio,
            agent=existing_agent,
        )
        return {'status': 'success', 'message': 'Subscribed to robot topic'}, 200

    robot = RobotModel.find(id).to_dict()
    agent = Agent(current_app.node, robot)
    current_app.agents[int_id] = agent

    current_app.pm.start_function(
        name=func_name,
        node=current_app.node,
        func=subscribe_robot_topic,
        socketio_instance=current_app.pm.socketio,
        agent=agent,
    )
    return {'status': 'success', 'message': 'Subscribed to robot topic'}, 200


@robot_bp.route('/robot/<id>/:unsubscribe_robot', methods=['POST'])
def unsubscribe_robot(id):
    current_app.pm.stop_function('subscribe_robot_' + str(id))
    # Agent와 ROS2 구독은 유지 — rclpy.spin 중 destroy하면 segfault
    # Agent는 다음 subscribe 시 재사용됨
    return {'status': 'success', 'message': 'Unsubscribed from robot topic'}, 200


def _ensure_can_interface(name: str):
    """
    Make sure the requested CAN interface exists.
    If 'canX' is missing but 'can_X' exists, rename it to 'canX'.
    """
    if not name.startswith("can"):
        return name
    # Already exists
    if os.path.exists(f"/sys/class/net/{name}"):
        return name
    # Try underscore variant
    num = name[3:]
    alt = f"can_{num}"
    if os.path.exists(f"/sys/class/net/{alt}"):
        try:
            subprocess.run(["ip", "link", "set", alt, "down"], check=True)
            subprocess.run(["ip", "link", "set", alt, "name", name], check=True)
            subprocess.run(["ip", "link", "set", name, "type", "can", "bitrate", "1000000"], check=True)
            subprocess.run(["ip", "link", "set", name, "up"], check=True)
            print(f"[PIPER] Renamed {alt} -> {name}")
        except Exception as e:
            print(f"[PIPER][WARN] Failed to rename {alt} -> {name}: {e}")
    return name
