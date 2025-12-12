import time
import serial.tools.list_ports
import dynamixel_sdk as dxl # 다이나믹셀 SDK 사용
from dynamixel_sdk import COMM_SUCCESS, PacketHandler, PortHandler
from ...env.dxl_controller import DxlController


def get_available_ports():
    """ 시스템의 시리얼 포트 목록을 반환합니다. """
    ports = serial.tools.list_ports.comports()
    # 리눅스/맥/윈도우 환경에 따라 필터링 (USB 장치 위주)
    target_ports = [p.device for p in ports if "ttyUSB" in p.device or "ttyACM" in p.device or "COM" in p.device]
    return target_ports

def scan_ids_on_port(port_name, baudrate=57600, protocol_ver=2.0):
    """ 특정 포트에서 연결된 다이나믹셀 ID들을 찾아서 리스트로 반환합니다. """
    found_ids = []
    
    # 임시 핸들러 생성
    portHandler = PortHandler(port_name)
    packetHandler = PacketHandler(protocol_ver)

    try:
        if not portHandler.openPort():
            return []
        if not portHandler.setBaudRate(baudrate):
            portHandler.closePort()
            return []
        
        print(f"Scanning port: {port_name} ...")
        
        # ID 0부터 20까지만 스캔 (필요시 252까지 늘리세요)
        for dxl_id in range(0, 20): 
            model_number, result, error = packetHandler.ping(portHandler, dxl_id)
            if result == COMM_SUCCESS:
                print(f"  [Found] ID: {dxl_id} (Model: {model_number})")
                found_ids.append(dxl_id)
                
    finally:
        print(f"Finished scanning port: {port_name}")
        # 중요: DxlController가 포트를 쓸 수 있게 반드시 닫아줘야 함
        portHandler.closePort()
        
    return found_ids

# ==========================================
# [Main Execution]
# ==========================================

def subscribe_dynamixel(socketio_instance, task_control, baudrate=57600):
    # 1. 모든 포트 검색
    ports = get_available_ports()
    print(f"Detected Ports: {ports}")

    controllers = []

    # 2. 각 포트별로 ID 스캔 및 컨트롤러 생성
    for port in ports:
        # 해당 포트의 다이나믹셀 ID 스캔
        connected_ids = scan_ids_on_port(port, baudrate=baudrate)
        
        if len(connected_ids) > 0:
            socketio_instance.emit('log_start_leader_robot', {
                'log': f'Connected Dynamixel IDs on {port}: {connected_ids}',
                'type': 'stdout'
            })
            
            # 3. DxlController 인스턴스 생성
            try:
                ctrl = DxlController(serial_port=port, dxl_ids=connected_ids, baudrate=baudrate)
                controllers.append(ctrl)
            except Exception as e:
                socketio_instance.emit('log_start_leader_robot', {
                    'log': f'[ERROR] Error creating controller for {port}: {str(e)}',
                    'type': 'stderr'
                })
        else:
            socketio_instance.emit('log_start_leader_robot', {
                'log': f'No Dynamixels found on port {port}.',
                'type': 'stdout'
            })

    socketio_instance.emit('log_start_leader_robot', {
        'log': f'Total Controllers Created: {len(controllers)}',
        'type': 'stdout'
    })

    # 4. 생성된 컨트롤러로 현재 위치 읽기
    try:
        while not task_control['stop']: # 데모를 위해 반복 읽기 (Ctrl+C로 종료)
            for ctrl in controllers:
                positions = ctrl.read_all_dynamixel()
                
                socketio_instance.emit('dynamixel_data', {
                    'port': ctrl.portHandler.getPortName(),
                    'values': positions
                })
            
            time.sleep(0.1) # 10Hz
            
    except KeyboardInterrupt:
        print("\nTerminating...")
        for ctrl in controllers:
            ctrl.close()
