import dynamixel_sdk as dxl # 다이나믹셀 SDK 사용
from dynamixel_sdk import GroupSyncRead, COMM_SUCCESS
import threading
import time


class DxlController:
    def __init__(self, serial_port, dxl_ids, gripper_dxl_ids=[]):
        self.portHandler = dxl.PortHandler(serial_port)  # 다이나믹셀 포트
        self.packetHandler = dxl.PacketHandler(2.0)         # 프로토콜 2.0 사용
        self.port_lock = threading.Lock()
        self.dxl_ids = dxl_ids
        self.gripper_dxl_ids = gripper_dxl_ids

        # 포트 열기 및 Baud rate 설정
        if not self.portHandler.openPort():
            print("Failed to open the port")
            return
        if not self.portHandler.setBaudRate(57600):
            print("Failed to set baudrate")
            return
        
        self.ADDR_PRESENT_POSITION = 132  # 예시: X-Series의 Present Position 주소
        self.LEN_PRESENT_POSITION = 4     # 예시: X-Series의 Present Position 길이 (bytes)
        
        self.syncRead = GroupSyncRead(self.portHandler, self.packetHandler,
                                      self.ADDR_PRESENT_POSITION, self.LEN_PRESENT_POSITION)
        
        for dxl_id in self.dxl_ids:
            add_param_result = self.syncRead.addParam(dxl_id)
            if add_param_result != True:
                print(f"[ID:{dxl_id}] GroupSyncRead addParam failed")

    def enable_torque(self):
        torque_enable_address = 64  # MX, X 시리즈 기준
        for index, dxl_id in enumerate(self.dxl_ids):
            print(f"Enabling torque for DXL ID: {dxl_id}")
            print(f"Gripper DXL IDs: {self.gripper_dxl_ids}")
            if dxl_id in self.gripper_dxl_ids:
                continue
            with self.port_lock:
                dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, dxl_id, torque_enable_address, 1)
            if dxl_comm_result != dxl.COMM_SUCCESS:
                print(f"Torque Enable Comm Error: {self.packetHandler.getTxRxResult(dxl_comm_result)}")
                return
            elif dxl_error != 0:
                print(f"Torque Enable Error: {self.packetHandler.getRxPacketError(dxl_error)}")
                return
            
    def remove_torque(self):
        torque_enable_address = 64  # MX, X 시리즈 기준
        for index, dxl_id in enumerate(self.dxl_ids):
            with self.port_lock:
                dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, dxl_id, torque_enable_address, 0)
            if dxl_comm_result != dxl.COMM_SUCCESS:
                print(f"Torque Remove Comm Error: {self.packetHandler.getTxRxResult(dxl_comm_result)}")
                return
            elif dxl_error != 0:
                print(f"Torque Remove Error: {self.packetHandler.getRxPacketError(dxl_error)}")
                return
            
            
    def read_dynamixel(self, dxl_id):
        present_position_address = 132
        with self.port_lock:
            position, dxl_comm_result, dxl_error = self.packetHandler.read4ByteTxRx(self.portHandler, dxl_id, present_position_address)

        if dxl_comm_result != dxl.COMM_SUCCESS:
            raise Exception("Failed to read position")
        elif dxl_error != 0:
            raise Exception("Error reading position")
        
        if position > 2147483647:
            position -= 4294967296  # 2**32 를 빼서 원래 음수 값을 구함
        return position
    
    def read_all_dynamixel(self):
        """ GroupSyncRead를 사용해 모든 다이나믹셀 위치를 한 번에 읽어옵니다. """
        
        positions = []
        
        # 1. 단 한 번의 통신으로 모든 데이터를 요청하고 받음
        dxl_comm_result = self.syncRead.txRxPacket()
        if dxl_comm_result != COMM_SUCCESS:
            print(f"SyncRead txRxPacket error: {self.packetHandler.getTxRxResult(dxl_comm_result)}")
            # 통신 실패 시, 오류를 알리거나 비어 있는 리스트 반환
            return [0] * len(self.dxl_ids) # 임시로 0 리스트 반환

        # 2. 수신된 데이터에서 각 ID의 값을 *로컬*에서 추출
        for dxl_id in self.dxl_ids:
            # 개별 모터에서 데이터를 잘 수신했는지 확인
            if self.syncRead.isAvailable(dxl_id, self.ADDR_PRESENT_POSITION, self.LEN_PRESENT_POSITION):
                position = self.syncRead.getData(dxl_id, self.ADDR_PRESENT_POSITION, self.LEN_PRESENT_POSITION)
                positions.append(position)
            else:
                print(f"[ID:{dxl_id}] SyncRead data not available")
                positions.append(0) # 오류 발생 시 0 또는 이전 값으로 대체
                        
        return positions
    

    def move_controller(self, goal_position):
        goal_position_address = 116  # Goal Position address (X 시리즈)
        velocity_address = 112       # Profile Velocity address

        success = True

        self.enable_torque()

        # 1. 속도 설정 (각 모터 동일한 속도)
        for dxl_id in self.dxl_ids:
            if dxl_id in self.gripper_dxl_ids:
                continue  # 그리퍼 모터는 건너뜀
            with self.port_lock:
                dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(self.portHandler, dxl_id, velocity_address, 10)
            if dxl_comm_result != dxl.COMM_SUCCESS:
                raise Exception(f"Velocity Comm Error: {self.packetHandler.getTxRxResult(dxl_comm_result)}")
            elif dxl_error != 0:
                raise Exception(f"Velocity Error: {self.packetHandler.getRxPacketError(dxl_error)}")

        # 2. GroupSyncWrite 객체 생성
        groupSyncWrite = dxl.GroupSyncWrite(self.portHandler, self.packetHandler, goal_position_address, 4)

        current_positions = self.read_all_dynamixel()
        
        # 3. 목표 위치 패킷 구성
        for index, dxl_id in enumerate(self.dxl_ids):
            if dxl_id in self.gripper_dxl_ids:
                continue  # 그리퍼 모터는 건너뜀
            print(current_positions, index, dxl_id)
            original_goal = goal_position[index]
            current_pos = current_positions[index]
            new_goal = calculate_shortest_path_goal(current_pos, original_goal)

            param_goal_position = [
                new_goal & 0xFF,
                (new_goal >> 8) & 0xFF,
                (new_goal >> 16) & 0xFF,
                (new_goal >> 24) & 0xFF
            ]
            groupSyncWrite.addParam(dxl_id, bytearray(param_goal_position))

        # 4. 한번에 전송
        dxl_comm_result = groupSyncWrite.txPacket()
        if dxl_comm_result != dxl.COMM_SUCCESS:
            print("GroupSyncWrite failed:", self.packetHandler.getTxRxResult(dxl_comm_result))
            raise Exception("GroupSyncWrite failed")
        
        groupSyncWrite.clearParam()

        # 5. 모든 모터가 도달할 때까지 대기
        reached = [False] * len(self.dxl_ids)

        while not all(reached):
            for i, dxl_id in enumerate(self.dxl_ids):
                if dxl_id in self.gripper_dxl_ids:
                    reached[i] = True
                    
                if reached[i]:
                    continue

                position = self.read_dynamixel(dxl_id)

                print(f"[ID:{dxl_id}] GoalPos:{goal_position[i]}  PresPos:{position}")
                if abs(goal_position[i] - position) < 40 or abs(goal_position[i] - position + 4096) < 40 or abs(goal_position[i] - position - 4096) < 40:
                    print(f"[ID:{dxl_id}] Reached goal position.")
                    reached[i] = True

            time.sleep(0.1)

        return success
    
def calculate_shortest_path_goal(current_pos, goal_pos, max_pos=4096):
    """
    현재 위치에서 목표 위치까지의 최단 경로를 계산하여 새로운 목표 위치를 반환합니다.
    확장 위치 제어 모드에서 사용됩니다.
    """
    print(f"Current Position: {current_pos}, Goal Position: {goal_pos}")
    diff = goal_pos - current_pos
    half_pos = max_pos / 2

    if diff > half_pos:
        return goal_pos - max_pos
    elif diff < -half_pos:
        return goal_pos + max_pos
    
    return goal_pos