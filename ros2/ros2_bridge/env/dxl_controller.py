import dynamixel_sdk as dxl # 다이나믹셀 SDK 사용
from dynamixel_sdk import GroupSyncRead, COMM_SUCCESS
import threading
import time


class DxlController:
    def __init__(self, serial_port, dxl_ids, gripper_dxl_ids=[], baudrate=4000000):
        print(f"Initializing DxlController on port {serial_port} with IDs {dxl_ids}")
        print(baudrate)
        self.portHandler = dxl.PortHandler(serial_port)  # 다이나믹셀 포트
        self.packetHandler = dxl.PacketHandler(2.0)         # 프로토콜 2.0 사용
        self.port_lock = threading.Lock()
        self.dxl_ids = dxl_ids
        self.gripper_dxl_ids = gripper_dxl_ids

        # 포트 열기 및 Baud rate 설정
        if not self.portHandler.openPort():
            print("[ERROR] Failed to open the port")
            raise Exception("Failed to open the port")
        if not self.portHandler.setBaudRate(baudrate):
            print("[ERROR] Failed to set baudrate")
            raise Exception("Failed to set baudrate")
        
        self.ADDR_PRESENT_POSITION = 132  # 예시: X-Series의 Present Position 주소
        self.LEN_PRESENT_POSITION = 4     # 예시: X-Series의 Present Position 길이 (bytes)
        
        self.syncRead = GroupSyncRead(self.portHandler, self.packetHandler,
                                      self.ADDR_PRESENT_POSITION, self.LEN_PRESENT_POSITION)
        
        for dxl_id in self.dxl_ids:
            add_param_result = self.syncRead.addParam(dxl_id)
            if add_param_result != True:
                print(f"[ERROR] [ID:{dxl_id}] GroupSyncRead addParam failed")

        self.closed = False
        self.controlled = False

    def enable_torque(self):
        torque_enable_address = 64  # MX, X 시리즈 기준
        self.controlled = True
        print(f"[enable_torque] dxl_ids={self.dxl_ids} gripper_dxl_ids={self.gripper_dxl_ids}", flush=True)
        enabled = []
        for index, dxl_id in enumerate(self.dxl_ids):
            if dxl_id in self.gripper_dxl_ids:
                continue
            with self.port_lock:
                dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, dxl_id, torque_enable_address, 1)
            if dxl_comm_result != dxl.COMM_SUCCESS:
                print(f"Torque Enable Comm Error: {self.packetHandler.getTxRxResult(dxl_comm_result)} dxl_id={dxl_id}", flush=True)
                return
            elif dxl_error != 0:
                print(f"Torque Enable Error: {self.packetHandler.getRxPacketError(dxl_error)} dxl_id={dxl_id}", flush=True)
                return
            enabled.append(dxl_id)
            time.sleep(0.01)
        print(f"[enable_torque] enabled={enabled}", flush=True)
            
    def remove_torque(self):
        import traceback
        torque_enable_address = 64  # MX, X 시리즈 기준
        print(f"[remove_torque] called on dxl_ids={self.dxl_ids}", flush=True)
        traceback.print_stack()
        for index, dxl_id in enumerate(self.dxl_ids):
            with self.port_lock:
                dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, dxl_id, torque_enable_address, 0)
            if dxl_comm_result != dxl.COMM_SUCCESS:
                print(f"Torque Remove Comm Error: {self.packetHandler.getTxRxResult(dxl_comm_result)}")
                return
            elif dxl_error != 0:
                print(f"Torque Remove Error: {self.packetHandler.getRxPacketError(dxl_error)}")
                return
        self.controlled = False
        
            
            
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
        max_retries = 20
        retry_count = 0
        
        with self.port_lock:
            for attempt in range(max_retries):
                retry_count += 1
                positions = []
                try:
                    dxl_comm_result = self.syncRead.txRxPacket()
                    
                    if dxl_comm_result == COMM_SUCCESS:
                        all_data_available = True
                        for dxl_id in self.dxl_ids:
                            # getData 호출 전에 안전하게 검사
                            if self.syncRead.isAvailable(dxl_id, self.ADDR_PRESENT_POSITION, self.LEN_PRESENT_POSITION):
                                # 여기서 에러가 날 가능성이 큼
                                position = self.syncRead.getData(dxl_id, self.ADDR_PRESENT_POSITION, self.LEN_PRESENT_POSITION)
                                
                                if position > 2147483647:
                                    position -= 4294967296
                                positions.append({'id': dxl_id, 'position': position})
                            else:
                                all_data_available = False
                                break
                        
                        if all_data_available:
                            return positions
                except Exception as e:
                    # 여기서 에러가 잡히면 어떤 ID 읽다가 죽었는지 알 수 있음
                    print(f"[SDK Error] ID 읽기 중 예외 발생: {e} (시도 {attempt+1})")
                
                time.sleep(0.001) # 0.001보다 조금 더 여유를 줌

            # 실패 시 마지막으로 읽었던 값이나 안전한 기본값 반환
            print("[ERROR] Failed to read all positions after multiple attempts.")
            return 0

    def close(self):
        self.portHandler.closePort()
    

    def move_controller(self, goal_joint_dicts):
        goal_position_address = 116  # Goal Position address (X 시리즈)
        velocity_address = 112       # Profile Velocity address

        success = True

        self.enable_torque()

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
        
        # # 3. 목표 위치 패킷 구성
        # for index, dxl_id in enumerate(self.dxl_ids):
        #     if dxl_id in self.gripper_dxl_ids:
        #         continue  # 그리퍼 모터는 건너뜀
        #     print(current_positions, index, dxl_id)
        #     original_goal = goal_position[index]
        #     current_pos = current_positions[index]
        #     new_goal = calculate_shortest_path_goal(current_pos, original_goal)

        #     param_goal_position = [
        #         new_goal & 0xFF,
        #         (new_goal >> 8) & 0xFF,
        #         (new_goal >> 16) & 0xFF,
        #         (new_goal >> 24) & 0xFF
        #     ]
        #     groupSyncWrite.addParam(dxl_id, bytearray(param_goal_position))

        for goal_joint_dict in goal_joint_dicts:
            dxl_id = goal_joint_dict['dxl_id']
            if dxl_id in self.gripper_dxl_ids:
                continue  # 그리퍼 모터는 건너뜀
            original_goal = goal_joint_dict['dxl_goal_position']
            current_pos = self.read_dynamixel(dxl_id)
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


        while not all(reached) and not self.closed:
            for i, goal_joint_dict in enumerate(goal_joint_dicts):
                dxl_id = goal_joint_dict['dxl_id']
                if dxl_id in self.gripper_dxl_ids:
                    reached[i] = True
                    
                if reached[i]:
                    continue

                position = self.read_dynamixel(dxl_id)
                goal_position = goal_joint_dict['dxl_goal_position']

                print(f"[ID:{dxl_id}] GoalPos:{goal_position}  PresPos:{position}")
                if abs(goal_position - position) < 20 or abs(goal_position - position + 4096) < 20 or abs(goal_position - position - 4096) < 20:
                    print(f"[ID:{dxl_id}] Reached goal position.")
                    reached[i] = True

            time.sleep(0.1)

        return success
    
def calculate_shortest_path_goal(current_pos, goal_pos, max_pos=4096):
    """
    현재 위치에서 목표 위치까지의 최단 경로를 계산하여 새로운 목표 위치를 반환합니다.
    확장 위치 제어 모드에서 사용됩니다.
    """
    diff = goal_pos - current_pos
    half_pos = max_pos / 2

    if diff > half_pos:
        return goal_pos - max_pos
    elif diff < -half_pos:
        return goal_pos + max_pos
    
    return goal_pos