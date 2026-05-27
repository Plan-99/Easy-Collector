import dynamixel_sdk as dxl # 다이나믹셀 SDK 사용
from dynamixel_sdk import GroupSyncRead, COMM_SUCCESS
import threading
import time


# Dynamixel X-Series control table addresses
ADDR_OPERATING_MODE = 11        # EEPROM, 1 byte — torque off needed for write
ADDR_CURRENT_LIMIT = 38         # EEPROM, 2 bytes — torque off needed for write
ADDR_TORQUE_ENABLE = 64         # RAM, 1 byte
ADDR_STATUS_RETURN_LEVEL = 68   # RAM, 1 byte
ADDR_GOAL_CURRENT = 102         # RAM, 2 bytes signed
ADDR_GOAL_POSITION = 116        # RAM, 4 bytes
ADDR_PRESENT_POSITION = 132     # RAM, 4 bytes

# Operating modes
OP_MODE_CURRENT = 0
OP_MODE_VELOCITY = 1
OP_MODE_POSITION = 3
OP_MODE_EXT_POSITION = 4
OP_MODE_CURRENT_POSITION = 5
OP_MODE_PWM = 16


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

        self.ADDR_PRESENT_POSITION = ADDR_PRESENT_POSITION
        self.LEN_PRESENT_POSITION = 4

        self.syncRead = GroupSyncRead(self.portHandler, self.packetHandler,
                                      self.ADDR_PRESENT_POSITION, self.LEN_PRESENT_POSITION)

        for dxl_id in self.dxl_ids:
            add_param_result = self.syncRead.addParam(dxl_id)
            if add_param_result != True:
                print(f"[ERROR] [ID:{dxl_id}] GroupSyncRead addParam failed")

        self.closed = False
        self.controlled = False

        # Status Return Level 강제 정규화. ROBOTIS lerobot OMX leader 같은 fast-sync
        # 설정에서는 모터의 Status Return Level 을 0 으로 깔아둬서 WRITE 명령에
        # status packet 이 안 오고 "There is no status packet" 으로 죽는다. RAM 영역이라
        # 우리 측에서 매 세션 시작 시 2 (= 모든 명령에 응답) 로 끌어올린다.
        self.ensure_status_return_level(target=2)

        # 모터별 현재 Operating Mode 캐시 — 리더 텔레옵에서 Position/Current 분기에 사용.
        # 시작 시점 한번 읽어두고, set_operating_mode 호출 시 캐시 업데이트.
        self.operating_modes = {}
        for dxl_id in self.dxl_ids:
            try:
                self.operating_modes[dxl_id] = self.read_operating_mode(dxl_id)
            except Exception as e:
                print(f"[WARN] could not read operating_mode for ID {dxl_id}: {e}", flush=True)
                self.operating_modes[dxl_id] = OP_MODE_POSITION
        print(f"[DxlController] operating_modes={self.operating_modes}", flush=True)

    # ------------------------------------------------------------------
    # Operating mode / current primitives (mode-aware leader teleop 용)
    # ------------------------------------------------------------------

    def read_operating_mode(self, dxl_id):
        with self.port_lock:
            value, result, err = self.packetHandler.read1ByteTxRx(
                self.portHandler, dxl_id, ADDR_OPERATING_MODE
            )
        if result != COMM_SUCCESS:
            raise Exception(f"read_operating_mode comm {result} for ID {dxl_id}")
        if err != 0:
            raise Exception(f"read_operating_mode err {err} for ID {dxl_id}")
        return value

    def read_status_return_level(self, dxl_id):
        """Status Return Level (addr 68) 읽기. 0/1 인 모터는 READ 자체가 실패 →
        그 경우 None 반환."""
        try:
            with self.port_lock:
                value, result, err = self.packetHandler.read1ByteTxRx(
                    self.portHandler, dxl_id, ADDR_STATUS_RETURN_LEVEL
                )
            if result != COMM_SUCCESS or err != 0:
                return None
            return value
        except Exception:
            return None

    def ensure_status_return_level(self, target=2):
        """모든 모터의 Status Return Level 을 target 이상으로 끌어올린다.

        Status Return Level 의미:
          0 = PING 만 응답 (READ/WRITE 응답 없음)
          1 = PING + READ 응답 (WRITE 응답 없음)
          2 = 모든 명령에 응답 (기본값)

        ROBOTIS lerobot OMX leader 등 fast-sync write 셋업에서는 0 으로 깔아두는데,
        그 상태로 우리 워크플로우(writeTxRx 사용) 가 들어오면 "There is no status packet"
        으로 매번 죽는다. RAM 영역이라 매 세션 시작 시 2 로 끌어올리고, 전원
        사이클 시 다시 기본값으로 돌아간다.

        level=0 인 모터에는 write 응답이 안 오므로 write1ByteTxOnly 를 쓴다.
        """
        for dxl_id in self.dxl_ids:
            current = self.read_status_return_level(dxl_id)
            if current is not None and current >= target:
                continue
            try:
                with self.port_lock:
                    # write1ByteTxOnly: status packet 안 기다림 → level=0 모터에도 동작.
                    self.packetHandler.write1ByteTxOnly(
                        self.portHandler, dxl_id, ADDR_STATUS_RETURN_LEVEL, target
                    )
                time.sleep(0.02)
                verify = self.read_status_return_level(dxl_id)
                print(
                    f"[status_return_level] ID {dxl_id}: {current} → {verify}",
                    flush=True,
                )
            except Exception as e:
                print(f"[WARN] failed to raise status_return_level for ID {dxl_id}: {e}", flush=True)

    def _write_torque(self, dxl_id, value):
        with self.port_lock:
            result, err = self.packetHandler.write1ByteTxRx(
                self.portHandler, dxl_id, ADDR_TORQUE_ENABLE, value
            )
        if result != COMM_SUCCESS:
            raise Exception(f"torque write comm {result} for ID {dxl_id}")
        if err != 0:
            raise Exception(f"torque write err {err} for ID {dxl_id}")

    def enable_torque_for_ids(self, ids):
        for dxl_id in ids:
            self._write_torque(dxl_id, 1)
            time.sleep(0.005)

    def disable_torque_for_ids(self, ids):
        for dxl_id in ids:
            self._write_torque(dxl_id, 0)
            time.sleep(0.005)

    def set_operating_mode(self, dxl_id, mode):
        """Operating Mode 변경. EEPROM 영역이라 torque off 후 써야 한다.

        호출 후에는 모터의 torque 가 OFF 상태로 남는다. 다시 켜고 싶으면 별도로
        enable_torque_for_ids 를 호출할 것.
        """
        try:
            self._write_torque(dxl_id, 0)
        except Exception:
            pass
        time.sleep(0.005)
        with self.port_lock:
            result, err = self.packetHandler.write1ByteTxRx(
                self.portHandler, dxl_id, ADDR_OPERATING_MODE, mode
            )
        if result != COMM_SUCCESS:
            raise Exception(f"set_operating_mode comm {result} for ID {dxl_id}")
        if err != 0:
            raise Exception(f"set_operating_mode err {err} for ID {dxl_id}")
        self.operating_modes[dxl_id] = mode

    def write_goal_current(self, dxl_id, current):
        """2-byte signed Goal_Current."""
        if current < 0:
            current = current + 65536  # 2's complement
        with self.port_lock:
            result, err = self.packetHandler.write2ByteTxRx(
                self.portHandler, dxl_id, ADDR_GOAL_CURRENT, current
            )
        if result != COMM_SUCCESS:
            raise Exception(f"goal_current comm {result} for ID {dxl_id}")
        if err != 0:
            raise Exception(f"goal_current err {err} for ID {dxl_id}")

    def write_current_limit(self, dxl_id, limit):
        """2-byte unsigned Current_Limit. EEPROM — torque off 필요."""
        try:
            self._write_torque(dxl_id, 0)
        except Exception:
            pass
        time.sleep(0.005)
        with self.port_lock:
            result, err = self.packetHandler.write2ByteTxRx(
                self.portHandler, dxl_id, ADDR_CURRENT_LIMIT, limit
            )
        if result != COMM_SUCCESS:
            raise Exception(f"current_limit comm {result} for ID {dxl_id}")
        if err != 0:
            raise Exception(f"current_limit err {err} for ID {dxl_id}")

    def write_goal_position(self, dxl_id, position):
        with self.port_lock:
            result, err = self.packetHandler.write4ByteTxRx(
                self.portHandler, dxl_id, ADDR_GOAL_POSITION, position
            )
        if result != COMM_SUCCESS:
            raise Exception(f"goal_position comm {result} for ID {dxl_id}")
        if err != 0:
            raise Exception(f"goal_position err {err} for ID {dxl_id}")

    def is_current_mode(self, dxl_id):
        return self.operating_modes.get(dxl_id) == OP_MODE_CURRENT

    def is_position_mode(self, dxl_id):
        return self.operating_modes.get(dxl_id) in (OP_MODE_POSITION, OP_MODE_EXT_POSITION)

    def is_current_position_mode(self, dxl_id):
        return self.operating_modes.get(dxl_id) == OP_MODE_CURRENT_POSITION

    # ------------------------------------------------------------------
    # Legacy bulk torque helpers (기존 호출처 호환 — 모든 ID 일괄 처리)
    # ------------------------------------------------------------------

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