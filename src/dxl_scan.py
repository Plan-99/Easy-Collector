import sys
import glob
import serial.tools.list_ports
from dynamixel_sdk import * 

def search_dynamixel_port(target_baudrate=57600, protocol_version=2.0, target_id=1):
    """
    연결된 모든 시리얼 포트를 검사하여 다이나믹셀이 응답하는 포트를 찾습니다.
    """
    # 1. 시스템의 모든 시리얼 포트 리스트 가져오기
    ports = serial.tools.list_ports.comports()
    available_ports = [port.device for port in ports]
    
    print(f"검색된 시스템 포트 목록: {available_ports}")

    packetHandler = PacketHandler(protocol_version)

    for port_name in available_ports:
        # Linux의 경우 보통 /dev/ttyUSB* 또는 /dev/ttyACM* 입니다.
        if "ttyUSB" not in port_name and "ttyACM" not in port_name and "COM" not in port_name:
            continue

        print(f"검사 중: {port_name} ...")
        
        try:
            portHandler = PortHandler(port_name)
            
            # 2. 포트 열기 시도
            if portHandler.openPort():
                # 3. 통신 속도 설정
                if portHandler.setBaudRate(target_baudrate):
                    # 4. Ping 보내기 (특정 ID 또는 ID를 모를 경우 브로드캐스트 ID 254 사용 가능하지만, 보통 1번으로 테스트)
                    # 주의: ID를 전혀 모른다면 ID 0~252까지 루프를 돌려야 할 수도 있습니다.
                    model_number, result, error = packetHandler.ping(portHandler, target_id)
                    
                    if result == COMM_SUCCESS:
                        print(f"\n[성공] 다이나믹셀 발견!")
                        print(f" - 포트: {port_name}")
                        print(f" - ID: {target_id}")
                        print(f" - 모델 번호: {model_number}")
                        
                        portHandler.closePort()
                        return port_name
                
                portHandler.closePort()
            else:
                print(f" - 포트 열기 실패 (권한 문제 등)")
                
        except Exception as e:
            print(f" - 에러 발생: {e}")

    print("\n[실패] 다이나믹셀이 연결된 포트를 찾지 못했습니다.")
    return None

if __name__ == "__main__":
    # 사용 중인 모터의 설정에 맞게 수정하세요 (보통 기본값: 57600, ID: 1)
    found_port = search_dynamixel_port(target_baudrate=57600, protocol_version=2.0, target_id=1)