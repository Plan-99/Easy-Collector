import asyncio
import logging
from pymodbus.server import StartAsyncTcpServer
from pymodbus.device import ModbusDeviceIdentification
from pymodbus.datastore import (
    ModbusSequentialDataBlock,
    ModbusSlaveContext,
    ModbusServerContext,
)

# 1. 로그 설정
logging.basicConfig()
log = logging.getLogger()
log.setLevel(logging.INFO)

async def run_server():
    # 2. 데이터 저장소 설정 (0~100번지 레지스터 생성)
    # OnRobot 드라이버는 주로 Holding Register(hr)를 읽고 씁니다.
    # 초기값 0으로 설정
    block = ModbusSequentialDataBlock(0, [0] * 1000)
    
    store = ModbusSlaveContext(
        di=block, # Discrete Inputs
        co=block, # Coils
        hr=block, # Holding Registers (핵심)
        ir=block  # Input Registers
    )

    # 3. 서버 컨텍스트 (Single=True로 설정하여 모든 Slave ID 요청에 응답)
    context = ModbusServerContext(slaves=store, single=True)

    # 4. 장치 정보 설정 (옵션)
    identity = ModbusDeviceIdentification()
    identity.VendorName = 'Fake OnRobot'
    identity.ProductName = 'Simulated RG6 (Pymodbus v3.9)'
    identity.ModelName = 'Mock Server'
    identity.MajorMinorRevision = '3.9.2'

    print("--------------------------------------------------")
    print("🤖 가짜 로봇(Modbus Server v3)이 시작되었습니다.")
    print("📡 접속 주소: 0.0.0.0 (모든 IP 허용)")
    print("🔌 포트: 502")
    print("--------------------------------------------------")

    # 5. 서버 시작 (Async 방식)
    # address=("IP", Port) 튜플 형태
    await StartAsyncTcpServer(
        context=context, 
        identity=identity, 
        address=("0.0.0.0", 502)
    )

if __name__ == "__main__":
    try:
        # 비동기 루프 실행
        asyncio.run(run_server())
    except KeyboardInterrupt:
        print("서버가 종료되었습니다.")
    except PermissionError:
        print("❌ 에러: 포트 502번은 관리자 권한이 필요합니다.")
        print("   sudo python3 fake_robot_v3.py 명령어로 실행해주세요.")