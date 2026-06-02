import time
import multiprocessing as mp
import serial.tools.list_ports
from dynamixel_sdk import COMM_SUCCESS, PacketHandler, PortHandler
from ...env.dxl_controller import DxlController


def get_available_ports():
    """ 시스템의 시리얼 포트 목록을 반환합니다. """
    ports = serial.tools.list_ports.comports()
    target_ports = [p.device for p in ports if "ttyUSB" in p.device or "ttyACM" in p.device or "COM" in p.device]
    return target_ports


# ---------------------------------------------------------------------------
# Subprocess scan
# ---------------------------------------------------------------------------
# Dynamixel SDK의 ping()은 baudrate가 안 맞을 때 rxPacket 내부 busy loop에
# 갇혀 같은 프로세스에서 interrupt 가 불가능하다. closePort() 를 별 스레드에서
# 호출해도 안 깨진다. 그래서 (port, baudrate) 조합당 짧은 수명의 자식
# 프로세스로 격리하고, timeout/stop 시 SIGTERM/SIGKILL 로 강제 종료한다.

def _scan_subprocess(port_name, baudrate, protocol_ver, queue):
    """자식 프로세스 entry — 단일 (port, baudrate) 스캔 후 결과를 queue 에 넣음."""
    found = []
    portHandler = None
    try:
        portHandler = PortHandler(port_name)
        packetHandler = PacketHandler(protocol_ver)
        if not portHandler.openPort():
            queue.put([])
            return
        if not portHandler.setBaudRate(baudrate):
            queue.put([])
            return

        for dxl_id in range(0, 20):
            model_number, result, _ = packetHandler.ping(portHandler, dxl_id)
            if result == COMM_SUCCESS:
                found.append((dxl_id, model_number))
    except Exception:
        pass
    finally:
        if portHandler is not None:
            try:
                portHandler.closePort()
            except Exception:
                pass
        try:
            queue.put(found)
        except Exception:
            pass


def scan_ids_on_port(port_name, baudrate=4000000, protocol_ver=2.0, task_control=None, timeout=1.5):
    """ 특정 포트에서 연결된 다이나믹셀 ID들을 찾아 리스트로 반환.

    자식 프로세스에서 실제 스캔을 돌리고 (1) task_control['stop'] (2) timeout
    중 하나라도 발생하면 자식을 강제 종료해서 1.5초 안에 무조건 빠져나온다.
    """
    print(f"Scanning port: {port_name} ..., baudrate: {baudrate}")

    ctx = mp.get_context('fork')
    queue = ctx.Queue()
    p = ctx.Process(
        target=_scan_subprocess,
        args=(port_name, baudrate, protocol_ver, queue),
        daemon=True,
    )
    p.start()

    deadline = time.monotonic() + timeout
    aborted = False
    abort_reason = ''

    while p.is_alive():
        if task_control is not None and task_control.get('stop'):
            aborted = True
            abort_reason = 'stop signal'
            break
        if time.monotonic() > deadline:
            aborted = True
            abort_reason = 'scan budget exceeded'
            break
        time.sleep(0.05)

    if aborted:
        print(f"  [Abort] {abort_reason} — killing scan subprocess for {port_name} @ {baudrate}")
        try:
            p.terminate()
            p.join(timeout=0.5)
            if p.is_alive():
                p.kill()
                p.join(timeout=0.5)
        except Exception as e:
            print(f"  [WARN] failed to kill scan subprocess: {e}")
        print(f"Finished scanning port: {port_name}")
        return []

    p.join(timeout=0.5)

    try:
        found_raw = queue.get_nowait()
    except Exception:
        found_raw = []

    found_ids = []
    for dxl_id, model_number in found_raw:
        print(f"  [Found] ID: {dxl_id} (Model: {model_number})")
        found_ids.append(dxl_id)

    print(f"Finished scanning port: {port_name}")
    return found_ids


# ==========================================
# [Main Execution]
# ==========================================

SCAN_BAUDRATES = [4000000, 1000000]


def subscribe_dynamixel(socketio_instance, task_control, baudrate=None, skip_ports=None):
    # 1. 모든 포트 검색
    ports = get_available_ports()
    print(f"Detected Ports: {ports}")

    # 팔로워 로봇이 이미 점유 중인 포트는 스캔에서 제외 (open 시도 자체가 hang 유발).
    skip_set = set(skip_ports or [])
    if skip_set:
        filtered = [p for p in ports if p not in skip_set]
        skipped = [p for p in ports if p in skip_set]
        if skipped:
            print(f"Skip Ports (in use by follower): {skipped}")
        ports = filtered

    # baudrate 인자가 명시되면 그것만 시도, 미지정이면 4M→1M 순서로 폴백.
    baud_list = [baudrate] if baudrate else SCAN_BAUDRATES

    controllers = []

    # 2. 각 포트별로 baudrate 후보를 순차 시도 → 처음으로 dxl 찾은 baudrate 채택.
    for port in ports:
        if task_control.get('stop'):
            print("Scan aborted by stop signal.")
            break
        connected_ids = []
        matched_baud = None
        for b in baud_list:
            if task_control.get('stop'):
                break
            try:
                ids = scan_ids_on_port(port, baudrate=b, task_control=task_control)
            except Exception as e:
                print(f'[ERROR] scan failed on {port} @ {b}: {e}')
                continue
            if len(ids) > 0:
                connected_ids = ids
                matched_baud = b
                break

        if task_control.get('stop'):
            break

        if len(connected_ids) > 0 and matched_baud is not None:
            print(f'Connected Dynamixel IDs on {port} (@ {matched_baud}): {connected_ids}')

            # 3. DxlController 인스턴스 생성
            try:
                ctrl = DxlController(serial_port=port, dxl_ids=connected_ids, baudrate=matched_baud)
                controllers.append(ctrl)
            except Exception as e:
                print(f'[ERROR] Error creating controller for {port}: {str(e)}')
        else:
            print(f'[ERROR] No Dynamixels found on port {port}.')

    print(f"Total Controllers Created: {len(controllers)}")

    # 4. 생성된 컨트롤러로 현재 위치 읽기
    try:
        while not task_control['stop']:
            for ctrl in controllers:
                positions = ctrl.read_all_dynamixel()

                socketio_instance.emit('dynamixel_data', {
                    'port': ctrl.portHandler.getPortName(),
                    'values': positions
                })

            time.sleep(0.1)  # 10Hz

    except KeyboardInterrupt:
        print("\nTerminating...")
    finally:
        for ctrl in controllers:
            try:
                ctrl.close()
            except Exception:
                pass
