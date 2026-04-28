"""
Piper Sponge-in-Box 서비스 노드.

서비스 콜을 받으면 환경 리셋 → BlueSponge를 WhiteBox에 넣기 → 결과 반환.
노드는 종료하지 않고 다음 서비스 콜을 대기합니다.

구조:
  - service_node: 서비스 서버 전용 (별도 스레드에서 spin)
  - controller: 로봇 제어 전용 (메인 스레드에서 spin_until_future_complete 사용)
  두 노드를 분리하여 executor 충돌을 방지합니다.

사용법 (MoveIt2 Docker 컨테이너 내부):
    ros2 run isaac_robot_control sponge_in_box_service_piper
    ros2 run isaac_robot_control sponge_in_box_service_piper --ros-args -p pick_object:=BlueSponge -p place_target:=WhiteBox

서비스 호출:
    ros2 service call /sponge_in_box std_srvs/srv/Trigger
"""

import time
import math
import threading
import rclpy
from std_srvs.srv import Trigger

from isaac_robot_control.robots import PiperConfig
from isaac_robot_control.core import RobotController
from isaac_robot_control.utils.skills import RobotSkills


def reset_env(node) -> bool:
    """시뮬레이션 환경 리셋 서비스 호출."""
    client = node.create_client(Trigger, "/simulation/reset_env")
    if not client.wait_for_service(timeout_sec=10.0):
        node.get_logger().error("리셋 서비스(/simulation/reset_env)를 찾을 수 없습니다.")
        return False

    future = client.call_async(Trigger.Request())
    rclpy.spin_until_future_complete(node, future, timeout_sec=10.0)

    if future.result() is not None and future.result().success:
        node.get_logger().info("환경 리셋 완료. 안정화 대기...")
        time.sleep(1.5)
        return True

    node.get_logger().error("환경 리셋 실패.")
    return False


def run_sponge_in_box(controller, pick_object, place_target) -> bool:
    """BlueSponge를 WhiteBox에 넣는 태스크."""
    skills = RobotSkills(controller)
    log = controller.get_logger()

    pick_pos = skills.get_object_position(pick_object)
    place_pos = skills.get_object_position(place_target)
    if pick_pos is None or place_pos is None:
        return False

    cx, cy, cz = pick_pos
    px, py, pz = place_pos

    log.info("=" * 50)
    log.info("  Sponge-in-Box 시작")
    log.info(f"  {pick_object}:    ({cx:.3f}, {cy:.3f}, {cz:.3f})")
    log.info(f"  {place_target}: ({px:.3f}, {py:.3f}, {pz:.3f})")
    log.info("=" * 50)

    initial_joints = skills.get_current_arm_joints()

    # 2) Pick
    log.info(f"\n[Pick] {pick_object}")
    if not skills.pick(cx, cy, cz):
        log.error("Pick 실패!")
        return False

    # 3) Place on WhiteBox (상대좌표 보정)
    log.info(f"\n[Place in] {place_target}")
    if not skills.place_on_object(pick_object, place_target, stack_offset=0.05):
        log.error("Place 실패!")
        return False

    # 복귀
    log.info("\n[완료] 초기 위치로 복귀")
    skills.go_to_joints(initial_joints)

    log.info("\n" + "=" * 50)
    log.info("  Sponge-in-Box 모션 완료!")
    log.info("=" * 50)
    return True


def evaluate(controller, pick_object, place_target) -> bool:
    """pick_object가 place_target 위/안에 있는지 판정."""
    log = controller.get_logger()

    for _ in range(10):
        rclpy.spin_once(controller, timeout_sec=0.1)

    positions = controller.object_positions
    if pick_object not in positions or place_target not in positions:
        log.error("evaluate: 오브젝트 위치를 읽을 수 없습니다.")
        return False

    pick_pos = positions[pick_object]
    place_pos = positions[place_target]

    dx = pick_pos[0] - place_pos[0]
    dy = pick_pos[1] - place_pos[1]
    xy_dist = math.sqrt(dx * dx + dy * dy)
    z_diff = pick_pos[2] - place_pos[2]

    log.info(f"[판정] {pick_object}: ({pick_pos[0]:.3f}, {pick_pos[1]:.3f}, {pick_pos[2]:.3f})")
    log.info(f"[판정] {place_target}: ({place_pos[0]:.3f}, {place_pos[1]:.3f}, {place_pos[2]:.3f})")
    log.info(f"[판정] XY 거리: {xy_dist:.3f}m, Z 차이: {z_diff:.3f}m")

    success = xy_dist < 0.08 and z_diff > 0
    log.info(f"[판정] {'성공' if success else '실패'}")
    return success


def main(args=None):
    rclpy.init(args=args)

    tmp_node = rclpy.create_node("_sponge_box_service_params")
    tmp_node.declare_parameter("pick_object", "BlueCube")
    tmp_node.declare_parameter("place_target", "WhiteBox")
    pick_object = tmp_node.get_parameter("pick_object").get_parameter_value().string_value
    place_target = tmp_node.get_parameter("place_target").get_parameter_value().string_value
    tmp_node.destroy_node()

    robot = PiperConfig()
    controller = RobotController(
        robot_config=robot,
        object_names=[pick_object, place_target],
        node_name="sponge_in_box_service_piper",
    )
    controller.wait_for_ready()

    log = controller.get_logger()
    trial_count = 0

    trigger = threading.Event()
    result_ready = threading.Event()
    task_result = {"success": False, "message": ""}

    service_node = rclpy.create_node("sponge_box_service_server")

    def service_cb(request, response):
        nonlocal trial_count
        trial_count += 1
        log.info(f"[DEBUG] service_cb 진입 (Trial #{trial_count}), thread={threading.current_thread().name}")

        result_ready.clear()
        trigger.set()
        log.info(f"[DEBUG] trigger.set() 완료, result_ready.wait() 시작...")

        waited = result_ready.wait(timeout=300.0)
        log.info(f"[DEBUG] result_ready.wait() 반환: waited={waited}, timeout={'NO' if waited else 'YES'}")

        response.success = task_result["success"]
        response.message = task_result["message"]
        log.info(f"[DEBUG] 응답 전송 직전: success={response.success}, message={response.message}")
        log.info(f"Trial #{trial_count} 결과: {response.message}")
        return response

    service_node.create_service(Trigger, "/sponge_in_box", service_cb)

    spin_thread = threading.Thread(
        target=rclpy.spin, args=(service_node,), daemon=True
    )
    spin_thread.start()

    log.info("=" * 50)
    log.info(f"  Sponge-in-Box 서비스 대기 중")
    log.info(f"  pick: {pick_object}, target: {place_target}")
    log.info(f"  호출: ros2 service call /sponge_in_box std_srvs/srv/Trigger")
    log.info("=" * 50)

    try:
        while rclpy.ok():
            if not trigger.wait(timeout=1.0):
                continue
            trigger.clear()
            log.info("[DEBUG] 메인 스레드: trigger 수신, 작업 시작")

            # 환경 리셋
            if not reset_env(controller):
                task_result["success"] = False
                task_result["message"] = "환경 리셋 실패"
                log.info("[DEBUG] 메인 스레드: 리셋 실패, result_ready.set()")
                result_ready.set()
                continue

            # 리셋 후 오브젝트 위치 갱신 대기
            for _ in range(20):
                rclpy.spin_once(controller, timeout_sec=0.1)

            log.info("[DEBUG] 메인 스레드: Sponge-in-Box 시작")

            # 타임아웃 처리: 별도 스레드에서 실행, 60초 제한
            task_done = threading.Event()
            task_success = [False]

            def _run_task():
                ok = run_sponge_in_box(controller, pick_object, place_target)
                if ok:
                    task_success[0] = evaluate(controller, pick_object, place_target)
                task_done.set()

            task_thread = threading.Thread(target=_run_task, daemon=True)
            task_thread.start()
            finished = task_done.wait(timeout=100)

            if finished:
                success = task_success[0]
                log.info(f"[DEBUG] 메인 스레드: task 완료, success={success}")
            else:
                success = False
                log.error(f"[TIMEOUT] 60초 초과 — 작업 실패 처리")

            task_result["success"] = success
            task_result["message"] = "성공" if success else ("타임아웃 (60초)" if not finished else "실패")
            log.info(f"[DEBUG] 메인 스레드: result_ready.set() 호출")
            result_ready.set()
            log.info(f"[DEBUG] 메인 스레드: result_ready.set() 완료, 다음 대기")
    except KeyboardInterrupt:
        pass

    service_node.destroy_node()
    controller.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
