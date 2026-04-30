"""
Piper Stack-Cube 서비스 노드.

서비스 콜을 받으면 환경 리셋 → RedCube를 GreenCube 위에 쌓기 → 결과 반환.
노드는 종료하지 않고 다음 서비스 콜을 대기합니다.

구조:
  - service_node: 서비스 서버 전용 (별도 스레드에서 spin)
  - controller: 로봇 제어 전용 (메인 스레드에서 spin_until_future_complete 사용)
  두 노드를 분리하여 executor 충돌을 방지합니다.

사용법 (MoveIt2 Docker 컨테이너 내부):
    ros2 run isaac_robot_control stack_cube_service_piper
    ros2 run isaac_robot_control stack_cube_service_piper --ros-args -p pick_object:=RedCube -p place_target:=GreenCube

서비스 호출:
    ros2 service call /stack_cube std_srvs/srv/Trigger
"""

import time
import threading
import rclpy
from std_srvs.srv import Trigger

from isaac_robot_control.robots import PiperConfig
from isaac_robot_control.core import RobotController
from isaac_robot_control.tasks import StackCubeTask


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


def main(args=None):
    rclpy.init(args=args)

    tmp_node = rclpy.create_node("_stack_cube_service_params")
    tmp_node.declare_parameter("pick_object", "RedCube")
    tmp_node.declare_parameter("place_target", "GreenCube")
    pick_object = tmp_node.get_parameter("pick_object").get_parameter_value().string_value
    place_target = tmp_node.get_parameter("place_target").get_parameter_value().string_value
    tmp_node.destroy_node()

    robot = PiperConfig()
    controller = RobotController(
        robot_config=robot,
        object_names=[pick_object, place_target],
        node_name="stack_cube_service_piper",
    )
    controller.wait_for_ready()

    log = controller.get_logger()
    trial_count = 0

    # 서비스 콜 → 메인 스레드 동기화용 이벤트
    trigger = threading.Event()
    result_ready = threading.Event()
    task_result = {"success": False, "message": ""}

    # 서비스 전용 노드 (별도 스레드에서 spin)
    service_node = rclpy.create_node("stack_cube_service_server")

    def service_cb(request, response):
        nonlocal trial_count
        trial_count += 1
        log.info(f"[DEBUG] service_cb 진입 (Trial #{trial_count}), thread={threading.current_thread().name}")

        # 메인 스레드에 작업 요청
        result_ready.clear()
        trigger.set()
        log.info(f"[DEBUG] trigger.set() 완료, result_ready.wait() 시작...")

        # 메인 스레드 작업 완료 대기 (최대 5분)
        waited = result_ready.wait(timeout=300.0)
        log.info(f"[DEBUG] result_ready.wait() 반환: waited={waited}, timeout={'NO' if waited else 'YES'}")

        response.success = task_result["success"]
        response.message = task_result["message"]
        log.info(f"[DEBUG] 응답 전송 직전: success={response.success}, message={response.message}")
        log.info(f"Trial #{trial_count} 결과: {response.message}")
        return response

    service_node.create_service(Trigger, "/stack_cube", service_cb)

    # 서비스 노드를 별도 스레드에서 spin
    spin_thread = threading.Thread(
        target=rclpy.spin, args=(service_node,), daemon=True
    )
    spin_thread.start()

    log.info("=" * 50)
    log.info(f"  Stack Cube 서비스 대기 중")
    log.info(f"  pick: {pick_object}, target: {place_target}")
    log.info(f"  호출: ros2 service call /stack_cube std_srvs/srv/Trigger")
    log.info("=" * 50)

    # 메인 루프: 서비스 콜 대기 → 작업 실행 → 결과 반환
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

            log.info("[DEBUG] 메인 스레드: StackCubeTask 시작")
            task = StackCubeTask(
                controller=controller,
                pick_object=pick_object,
                place_target=place_target,
            )

            # 타임아웃 처리: 별도 스레드에서 실행, 30초 제한
            task_done = threading.Event()
            task_success = [False]

            def _run_task():
                task_success[0] = task.run()
                task_done.set()

            task_thread = threading.Thread(target=_run_task, daemon=True)
            task_thread.start()
            finished = task_done.wait(timeout=30.0)

            if finished:
                success = task_success[0]
                log.info(f"[DEBUG] 메인 스레드: task.run() 완료, success={success}")
            else:
                success = False
                log.error(f"[TIMEOUT] 30초 초과 — 작업 실패 처리")

            task_result["success"] = success
            task_result["message"] = "성공" if success else ("타임아웃 (30초)" if not finished else "실패")
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
