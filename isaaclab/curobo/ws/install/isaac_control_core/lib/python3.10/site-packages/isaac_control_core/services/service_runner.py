"""범용 태스크 서비스 러너.

모션 플래너에 독립적인 서비스 루프를 제공한다.
서비스 콜을 받으면 환경 리셋 → 태스크 실행 → 결과 반환.

Gripper 스케일 브릿지:
    /simulation/joint_states  (sim 스케일) → /joint_states  (real 스케일)
    /simulation/joint_command (sim 스케일) → /joint_command (real 스케일)

사용법:
    run_task_service(
        controller=controller,
        task_class=PickAndPlaceTask,
        service_name="/pick_and_place",
        service_label="Pick-and-Place",
        pick_object="RedCube",
        place_target="WhitePlate",
    )
"""

import time
import threading

import rclpy
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger

from isaac_control_core.core.task import BaseTask
from isaac_control_core.core.motion_controller import MotionController
from isaac_control_core.utils.skills import RobotSkills


class GripperScaleBridge:
    """Gripper joint를 sim 스케일 → real 스케일로 변환하여 republish.

    /simulation/joint_states  → /joint_states  (gripper: sim→real)
    /simulation/joint_command → /joint_command (gripper: sim→real)
    """

    def __init__(self, node, gripper_joint_names: list[str],
                 sim_max: float, real_max: float):
        self._gripper_names = set(gripper_joint_names)
        self._gripper_names.add("joint8")  # mimic joint
        self._scale = real_max / sim_max if sim_max > 0 else 1.0

        self._js_pub = node.create_publisher(JointState, "/joint_states", 10)
        self._cmd_pub = node.create_publisher(JointState, "/joint_command", 10)

        node.create_subscription(
            JointState, "/simulation/joint_states", self._on_joint_states, 10)
        node.create_subscription(
            JointState, "/simulation/joint_command", self._on_joint_command, 10)

    def _convert(self, msg: JointState) -> JointState:
        """gripper joint만 sim→real 스케일로 변환한 복사본 반환."""
        out = JointState()
        out.header = msg.header
        out.name = list(msg.name)
        out.position = list(msg.position)
        out.velocity = list(msg.velocity)
        out.effort = list(msg.effort)

        for i, name in enumerate(out.name):
            if name in self._gripper_names:
                out.position[i] = abs(out.position[i]) * self._scale
        return out

    def _on_joint_states(self, msg: JointState):
        self._js_pub.publish(self._convert(msg))

    def _on_joint_command(self, msg: JointState):
        self._cmd_pub.publish(self._convert(msg))


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


def run_task_service(
    controller: MotionController,
    task_class: type[BaseTask],
    service_name: str,
    service_label: str,
    pick_object: str,
    place_target: str,
    task_kwargs: dict | None = None,
    timeout: float = 120.0,
    open_gripper_after: bool = False,
):
    """범용 태스크 서비스 루프.

    Args:
        controller: 모션 컨트롤러 (이미 wait_for_ready 완료된 상태).
        task_class: BaseTask 서브클래스.
        service_name: ROS2 서비스 이름 (예: "/pick_and_place").
        service_label: 로그에 표시할 이름 (예: "Pick-and-Place").
        pick_object: 잡을 오브젝트 이름.
        place_target: 놓을 대상 이름.
        task_kwargs: task_class 생성 시 추가 인자 (예: {"stack_offset": 0.05}).
        timeout: 태스크 타임아웃 (초).
        open_gripper_after: 태스크 완료 후 그리퍼 열기 여부.
    """
    log = controller.get_logger()
    trial_count = 0

    trigger = threading.Event()
    result_ready = threading.Event()
    task_result = {"success": False, "message": ""}

    service_node = rclpy.create_node(f"{service_name.strip('/')}_server")

    # Gripper 스케일 브릿지 (sim→real republish)
    robot = controller.robot_config
    gripper_names = robot.gripper.joint_names if robot.gripper else []
    sim_max_map = {"piper": 0.035, "piper_v100": 0.05}
    sim_max = sim_max_map.get(robot.name, 0.035)
    real_max = robot.gripper.open_width if robot.gripper else 0.085
    _bridge = GripperScaleBridge(service_node, gripper_names, sim_max, real_max)
    log.info(f"[GripperBridge] sim={sim_max} → real={real_max}")

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

    service_node.create_service(Trigger, service_name, service_cb)

    spin_thread = threading.Thread(
        target=rclpy.spin, args=(service_node,), daemon=True
    )
    spin_thread.start()

    log.info("=" * 50)
    log.info(f"  {service_label} 서비스 대기 중")
    log.info(f"  pick: {pick_object}, target: {place_target}")
    log.info(f"  호출: ros2 service call {service_name} std_srvs/srv/Trigger")
    log.info("=" * 50)

    try:
        while rclpy.ok():
            if not trigger.wait(timeout=1.0):
                continue
            trigger.clear()
            log.info("[DEBUG] 메인 스레드: trigger 수신, 작업 시작")

            if not reset_env(controller):
                task_result["success"] = False
                task_result["message"] = "환경 리셋 실패"
                log.info("[DEBUG] 메인 스레드: 리셋 실패, result_ready.set()")
                result_ready.set()
                continue

            for _ in range(20):
                rclpy.spin_once(controller, timeout_sec=0.1)

            # 매 trial 시작 전 그리퍼 열기 (이전 실패 시 닫힌 상태 방지)
            RobotSkills(controller).open_gripper()

            log.info(f"[DEBUG] 메인 스레드: {service_label} 시작")
            kwargs = {"controller": controller, "pick_object": pick_object, "place_target": place_target}
            if task_kwargs:
                kwargs.update(task_kwargs)
            task = task_class(**kwargs)

            task_done = threading.Event()
            task_success = [False]

            def _run_task():
                try:
                    task_success[0] = task.run()
                finally:
                    if open_gripper_after:
                        RobotSkills(controller).open_gripper()
                task_done.set()

            task_thread = threading.Thread(target=_run_task, daemon=True)
            task_thread.start()
            finished = task_done.wait(timeout=timeout)

            if finished:
                success = task_success[0]
                log.info(f"[DEBUG] 메인 스레드: task.run() 완료, success={success}")
            else:
                success = False
                log.error(f"[TIMEOUT] {timeout}초 초과 — 작업 실패 처리")

            task_result["success"] = success
            task_result["message"] = "성공" if success else (f"타임아웃 ({timeout}초)" if not finished else "실패")
            log.info(f"[DEBUG] 메인 스레드: result_ready.set() 호출")
            result_ready.set()
            log.info(f"[DEBUG] 메인 스레드: result_ready.set() 완료, 다음 대기")
    except KeyboardInterrupt:
        pass

    service_node.destroy_node()
    controller.destroy_node()
    rclpy.shutdown()
