"""
Piper Pick-and-Place 반복 벤치마크.

N번 반복하면서 매 시행마다 환경을 리셋하고 pick-and-place를 실행합니다.
최종 성공률을 출력합니다.

사용법 (MoveIt2 Docker 컨테이너 내부):
    ros2 run isaac_robot_control benchmark_piper
    ros2 run isaac_robot_control benchmark_piper --ros-args -p n_trials:=20
    ros2 run isaac_robot_control benchmark_piper --ros-args -p pick_object:=GreenCube -p place_target:=WhitePlate -p n_trials:=10
"""

import time
import rclpy
from std_srvs.srv import Trigger

from isaac_robot_control.robots import PiperConfig
from isaac_robot_control.core import RobotController
from isaac_robot_control.tasks import PickAndPlaceTask


def reset_env(node) -> bool:
    """시뮬레이션 환경 리셋 서비스 호출."""
    client = node.create_client(Trigger, "/simulation/reset_env")
    if not client.wait_for_service(timeout_sec=10.0):
        node.get_logger().error(
            "리셋 서비스(/simulation/reset_env)를 찾을 수 없습니다. "
            "IsaacSim을 최신 base_env.py로 재시작했는지 확인하세요."
        )
        return False

    future = client.call_async(Trigger.Request())
    rclpy.spin_until_future_complete(node, future, timeout_sec=10.0)

    if future.result() is not None and future.result().success:
        node.get_logger().info("환경 리셋 완료. 안정화 대기...")
        time.sleep(3.0)
        return True

    node.get_logger().error("환경 리셋 실패.")
    return False


def main(args=None):
    rclpy.init(args=args)

    tmp_node = rclpy.create_node("_benchmark_piper_params")
    tmp_node.declare_parameter("pick_object", "RedCube")
    tmp_node.declare_parameter("place_target", "WhitePlate")
    tmp_node.declare_parameter("n_trials", 10)
    pick_object = tmp_node.get_parameter("pick_object").get_parameter_value().string_value
    place_target = tmp_node.get_parameter("place_target").get_parameter_value().string_value
    n_trials = tmp_node.get_parameter("n_trials").get_parameter_value().integer_value
    tmp_node.destroy_node()

    robot = PiperConfig()
    controller = RobotController(
        robot_config=robot,
        object_names=[pick_object, place_target],
        node_name="benchmark_piper",
    )
    controller.wait_for_ready()

    results = []
    log = controller.get_logger()

    log.info("=" * 60)
    log.info(f"  Benchmark 시작: {n_trials}회 반복")
    log.info(f"  pick: {pick_object}, place: {place_target}")
    log.info("=" * 60)

    for i in range(n_trials):
        log.info(f"\n{'='*60}")
        log.info(f"  Trial {i+1}/{n_trials}")
        log.info(f"{'='*60}")

        # 환경 리셋 (첫 번째 시행은 이미 초기 상태이므로 스킵 가능)
        if i > 0:
            if not reset_env(controller):
                log.error(f"Trial {i+1}: 환경 리셋 실패, 건너뜀")
                results.append(False)
                continue

            # 리셋 후 오브젝트 위치가 갱신될 때까지 대기
            for _ in range(20):
                rclpy.spin_once(controller, timeout_sec=0.1)

        task = PickAndPlaceTask(
            controller=controller,
            pick_object=pick_object,
            place_target=place_target,
        )
        success = task.run()
        results.append(success)

        status = "성공" if success else "실패"
        log.info(f"\n  Trial {i+1} 결과: {status}")

    # 최종 결과 출력
    n_success = sum(results)
    n_fail = len(results) - n_success
    rate = n_success / len(results) * 100 if results else 0

    log.info("\n" + "=" * 60)
    log.info("  Benchmark 결과")
    log.info("=" * 60)
    log.info(f"  총 시행: {len(results)}")
    log.info(f"  성공: {n_success}")
    log.info(f"  실패: {n_fail}")
    log.info(f"  성공률: {rate:.1f}%")
    for i, r in enumerate(results):
        log.info(f"    Trial {i+1}: {'성공' if r else '실패'}")
    log.info("=" * 60)

    controller.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
