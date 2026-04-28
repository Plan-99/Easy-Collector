"""Pick-and-Place 서비스 설정.

컨트롤러 생성 후 run_task_service()를 호출하는 헬퍼.
MoveIt/cuRobo 양쪽에서 동일하게 사용.

사용법 (entry point에서):
    from isaac_control_core.services.pick_and_place_service import run
    from isaac_robot_control.core import MoveItController  # 또는 CuroboController
    run(MoveItController)
"""

import rclpy
from isaac_control_core.robots import PiperConfig, PiperV100Config
from isaac_control_core.tasks import PickAndPlaceTask
from isaac_control_core.services.service_runner import run_task_service


def run(controller_class, args=None, **controller_kwargs):
    """Pick-and-Place 서비스 실행.

    Args:
        controller_class: MotionController 구현 클래스 (MoveItController 또는 CuroboController).
        args: ROS2 args.
        **controller_kwargs: controller_class에 전달할 추가 인자 (예: curobo_config_path).
    """
    rclpy.init(args=args)

    tmp_node = rclpy.create_node("_pnp_service_params")
    tmp_node.declare_parameter("pick_object", "RedCube")
    tmp_node.declare_parameter("place_target", "WhitePlate")
    tmp_node.declare_parameter("grasp_yaw", "auto")
    pick_object = tmp_node.get_parameter("pick_object").get_parameter_value().string_value
    place_target = tmp_node.get_parameter("place_target").get_parameter_value().string_value
    grasp_yaw_str = tmp_node.get_parameter("grasp_yaw").get_parameter_value().string_value
    tmp_node.destroy_node()

    # grasp_yaw 파싱: "auto", "horizontal", "vertical", 또는 숫자(도)
    if grasp_yaw_str in ("auto", "horizontal", "vertical"):
        grasp_yaw = grasp_yaw_str
    else:
        try:
            grasp_yaw = float(grasp_yaw_str)
        except ValueError:
            grasp_yaw = "auto"

    robot = PiperV100Config()
    controller = controller_class(
        robot_config=robot,
        object_names=[pick_object, place_target],
        node_name="pick_and_place_service_piper",
        **controller_kwargs,
    )
    controller.wait_for_ready()

    run_task_service(
        controller=controller,
        task_class=PickAndPlaceTask,
        service_name="/pick_and_place",
        service_label="Pick-and-Place",
        pick_object=pick_object,
        place_target=place_target,
        task_kwargs={"grasp_yaw": 30},
        timeout=120.0,
    )
