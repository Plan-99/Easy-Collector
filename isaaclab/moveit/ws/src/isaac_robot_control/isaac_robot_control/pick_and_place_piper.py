"""
Piper Pick-and-Place 엔트리포인트.

Piper 로봇으로 오브젝트를 잡아서 목표 위치에 올려놓는 예제.

사용법 (MoveIt2 Docker 컨테이너 내부):
    ros2 run isaac_robot_control pick_and_place_piper
    ros2 run isaac_robot_control pick_and_place_piper --ros-args -p pick_object:=BlueCube -p place_target:=GreenPlate
"""

import rclpy

from isaac_robot_control.robots import PiperConfig
from isaac_robot_control.core import RobotController
from isaac_robot_control.tasks import PickAndPlaceTask


def main(args=None):
    rclpy.init(args=args)

    tmp_node = rclpy.create_node("_pick_and_place_piper_params")
    tmp_node.declare_parameter("pick_object", "RedCube")
    tmp_node.declare_parameter("place_target", "WhitePlate")
    pick_object = tmp_node.get_parameter("pick_object").get_parameter_value().string_value
    place_target = tmp_node.get_parameter("place_target").get_parameter_value().string_value
    tmp_node.destroy_node()

    robot = PiperConfig()
    controller = RobotController(
        robot_config=robot,
        object_names=[pick_object, place_target],
        node_name="pick_and_place_piper",
    )

    controller.wait_for_ready()

    task = PickAndPlaceTask(
        controller=controller,
        pick_object=pick_object,
        place_target=place_target,
    )
    success = task.run()

    if success:
        controller.get_logger().info("작업 성공. 종료합니다.")
    else:
        controller.get_logger().error("작업 실패. 종료합니다.")

    controller.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
