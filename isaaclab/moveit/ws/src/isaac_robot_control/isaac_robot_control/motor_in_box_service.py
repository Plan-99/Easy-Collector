"""MoveIt 기반 Motor-in-Box 서비스 entry point."""

from isaac_control_core.services import run_motor_in_box
from isaac_robot_control.core import MoveItController


def main(args=None):
    run_motor_in_box(MoveItController, args=args)


if __name__ == "__main__":
    main()
