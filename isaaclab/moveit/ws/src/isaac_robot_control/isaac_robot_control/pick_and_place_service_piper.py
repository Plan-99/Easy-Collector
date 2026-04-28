"""MoveIt 기반 Pick-and-Place 서비스 entry point."""

from isaac_control_core.services import run_pick_and_place
from isaac_robot_control.core import MoveItController


def main(args=None):
    run_pick_and_place(MoveItController, args=args)


if __name__ == "__main__":
    main()
