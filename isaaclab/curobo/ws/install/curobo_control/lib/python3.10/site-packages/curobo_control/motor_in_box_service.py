"""cuRobo 기반 Motor-in-Box 서비스 entry point."""

from isaac_control_core.services import run_motor_in_box
from curobo_control.core import CuroboController


def main(args=None):
    run_motor_in_box(
        CuroboController,
        args=args,
        curobo_config_path="/root/ws/src/curobo_control/config/piper.yml",
    )


if __name__ == "__main__":
    main()
