"""cuRobo 기반 Pick-and-Place 서비스 entry point."""

from isaac_control_core.services import run_pick_and_place
from curobo_control.core import CuroboController


def main(args=None):
    run_pick_and_place(
        CuroboController,
        args=args,
        curobo_config_path="/root/ws/src/curobo_control/config/piper.yml",
    )


if __name__ == "__main__":
    main()
