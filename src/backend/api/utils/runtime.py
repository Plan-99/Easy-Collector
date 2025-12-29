from typing import Iterable, Mapping, MutableMapping


def attach_robot_runtime(robot: MutableMapping, processes: Iterable[str]) -> MutableMapping:
    """
    Enrich a robot dict with runtime info derived from ProcessManager state.

    - process_id: stable process name used across the app
    - status: 'on' if the robot process or its subscriber is running, otherwise 'off'
    """
    proc_set = set(processes or [])
    robot_id = robot.get("id")
    process_id = f"robot_{robot_id}"
    subscribe_id = f"subscribe_robot_{robot_id}"

    robot["process_id"] = process_id
    robot["status"] = "on" if process_id in proc_set or subscribe_id in proc_set else "off"

    return robot
