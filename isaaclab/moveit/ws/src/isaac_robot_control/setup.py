from setuptools import find_packages, setup
import os
from glob import glob

package_name = "isaac_robot_control"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml") + glob("config/*.rviz") + glob("config/*.srdf")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    entry_points={
        "console_scripts": [
            "trajectory_bridge = isaac_robot_control.trajectory_bridge:main",
            "trajectory_bridge_piper = isaac_robot_control.trajectory_bridge_piper:main",
            "demo_control = isaac_robot_control.demo_control:main",
            "pick_and_place = isaac_robot_control.pick_and_place:main",
            "pick_and_place_piper = isaac_robot_control.pick_and_place_piper:main",
            "benchmark_piper = isaac_robot_control.benchmark_piper:main",
            "pick_and_place_service_piper = isaac_robot_control.pick_and_place_service_piper:main",
            "stack_cube_service_piper = isaac_robot_control.stack_cube_service_piper:main",
            "sponge_in_box_service_piper = isaac_robot_control.sponge_in_box_service_piper:main",
            "motor_in_box_service = isaac_robot_control.motor_in_box_service:main",
        ],
    },
)
