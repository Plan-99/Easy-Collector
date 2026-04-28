from setuptools import find_packages, setup
import os
from glob import glob

package_name = "curobo_control"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "config"), glob("config/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    entry_points={
        "console_scripts": [
            "motor_in_box_service = curobo_control.motor_in_box_service:main",
            "pick_and_place_service = curobo_control.pick_and_place_service:main",
        ],
    },
)
