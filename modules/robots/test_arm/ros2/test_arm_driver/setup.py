import os
import sys
from glob import glob
from setuptools import setup

package_name = 'test_arm_driver'


def _strip_flag(flag: str, drop_value: bool = False) -> None:
    if flag in sys.argv:
        idx = sys.argv.index(flag)
        if drop_value and idx + 1 < len(sys.argv) and not sys.argv[idx + 1].startswith('-'):
            sys.argv.pop(idx + 1)
        sys.argv.remove(flag)
    sys.argv[:] = [arg for arg in sys.argv if not arg.startswith(f"{flag}=")]


# colcon-pip 호환 — extras 이 setup.py 로 새는 경우 정리
_strip_flag('--editable')
_strip_flag('--uninstall')
_strip_flag('--build-directory', drop_value=True)


setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.urdf')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='Virtual 6-DOF arm driver (test fixture).',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'virtual_arm_node = test_arm_driver.virtual_arm_node:main',
        ],
    },
)
