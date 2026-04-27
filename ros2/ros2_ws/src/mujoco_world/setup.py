import os
import sys
from glob import glob
from setuptools import setup


def _strip_flag(flag: str, drop_value: bool = False) -> None:
    """Make setup.py tolerant of pip's editable / pip-internal flags."""
    if flag in sys.argv:
        idx = sys.argv.index(flag)
        if drop_value and idx + 1 < len(sys.argv) and not sys.argv[idx + 1].startswith('-'):
            sys.argv.pop(idx + 1)
        sys.argv.remove(flag)
    sys.argv[:] = [arg for arg in sys.argv if not arg.startswith(f"{flag}=")]


_strip_flag('--editable')
_strip_flag('--uninstall')
_strip_flag('--build-directory', drop_value=True)


package_name = 'mujoco_world'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'assets'), glob('assets/*.xml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='EasyTrainer',
    maintainer_email='user@todo.todo',
    description='MuJoCo simulation world for EasyTrainer tutorial mode.',
    license='Apache License 2.0',
    entry_points={
        'console_scripts': [
            'mujoco_world_node = mujoco_world.mujoco_world_node:main',
        ],
    },
)
