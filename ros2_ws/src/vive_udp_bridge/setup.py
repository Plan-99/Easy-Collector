import sys
from setuptools import find_packages, setup

package_name = 'vive_udp_bridge'

def _strip_flag(flag: str, drop_value: bool = False) -> None:
    if flag in sys.argv:
        idx = sys.argv.index(flag)
        if drop_value and idx + 1 < len(sys.argv) and not sys.argv[idx + 1].startswith('-'):
            sys.argv.pop(idx + 1)
        sys.argv.remove(flag)
    sys.argv[:] = [arg for arg in sys.argv if not arg.startswith(f"{flag}=")]

_strip_flag('--editable')
_strip_flag('--uninstall')
_strip_flag('--build-directory', drop_value=True)

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'udp_json_to_ros2 = vive_udp_bridge.udp_json_to_ros2:main',
        ],
    },
)
