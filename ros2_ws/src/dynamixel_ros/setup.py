import sys
from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'dynamixel_ros'

# Some build tools pass non-standard flags to setup.py; strip them so the
# distutils parser doesn't error out during build.
def _strip_flag(flag: str, drop_value: bool = False) -> None:
    """Remove a flag (and its following value if present) from sys.argv."""
    if flag in sys.argv:
        idx = sys.argv.index(flag)
        if drop_value and idx + 1 < len(sys.argv) and not sys.argv[idx + 1].startswith('-'):
            sys.argv.pop(idx + 1)
        sys.argv.remove(flag)
    # Handle the "--flag=value" form as well.
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
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),

        # [수정] 아래 두 줄을 추가하여 메시지 파일을 설치합니다.
        (os.path.join('share', package_name, 'msg'), glob('msg/*.msg')), # msg 파일 설치
        (os.path.join('lib', package_name), glob('dynamixel_ros/*.py')) # 메시지 모듈이 설치되도록 유도
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
            'dynamixel_node = dynamixel_ros.dynamixel_node:main',
        ],
    },
)
