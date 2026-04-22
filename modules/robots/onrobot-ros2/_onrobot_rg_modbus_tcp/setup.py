import sys
from setuptools import find_packages, setup

package_name = 'onrobot_rg_modbus_tcp'

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
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Makány András',
    maintainer_email='andras.makany@irob.uni-obuda.hu',
    description='A stack to communicate with OnRobot RG grippers using the Modbus/TCP protocol. Based on Takuya Kiyokawa\'s package.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
