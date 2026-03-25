import sys
from setuptools import setup

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

package_name = 'tm_mod_urdf'

setup(
    name=package_name,
    version='2.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Ken Tsai',
    maintainer_email='ken.tsai@tm-robot.com',
    description='tm_mod_urdf',
    license='BSD-3-Clause',
    # tests_require=['pytest'],
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            'modify_urdf = tm_mod_urdf.modify_urdf:main',
            'modify_xacro = tm_mod_urdf.modify_xacro:main'
        ],
    },
)
