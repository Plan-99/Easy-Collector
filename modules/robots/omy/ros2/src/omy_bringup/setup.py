from glob import glob
import os

from setuptools import find_packages
from setuptools import setup

package_name = 'omy_bringup'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config/omy_3m'),
            glob('config/omy_3m/*')),
        (os.path.join('share', package_name, 'config/omy_f3m'),
            glob('config/omy_f3m/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Pyo',
    maintainer_email='pyo@robotis.com',
    description='EasyTrainer bringup for ROBOTIS OMY.',
    license='Apache 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [],
    },
)
