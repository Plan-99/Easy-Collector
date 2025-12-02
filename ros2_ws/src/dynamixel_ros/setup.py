from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'dynamixel_ros'

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
