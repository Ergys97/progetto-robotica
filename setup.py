from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'progetto_robotica'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[py|yaml]*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ergys',
    maintainer_email='ergys@todo.todo',
    description='ROS 2 Teleoperation and Web Dashboard for Unitree G1 in MuJoCo',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mujoco_sim = progetto_robotica.mujoco_sim:main',
            'web_teleop = progetto_robotica.web_teleop:main',
        ],
    },
)
