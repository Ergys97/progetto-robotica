import os
from glob import glob

from setuptools import find_packages, setup
from setuptools.command.install import install

package_name = 'progetto_robotica'


class RosInstallCommand(install):
    def finalize_options(self):
        super().finalize_options()
        self.install_scripts = os.path.join(self.install_base, 'lib', package_name)


setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*_launch.py')),
        (os.path.join('share', package_name, 'web', 'templates'), glob('web/templates/*.html')),
        (os.path.join('share', package_name, 'web', 'static', 'css'), glob('web/static/css/*.css')),
        (os.path.join('share', package_name, 'web', 'static', 'js'), glob('web/static/js/*.js')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ergys',
    maintainer_email='ergysperdeda97@gmail.com',
    description='ROS 2 Teleoperation and Web Dashboard for Unitree G1 in MuJoCo',
    license='Apache-2.0',
    cmdclass={'install': RosInstallCommand},
    entry_points={
        'console_scripts': [
            'mujoco_sim = progetto_robotica.mujoco_sim:main',
            'web_teleop = progetto_robotica.web_teleop:main',
            'replay_eval = progetto_robotica.replay_eval:main',
        ],
    },
)
