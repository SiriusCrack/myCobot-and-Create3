from setuptools import setup
import os
from glob import glob

package_name = 'kris_morg_py'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Kristie Olds',
    maintainer_email='olds8434@vandals.uidaho.edu',
    description='Manipulating the MyCobot 320Pi via ROS2',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
		'move_action_server_exe = kris_morg_py.move_action_server:main',
		'with_roomba_exe = kris_morg_py.with_roomba:main',
		'gripper_action_server_exe = kris_morg_py.gripper_action_server:main',
        ],
    },
)
