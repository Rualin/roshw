from setuptools import setup
import os
from glob import glob

package_name = 'gazebo_robot'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.world')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        ('share/' + package_name, ['robot.urdf.xacro', 'robot.gazebo.xacro']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ezhsluny',
    maintainer_email='y.basova@g.nsu.ru',
    description='Autonomous robot with obstacle avoidance',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'obstacle_avoider = gazebo_robot.obstacle_avoider:main',            
            'move_robot_imu = gazebo_robot.move_robot_imu:main',
        ],
    },
)