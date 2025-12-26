from setuptools import find_packages, setup

package_name = 'gazebo_robot'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', [
            'launch/robot_lidar.launch.py',
            'launch/robot_camera.launch.py',
        ]),
        ('share/' + package_name + '/rviz', ['rviz/lidar.rviz']),
        ('share/' + package_name + '/rviz', ['rviz/camera.rviz']),
        ('share/' + package_name + '/worlds', ['worlds/empty.world']),
        ('share/' + package_name + '/worlds', ['worlds/gpu_lidar_sensor.sdf']),
        ('share/' + package_name + '/config', ['config/robot_bridge.yaml']),
        ('share/' + package_name + '/config', ['config/lidar_bridge.yaml']),
        ('share/' + package_name, ['robot.urdf.xacro']),
        ('share/' + package_name, ['robot.gazebo.xacro']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ezhsluny',
    maintainer_email='y.basova@g.nsu.ru',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
        ],
    },
)
