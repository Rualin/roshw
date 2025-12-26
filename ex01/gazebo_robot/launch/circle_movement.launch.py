import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import TimerAction


def generate_launch_description():
    robot_nodes = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_robot'), 'launch'),
                                       '/diff_drive.launch.py']),
    )

    return LaunchDescription([
        robot_nodes,
        
        # Параметры
        DeclareLaunchArgument(
            'radius', default_value="2.0",
            description='Radius of the circle.'
        ),
        DeclareLaunchArgument(
            'linear_speed', default_value="0.5",
            description='Linear speed for circle movement.'
        ),
        DeclareLaunchArgument(
            'direction', default_value='1.0',
            description='Direction of rotation (1.0 for CCW, -1.0 for CW).'
        ),
        
        # Узел для движения по кругу с задержкой
        TimerAction(
            period=3.0,
            actions=[
                Node(
                    package='gazebo_robot_circle',
                    executable='circle_movement',
                    name='circle_movement_node',
                    parameters=[
                        {'radius': LaunchConfiguration('radius')},
                        {'linear_speed': LaunchConfiguration('linear_speed')},
                        {'direction_of_rotation': LaunchConfiguration('direction')},
                        {'use_sim_time': True}
                    ],
                    output='screen'
                )
            ]
        ),
    ])