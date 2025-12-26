#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Включаем основной launch-файл для робота
    robot_nodes = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('gazebo_robot'), 'launch', 'diff_drive.launch.py')
        ]),
        launch_arguments={
            'rviz': 'true'
        }.items()
    )

    return LaunchDescription([
        # Основные узлы робота
        robot_nodes,
        
        # Параметры для движения по линии с танцем
        DeclareLaunchArgument(
            'forward_distance',
            default_value='2.0',
            description='Distance to move forward (meters)'
        ),
        DeclareLaunchArgument(
            'backward_distance',
            default_value='2.0',
            description='Distance to move backward (meters)'
        ),
        DeclareLaunchArgument(
            'linear_speed',
            default_value='0.6',
            description='Linear speed for moving forward/backward (m/s)'
        ),
        DeclareLaunchArgument(
            'rotation_speed',
            default_value='1.2',
            description='Rotation speed for dancing and turning (rad/s)'
        ),
        DeclareLaunchArgument(
            'dance_cycles',
            default_value='2',
            description='Number of dance cycles (left-right swings)'
        ),
        DeclareLaunchArgument(
            'dance_angle_deg',
            default_value='60.0',
            description='Dance rotation angle in degrees (max 60)'
        ),
        
        # Узел движения по линии с танцем
        TimerAction(
            period=5.0,
            actions=[
                Node(
                    package='gazebo_robot_circle',
                    executable='line_dance',
                    name='line_dance',
                    output='screen',
                    emulate_tty=True,
                    parameters=[
                        {
                            'forward_distance': LaunchConfiguration('forward_distance'),
                            'backward_distance': LaunchConfiguration('backward_distance'),
                            'linear_speed': LaunchConfiguration('linear_speed'),
                            'rotation_speed': LaunchConfiguration('rotation_speed'),
                            'dance_cycles': LaunchConfiguration('dance_cycles'),
                            'dance_angle': LaunchConfiguration('dance_angle_deg')
                        }
                    ]
                )
            ]
        )
    ])