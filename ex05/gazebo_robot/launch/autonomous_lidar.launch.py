#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions import Node
from launch.actions import TimerAction
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    pkg_project_bringup = get_package_share_directory('gazebo_robot')
    
    urdf_path = os.path.join(pkg_project_bringup, 'robot.urdf.xacro')
    robot_desc = ParameterValue(Command(['xacro ', urdf_path]), value_type=str)
    world_file = os.path.join(pkg_project_bringup, 'worlds', 'empty.world')
    
    # Gazebo simulation
    gazebo = ExecuteProcess(
        cmd=['gz', 'sim', '-r', world_file],
        output='screen',
        additional_env={'GZ_SIM_RESOURCE_PATH': os.path.join(pkg_project_bringup, 'worlds')}
    )
    
    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[
            {'robot_description': robot_desc},
            {'use_sim_time': True}
        ]
    )
    
    # Spawn robot
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'robot',
            '-topic', 'robot_description',
            '-x', '1.0',
            '-y', '0.5',
            '-z', '0.8',
            '-Y', '-1.58'
        ],
        output='screen',
    )
    
    # Bridge for communication between ROS and Gazebo
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry',
            '/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
            '/model/robot/pose@geometry_msgs/msg/Pose@gz.msgs.Pose',
            '/clock@rosgraph_msgs/msg/Clock@gz.msgs.Clock'
        ],
        output='screen',
        parameters=[{'use_sim_time': True}]
    )
    
    # RViz visualization
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(pkg_project_bringup, 'rviz', 'autonomous.rviz')],
        condition=IfCondition(LaunchConfiguration('rviz')),
        parameters=[{'use_sim_time': True}],
        output='screen'
    )
    
    # Obstacle avoider node
    obstacle_avoider = Node(
        package='gazebo_robot',
        executable='obstacle_avoider',
        name='obstacle_avoider',
        output='screen',
        parameters=[
            os.path.join(pkg_project_bringup, 'config', 'autonomous_params.yaml'),
            {'use_sim_time': True}
        ]
    )
    
    # Optional: TF static broadcaster
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )
    
    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='true',
                              description='Open RViz.'),
        DeclareLaunchArgument('gui', default_value='true',
                              description='Open Gazebo GUI.'),
        
        gazebo,
        
        TimerAction(
            period=1.0,
            actions=[robot_state_publisher]
        ),
        
        TimerAction(
            period=2.0,
            actions=[spawn_entity]
        ),
        
        TimerAction(
            period=3.0,
            actions=[bridge, static_tf]
        ),
        
        TimerAction(
            period=4.0,
            actions=[obstacle_avoider]
        ),
        
        TimerAction(
            period=5.0,
            actions=[rviz]
        )
    ])