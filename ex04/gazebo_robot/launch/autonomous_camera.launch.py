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
            '-x', '0.5',
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
            # Depth camera topics
            '/depth_camera@sensor_msgs/msg/Image@gz.msgs.Image',
            '/depth_camera/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked',
            '/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo'
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
    
    # Depth camera obstacle avoider node
    depth_camera_avoider = Node(
        package='gazebo_robot',
        executable='depth_camera_avoider',
        name='depth_camera_avoider',
        output='screen',
        parameters=[
            {'max_linear_speed': 0.3},
            {'stop_distance': 1.2},
            {'slow_distance': 1.8},
            {'safe_distance': 2.5},
            {'camera_fov_horizontal': 87.0},  # degrees (from URDF: 1.51843645 rad ≈ 87°)
            {'camera_fov_vertical': 58.0},    # degrees (calculated from resolution)
            {'roi_width_ratio': 0.6},         # Use center 60% of image
            {'roi_height_ratio': 0.4},        # Use lower 40% of image
            {'min_depth': 0.3},               # meters
            {'max_depth': 5.0},               # meters
            {'control_rate': 10.0},
            {'obstacle_timeout': 2.0},
            {'use_sim_time': True},
            {'debug_mode': True}
        ]
    )
    
    # Image viewer for debugging (optional)
    image_viewer = Node(
        package='image_view',
        executable='image_view',
        name='depth_image_viewer',
        parameters=[{'use_sim_time': True}],
        remappings=[('image', '/depth_camera')],
        output='screen',
        condition=IfCondition(LaunchConfiguration('debug'))
    )
    
    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='true',
                              description='Open RViz.'),
        DeclareLaunchArgument('debug', default_value='false',
                              description='Show debug windows.'),
        
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
            period=2.0,
            actions=[bridge]
        ),
        
        TimerAction(
            period=2.0,
            actions=[depth_camera_avoider]
        ),
        
        TimerAction(
            period=2.0,
            actions=[rviz, image_viewer]
        )
    ])