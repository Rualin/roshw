import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.actions import TimerAction


def generate_launch_description():
    pkg_project_bringup = get_package_share_directory("cat_robot_bring_up")

    diff_drive_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_project_bringup, "launch", "diff_drive.launch.py")))
    
    movement_node = Node(
        package="cat_robot_bring_up",
        executable="circle_publisher",
        name="circle_publisher",
        output="screen",
        remappings=[
            ("/cmd_vel", "/catbot/cmd_vel"),
        ]
    )

    return LaunchDescription([
        diff_drive_launch,
        TimerAction(period=7.0, actions=[movement_node,]),
    ])
