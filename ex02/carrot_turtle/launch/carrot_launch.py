from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterValue

from launch_ros.actions import Node


def generate_launch_description():
    target_frame_arg = DeclareLaunchArgument(
        "target_frame", default_value="carrot1",
        description="Target frame name."
    )
    radius_arg = DeclareLaunchArgument(
        "radius", default_value="5.0",
        description="Radius of carrot circle."
    )
    direction_arg = DeclareLaunchArgument(
        "direction_of_rotation", default_value="1",
        description="Direction of carrot rotation."
    )
    return LaunchDescription([
        target_frame_arg,
        radius_arg,
        direction_arg,
        Node(
            package="turtlesim",
            executable="turtlesim_node",
            name="sim",
        ),
        Node(
            package="carrot_turtle",
            executable="turtle_broadcaster",
            name="broadcaster1",
            parameters=[{"turtlename": "turtle1"},],
        ),
        Node(
            package="carrot_turtle",
            executable="turtle_broadcaster",
            name="broadcaster2",
            parameters=[{"turtlename": "turtle2"},],
        ),
        Node(
            package="carrot_turtle",
            executable="turtle_listener",
            name="listener",
            parameters=[{"target_frame": LaunchConfiguration("target_frame")},],
        ),
        Node(
            package="carrot_turtle",
            executable="carrot_broadcaster",
            name="dynamic_broadcaster",
            parameters=[
                {
                    "radius": ParameterValue(LaunchConfiguration("radius"), value_type=float),
                    "direction_of_rotation": ParameterValue(LaunchConfiguration("direction_of_rotation"), value_type=int),
                },
            ],
        ),
    ])
