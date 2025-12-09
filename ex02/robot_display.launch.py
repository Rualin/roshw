from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    ld = LaunchDescription()

    default_model_path = PathJoinSubstitution(["urdf", "robot.urdf"])

    ld.add_action(DeclareLaunchArgument(name="model", default_value=default_model_path,
                                        description="Path to robot urdf file relative to urdf_tutorial package"))

    ld.add_action(IncludeLaunchDescription(
        PathJoinSubstitution([FindPackageShare("urdf_launch"), "launch", "display.launch.py"]),
        launch_arguments={
            "urdf_package": "cat_urdf_robot",
            "urdf_package_path": LaunchConfiguration("model"),
        }.items()
    ))

    return ld