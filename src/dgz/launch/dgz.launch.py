from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    file_declare = DeclareLaunchArgument(
        "file",
        default_value="dgz_params.yaml",
        description="Path to the config yaml file",
    )
    file_arg = LaunchConfiguration("file")
    dgz_pkg_path = get_package_share_directory("dgz")
    manette_pkg_path = get_package_share_directory("manette")

    manette = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([manette_pkg_path, "launch", "manette.launch.py"])
        ),
        launch_arguments={
            "file_path": PathJoinSubstitution([dgz_pkg_path, "config", file_arg]),
            "is_file_path": "true",
        }.items(),
    )

    move_ctrl_node = Node(
        package="dgz",
        executable="move_ctrl_node",
        name="move_ctrl_node",
        output="screen",
        parameters=[PathJoinSubstitution([dgz_pkg_path, "config", file_arg])],
    )

    dgz_ctrl_node = Node(
        package="dgz",
        executable="dgz_ctrl_node",
        name="dgz_ctrl_node",
        output="screen",
        parameters=[PathJoinSubstitution([dgz_pkg_path, "config", file_arg])],
    )
    return LaunchDescription(
        [
            file_declare,
            manette,
            move_ctrl_node,
            dgz_ctrl_node,
        ]
    )
