from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    file_declare = DeclareLaunchArgument(
        "file",
        default_value="manette_params.yaml",
        description="File name of the config yaml file",
    )
    file_path_declare= DeclareLaunchArgument(
        "file_path",
        default_value="",
        description="File name with path of the config yaml file",
    )
    is_file_path_declare = DeclareLaunchArgument(
        "is_file_path",
        default_value="false",
        description="If is file with path",
    )
    file_arg = LaunchConfiguration("file")
    file_path_arg = LaunchConfiguration("file_path")
    is_file_path_arg = LaunchConfiguration("is_file_path")
    manette_pkg_path = get_package_share_directory("manette")

    _1_joy_linux_node = Node(
        package="joy_linux",
        executable="joy_linux_node",
        name="joy_linux_node",
        output="screen",
        parameters=[file_path_arg],
        condition=IfCondition(is_file_path_arg)
    )
    _2_joy_linux_node = Node(
        package="joy_linux",
        executable="joy_linux_node",
        name="joy_linux_node",
        output="screen",
        parameters=[PathJoinSubstitution([manette_pkg_path, "config", file_arg])],
        condition=UnlessCondition(is_file_path_arg)
    )
    manette_node = Node(
        package="manette",
        executable="manette_node",
        name="manette_node",
        output="screen"
    )
    return LaunchDescription([
        file_declare,
        file_path_declare,
        is_file_path_declare,
        _1_joy_linux_node,
        _2_joy_linux_node,
        manette_node
    ])