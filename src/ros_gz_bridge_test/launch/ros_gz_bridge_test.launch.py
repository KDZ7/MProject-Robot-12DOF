from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    bridge_name_declare = DeclareLaunchArgument(
        "bridge_name",
        default_value="ros_gz_bridge_test",
        description="Name of the bridge",
    )
    file_declare = DeclareLaunchArgument(
        "file",
        default_value="ros_gz_bridge.yaml",
        description="YAML file for parameter",
    )

    ros_gz_bridge_test_pkg_path = get_package_share_directory("ros_gz_bridge_test")
    ros_gz_bridge_pkg_path = get_package_share_directory("ros_gz_bridge")
    
    ros_gz_bridge_test_node = Node(
        package="ros_gz_bridge_test",
        executable="ros_gz_bridge_test_node",
        name="ros_gz_bridge_test_node",
        output="screen",
    )
    ros_gz_bridge_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([ros_gz_bridge_pkg_path, "launch", "ros_gz_bridge.launch.py"])
        ),
        launch_arguments={
            "bridge_name": LaunchConfiguration("bridge_name"),
            "config_file": PathJoinSubstitution([ros_gz_bridge_test_pkg_path, "config", LaunchConfiguration("file")]),
        }.items(),
    )

    return LaunchDescription([
        bridge_name_declare,
        file_declare,
        ros_gz_bridge_test_node,
        ros_gz_bridge_launch,
    ])