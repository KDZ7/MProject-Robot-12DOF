from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    file_declare = DeclareLaunchArgument(
        "file",
        default_value="dgz_description_params.yaml",
        description="YAML file for parameter",
    )

    urdf_declare = DeclareLaunchArgument(
        "urdf",
        default_value="champ.urdf.xacro",
        description="URDF file for robot description",
    )

    disable_file_declare = DeclareLaunchArgument(
        "disable_file", default_value="true", description="Disable YAML parameters"
    )

    rviz_declare = DeclareLaunchArgument(
        "rviz", 
        default_value="rviz.yaml",
        description="RViz file for visualization"
    )

    file_arg = LaunchConfiguration("file")
    urdf_arg = LaunchConfiguration("urdf")
    disable_file_arg = LaunchConfiguration("disable_file")
    rviz_arg = LaunchConfiguration("rviz")
    dgz_viewer_pkg_path = get_package_share_directory("dgz_viewer")

    _1_robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            {
                "robot_description": ParameterValue(
                    Command(
                        [
                            "xacro ",
                            PathJoinSubstitution(
                                [dgz_viewer_pkg_path, "urdf", urdf_arg]
                            ),
                        ]
                    ),
                    value_type=str,
                )
            }
        ],
        condition=IfCondition(disable_file_arg),
    )

    _2_robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            PathJoinSubstitution([dgz_viewer_pkg_path, "config", file_arg])
        ],
        condition=UnlessCondition(disable_file_arg),
    )

    joint_state_publisher_node = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
        output="screen",
    )

    joint_state_publisher_gui_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher_gui",
        output="screen",
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz",
        output="screen",
        arguments=["-d", PathJoinSubstitution([dgz_viewer_pkg_path, "config", rviz_arg])]
    )

    return LaunchDescription([
        file_declare,
        urdf_declare,
        disable_file_declare,
        rviz_declare,
        _1_robot_state_publisher_node,
        _2_robot_state_publisher_node,
        joint_state_publisher_node,
        joint_state_publisher_gui_node,
        rviz_node
    ])