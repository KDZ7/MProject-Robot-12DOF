from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions import Node


def generate_launch_description():
    package_declare = DeclareLaunchArgument(
        "package",
        default_value="",
        description="The package containing the URDF file",
    )
    urdf_declare = DeclareLaunchArgument(
        "urdf",
        default_value="",
        description="The URDF file to be loaded",
    )
    rviz_config_declare = DeclareLaunchArgument(
        "rviz_config",
        default_value="urdf.rviz",
        description="The RViz configuration file to be loaded",
    )
    use_gazebo_declare = DeclareLaunchArgument(
        "use_gazebo",
        default_value="false",
        description="Use Gazebo simulation if true",
    )
    use_sim_time_declare = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use simulation (Gazebo) clock if true",
    )
    package = LaunchConfiguration("package")
    urdf = LaunchConfiguration("urdf")
    rviz_config = LaunchConfiguration("rviz_config")
    use_gazebo = LaunchConfiguration("use_gazebo", default="false")
    use_sim_time = LaunchConfiguration("use_sim_time", default="false")
    urdf_path = PathJoinSubstitution([FindPackageShare(package), "urdf", urdf])
    rviz_config_path = PathJoinSubstitution(
        [FindPackageShare(package), "rviz", rviz_config]
    )
    urdf_content = ParameterValue(Command(["xacro ", urdf_path]), value_type=str)

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time, "robot_description": urdf_content}],
        arguments=[urdf_path],
    )

    joint_state_publisher_node = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time, "robot_description": urdf_content}],
        arguments=[urdf_path],
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
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config_path],
    )

    return LaunchDescription(
        [
            package_declare,
            urdf_declare,
            rviz_config_declare,
            use_sim_time_declare,
            robot_state_publisher_node,
            joint_state_publisher_node,
            joint_state_publisher_gui_node,
            rviz_node,
        ]
    )
