import os
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    quadrobot_viewer_pkg_path = get_package_share_directory("quadrobot_viewer")
    ros_gz_sim_pkg_path = get_package_share_directory("ros_gz_sim")
    config_path = os.path.join(quadrobot_viewer_pkg_path, "config", "config.yaml")
    config_ros2_control_path = os.path.join(quadrobot_viewer_pkg_path, "config", "ros2_control.yaml")

    with open(config_path, "r") as yaml_file:
        config_data = yaml.safe_load(yaml_file)

    os.environ["GZ_SIM_RESOURCE_PATH"] = f"{quadrobot_viewer_pkg_path}/models/worlds:{quadrobot_viewer_pkg_path}/.."
    for key, value in config_data["environment"].items():
        os.environ[key] = value

    exec_printenv = ExecuteProcess(
        cmd=["bash", "-c", "printenv | grep -E 'ROS_DOMAIN_ID|LIBGL_ALWAYS_SOFTWARE|GZ_SIM_RESOURCE_PATH'"],
        output="screen"
    )
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            {
                "robot_description": ParameterValue(Command(["xacro ", PathJoinSubstitution([quadrobot_viewer_pkg_path, "urdf", config_data["robot_state_publisher"]["urdf"]])])),
                "use_sim_time": config_data["robot_state_publisher"]["use_sim_time"],
            }
        ]
    )

    joint_state_publisher_node = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
        output="screen",
        parameters=[config_data["joint_state_publisher"]]
    )

    joint_state_publisher_gui_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher_gui",
        parameters=[config_data["joint_state_publisher_gui"]],
        output="screen",
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz",
        output="screen",
        arguments=["-d", PathJoinSubstitution([quadrobot_viewer_pkg_path, "config", config_data["rviz2"]["config"]])]
    )

    gz_server_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([PathJoinSubstitution([ros_gz_sim_pkg_path, "launch", "gz_sim.launch.py"])]),
        launch_arguments={
            "gz_args": [" -s -r -v4 ", PathJoinSubstitution(["empty.world.sdf"])],
            "on_exit_shutdown": "true"
        }.items()
    )

    create_node = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-name", config_data["spawn_entity"]["name"],
            "-topic", config_data["spawn_entity"]["topic"],
            "-x", config_data["spawn_entity"]["x"],
            "-y", config_data["spawn_entity"]["y"],
            "-z", config_data["spawn_entity"]["z"],
        ],
        output="screen"
    )

    gz_client_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([PathJoinSubstitution([ros_gz_sim_pkg_path, "launch", "gz_sim.launch.py"])]),
        launch_arguments={
            "gz_args": ["-g -v4"],
            "on_exit_shutdown": "true"
        }.items(),
    )

    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[config_ros2_control_path],
        output="screen"
    )

    ros2_control_spawn_node_1 = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
        output="screen"
    )
    ros2_control_spawn_node_2 = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_group_position_controller"],
        output="screen"
    )

    return LaunchDescription([
        robot_state_publisher_node,
        # joint_state_publisher_node,
        # joint_state_publisher_gui_node,
        rviz_node,
        exec_printenv,
        gz_server_launch,
        create_node,
        gz_client_launch,
        ros2_control_node,
        ros2_control_spawn_node_1,
        ros2_control_spawn_node_2
    ])