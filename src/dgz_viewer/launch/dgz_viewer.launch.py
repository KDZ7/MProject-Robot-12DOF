import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    file_declare = DeclareLaunchArgument(
        "file",
        default_value="dgz_viewer_params.yaml",
        description="YAML file for parameter",
    )

    urdf_declare = DeclareLaunchArgument(
        "urdf",
        default_value="champ.urdf.xacro",
        description="URDF file for robot description",
    )

    disable_file_declare = DeclareLaunchArgument(
        "disable_file", default_value="True", description="Disable YAML parameters"
    )

    rviz_declare = DeclareLaunchArgument(
        "rviz", 
        default_value="rviz.yaml",
        description="RViz file for visualization"
    )

    use_gz_declare = DeclareLaunchArgument(
        "use_gz", 
        default_value="True",
        description="Use Gazebo simulation"
    )

    use_sim_time_declare = DeclareLaunchArgument(
        "use_sim_time",
        default_value="True",
        description="Use Gazebo simulation time"
    )

    file_arg = LaunchConfiguration("file")
    urdf_arg = LaunchConfiguration("urdf")
    disable_file_arg = LaunchConfiguration("disable_file")
    rviz_arg = LaunchConfiguration("rviz")
    use_gz_arg = LaunchConfiguration("use_gz")
    use_sim_time_arg = LaunchConfiguration("use_sim_time")
    dgz_viewer_pkg_path = get_package_share_directory("dgz_viewer")
    ros_gz_sim_pkg_path = get_package_share_directory("ros_gz_sim")

    _1_robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            {
                "robot_description": ParameterValue(Command(["xacro ", PathJoinSubstitution([dgz_viewer_pkg_path, "urdf", urdf_arg])]),value_type=str),
                "use_sim_time": use_sim_time_arg
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
        parameters=[{"use_sim_time": use_sim_time_arg}],
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
    os.environ["LIBGL_ALWAYS_SOFTWARE"] = "1"
    os.environ["GZ_SIM_RESOURCE_PATH"] = f"{dgz_viewer_pkg_path}/models/worlds:{dgz_viewer_pkg_path}/.."
    
    exec_printenv = ExecuteProcess(
        cmd=["bash", "-c", "printenv | grep -E 'LIBGL_ALWAYS_SOFTWARE|GZ_SIM_RESOURCE_PATH'"],
        output="screen"
    )

    gz_server_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([PathJoinSubstitution([ros_gz_sim_pkg_path, "launch", "gz_sim.launch.py"])]),
        launch_arguments={
            "gz_args": [" -s -v4 ", PathJoinSubstitution([dgz_viewer_pkg_path, "models", "worlds", "empty.world.sdf"])],
            "on_exit_shutdown": "True"
        }.items(),
        condition=IfCondition(use_gz_arg)
    )
    create_node = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[ "-name", "dogzilla", "-topic", "robot_description", "-z", "0.3"],
        output="screen",
        condition=IfCondition(use_gz_arg)
    )
    gz_client_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([PathJoinSubstitution([ros_gz_sim_pkg_path, "launch", "gz_sim.launch.py"])]),
        launch_arguments={
            "gz_args": [" -g -v4 "],
            "on_exit_shutdown": "True"
        }.items(),
        condition=IfCondition(use_gz_arg)
    )

    return LaunchDescription([
        file_declare,
        urdf_declare,
        disable_file_declare,
        rviz_declare,
        use_gz_declare,
        use_sim_time_declare,
        _1_robot_state_publisher_node,
        _2_robot_state_publisher_node,
        joint_state_publisher_node,
        joint_state_publisher_gui_node,
        # rviz_node,
        exec_printenv,
        gz_server_launch,
        create_node,
        gz_client_launch,
    ])