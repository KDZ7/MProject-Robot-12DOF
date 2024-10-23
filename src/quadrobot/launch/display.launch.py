import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, SetEnvironmentVariable, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
  pkg_declare = DeclareLaunchArgument(
    "pkg",
    default_value="quadrobot",
    description="The package containing the URDF file",
  )
  urdf_declare = DeclareLaunchArgument(
    "urdf",
    default_value="",
    description="The URDF file to be loaded",
  )
  rviz_cfg_declare = DeclareLaunchArgument(
    "rviz_cfg",
    default_value="rviz.yaml",
    description="The Rviz configuration file to be loaded",
  )
  use_gz_declare = DeclareLaunchArgument(
    "use_gz",
    default_value="True",
    description="Use Gazebo simulation if True",
  )
  use_sim_time_declare = DeclareLaunchArgument(
    "use_sim_time",
    default_value="True",
    description="Use Gazebo simulation time if True",
  )
  world_declare = DeclareLaunchArgument(
    "world",
    default_value="empty.world.sdf",
    description="World file to load in Gazebo",
  )

  pkg = LaunchConfiguration("pkg")
  urdf = LaunchConfiguration("urdf")
  rviz_cfg = LaunchConfiguration("rviz_cfg")
  use_gz = LaunchConfiguration("use_gz")
  use_sim_time = LaunchConfiguration("use_sim_time")
  world = LaunchConfiguration("world")
  
  pkg_path = get_package_share_directory("quadrobot")
  ros_gz_sim_path = get_package_share_directory("ros_gz_sim")

  urdf_path = PathJoinSubstitution([pkg_path, "urdf", urdf])
  urdf_path = PathJoinSubstitution([pkg_path, "urdf", urdf])
  rviz_cfg_path = PathJoinSubstitution([pkg_path, "config", rviz_cfg])
  urdf_content = ParameterValue(Command(["xacro ", urdf_path]), value_type=str)
  world_path = PathJoinSubstitution([pkg_path, "models", world])

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
    arguments=["-d", rviz_cfg_path],
    condition=UnlessCondition(use_gz),
  )

  envs = {
    "LIBGL_ALWAYS_SOFTWARE": "1",
    "GZ_SIM_RESOURCE_PATH": PathJoinSubstitution([pkg_path, ".."]),
  }

  set_envs = [SetEnvironmentVariable(name=key, value=value) for key, value in envs.items()]

  echo_path = ExecuteProcess(
    cmd=["bash", "-c", "export | grep -E 'LIBGL_ALWAYS_SOFTWARE|GZ_SIM_RESOURCE_PATH' "],
    output="screen",
    condition=IfCondition(use_gz),
  )

  gzserver = IncludeLaunchDescription(
    PythonLaunchDescriptionSource([PathJoinSubstitution([ros_gz_sim_path, "launch", "gz_sim.launch.py"])]),
    launch_arguments={"gz_args": ["-s -v4 ", world_path], "on_exit_shutdown": "True"}.items(),
    condition=IfCondition(use_gz),
  )

  create_node = Node(
    package="ros_gz_sim",
    executable="create",
    arguments=[ "-name", pkg, "-topic", "robot_description", "-z", "0.3"],
    output="screen",
    condition=IfCondition(use_gz),
  )

  gzclient = IncludeLaunchDescription(
    PythonLaunchDescriptionSource([PathJoinSubstitution([ros_gz_sim_path, "launch", "gz_sim.launch.py"])]),
    launch_arguments={"gz_args": ["-g -v4"]}.items(),
    condition=IfCondition(use_gz),
  )

  return LaunchDescription(
    [
      pkg_declare,
      urdf_declare,
      rviz_cfg_declare,
      use_gz_declare,
      use_sim_time_declare,
      world_declare,
      robot_state_publisher_node,
      joint_state_publisher_node,
      joint_state_publisher_gui_node,
      rviz_node,
      *set_envs,
      echo_path,
      gzserver,
      create_node,
      gzclient,
    ]
  )
