from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution


def generate_launch_description():
    opt1 = DeclareLaunchArgument("ns_frame", default_value="ns_turtlesim")
    opt2 = DeclareLaunchArgument("ns_tf_broadcast", default_value="ns_tf2_broadcast")
    opt3 = DeclareLaunchArgument("parent", default_value="world")
    opt4 = DeclareLaunchArgument("child", default_value="child")
    turtlesim_node = Node(
        package="turtlesim",
        namespace=LaunchConfiguration("ns_frame"),
        executable="turtlesim_node",
        name="node_turtle",
    )
    frame_tf2_broadcaster_node = Node(
        package="pkg_tf2_py",
        namespace=LaunchConfiguration("ns_tf_broadcast"),
        executable="frame_tf2_broadcaster",
        name="node_tf2_broadcaster",
        parameters=[
            {
                "--frame-id": LaunchConfiguration("parent"),
                "--child-frame-id": LaunchConfiguration("child"),
                "--topic-position": PathJoinSubstitution(
                    ["/", LaunchConfiguration("ns_frame"), "turtle1", "pose"]
                ),
            },
        ],
    )
    return LaunchDescription(
        [
            opt1,
            opt2,
            opt3,
            opt4,
            turtlesim_node,
            frame_tf2_broadcaster_node,
        ]
    )
