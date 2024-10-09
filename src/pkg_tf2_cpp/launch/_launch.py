from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    turtlesim_node = Node(
        package="turtlesim",
        executable="turtlesim_node",
        name="turtlesim_1",
    )
    frame_tf2_broadcaster_node = Node(
        package="pkg_tf2_cpp",
        executable="frame_tf2_broadcaster",
        name="frame_tf2_broadcaster_1",
        parameters=[
            {
                "--frame-id": "world",
                "--child-frame-id": "turtle_frame_1",
                "--topic-position": "/turtle1/pose",
            },
        ],
    )
    return LaunchDescription([turtlesim_node, frame_tf2_broadcaster_node])
