from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    turtle_node_1 = Node(
        package="turtlesim", namespace="space1", executable="turtlesim_node"
    )
    broadcaster_turtle_node_1 = Node(
        package="pkg_tf2_py",
        executable="frame_tf2_broadcaster",
        name="frame_tf2_broadcaster_1",
        parameters=[
            {
                "--frame-id": "world",
                "--child-frame-id": "/space1/turtlesim",
                "--topic-position": "/space1/turtlesim/pose",
            }
        ],
    )
    broadcaster_turtle_node_2 = Node(
        package="pkg_tf2_py",
        executable="frame_tf2_broadcaster",
        name="frame_tf2_broadcaster_2",
        parameters=[
            {
                "--frame-id": "world",
                "--child-frame-id": "/space2/turtlesim",
                "--topic-position": "/space2/turtlesim/pose",
            }
        ],
    )
    listener_turtle_node_1 = Node(
        package="pkg_tf2_listener",
        executable="listener",
        name="listener_1",
        parameters=[
            {
                "--source-frame": "space2/turtlesim",
                "--target-frame": "world",
                "--service-comm": "/space1/spawn",
            }
        ],
    )

    return LaunchDescription(
        [
            turtle_node_1,
            broadcaster_turtle_node_1,
            broadcaster_turtle_node_2,
            listener_turtle_node_1,
        ]
    )
