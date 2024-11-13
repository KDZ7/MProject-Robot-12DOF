from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    joint_state_publisher_node = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
        output="screen",
    )

    return LaunchDescription([joint_state_publisher_node])
