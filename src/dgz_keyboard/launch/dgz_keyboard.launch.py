from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():

    turtle_teleop_key_process = ExecuteProcess(
        cmd=[
            "bash", "-c",
            "tmux has-session -t turtle_teleop_key 2>/dev/null || "
            "tmux new-session -d -s turtle_teleop_key 'ros2 run turtlesim turtle_teleop_key --ros-args --remap /turtle1/cmd_vel:=/dgz/cmd_vel'"
        ],
        output="screen"
    )

    dgz_keyboard_node = Node(
        package="dgz_keyboard",
        executable="dgz_keyboard_node",
        name="dgz_keyboard"
    )

    return LaunchDescription([
        turtle_teleop_key_process,
        dgz_keyboard_node
    ])
