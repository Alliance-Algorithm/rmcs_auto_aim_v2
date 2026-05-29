from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    launch = LaunchDescription()

    node = Node(
        package="rmcs_auto_aim_v2",
        executable="rmcs_auto_aim_v2_runtime",
        output="screen",
        respawn=True,
        respawn_delay=1.0,
    )
    launch.add_action(node)

    return launch
