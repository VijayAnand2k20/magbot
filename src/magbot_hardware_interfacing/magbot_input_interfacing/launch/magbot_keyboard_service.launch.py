from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='magbot_input_interfacing',
            executable="magbot_keyboard_interfacing",
            name="magbot_keyboard_node",
            output="screen",
        ),
    ])