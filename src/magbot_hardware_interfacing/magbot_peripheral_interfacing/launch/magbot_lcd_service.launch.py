from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='magbot_peripheral_interfacing',
            executable="magbot_lcd_interfacing",
            name="magbot_LCD_node",
            output="screen",
        ),
    ])