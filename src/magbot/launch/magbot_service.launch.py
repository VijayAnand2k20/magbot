from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('is_sim', default_value='0'),
        DeclareLaunchArgument('is_physical', default_value='1'),
        DeclareLaunchArgument('use_joystick', default_value='1'),
        DeclareLaunchArgument('use_keyboard', default_value='0'),
        DeclareLaunchArgument('serial_port', default_value='/dev/ttyS0'),
        DeclareLaunchArgument('use_imu', default_value='0'),

        GroupAction([
            Node(
                package='rosserial_python',
                executable='serial_node.py',
                name='magbot_rosserial',
                parameters=[{'serial_port': LaunchConfiguration('serial_port')}],
                output='screen',
                condition=IfCondition(LaunchConfiguration('is_physical'))
            ),
            Node(
                package='magbot_peripheral_interfacing',
                executable='magbot_lcd_interfacing',
                name='magbot_LCD_node',
                output='screen',
                condition=IfCondition(LaunchConfiguration('is_physical'))
            )
        ]),

        GroupAction([
            Node(
                package='joy',
                executable='joy_node',
                name='JOYSTICK',
                parameters=[{'autorepeat_rate': 30}],
                output='screen',
                condition=IfCondition(LaunchConfiguration('use_joystick'))
            )
        ]),

        GroupAction([
            Node(
                package='magbot_input_interfacing',
                executable='magbot_keyboard_interfacing',
                name='keyboard_input_listener',
                output='screen',
                condition=IfCondition(LaunchConfiguration('use_keyboard'))
            )
        ]),

        Node(
            package='magbot',
            executable='magbot_driver.py',
            name='magbot',
            arguments=[
                LaunchConfiguration('is_sim'), 
                LaunchConfiguration('is_physical'), 
                LaunchConfiguration('use_imu')
            ],
            output='screen'
        )
    ])
