import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, RegisterEventHandler, ExecuteProcess, TimerAction
from launch.event_handlers import OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command, FindExecutable
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    urdf_file = os.path.join(get_package_share_directory(
        'magbot_gazebo'), 'description', 'urdf', 'dingo.urdf.xacro')

    robot_description = ParameterValue(Command([
        FindExecutable(name='xacro'), ' ', urdf_file
    ]), value_type=str)
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={'verbose': 'true'}.items()
    )

    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description',
                   '-entity', 'magbot', '-z', '0.2'],
        output='screen'
    )

    # joint_state_broadcaster_spawner = Node(
    #     package='controller_manager',
    #     executable='spawner',
    #     parameters=[
    #             {"use_sim_time": True}
    #     ],
    #     arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
    #     output='screen'
    # )

    dingo_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['dingo_controller', '--controller-manager', '/controller_manager'],
        output='screen'
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'publish_frequency': 10.0,
                     'robot_description': robot_description,
                     'use_sim_time': use_sim_time}],
        remappings=[('/joint_states', '/dingo_gazebo/joint_states')]
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true', description='Use sim time'),
        robot_state_publisher,
        gazebo,
        spawn_robot,
        # joint_state_broadcaster_spawner,
        dingo_controller_spawner,
    ])