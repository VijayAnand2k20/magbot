# import os
# from launch import LaunchDescription
# from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, RegisterEventHandler
# from launch.event_handlers import OnProcessExit
# from launch.launch_description_sources import PythonLaunchDescriptionSource
# from launch.substitutions import LaunchConfiguration, Command, FindExecutable
# from launch_ros.actions import Node
# from launch_ros.parameter_descriptions import ParameterValue
# from ament_index_python.packages import get_package_share_directory

# def generate_launch_description():
#     # Declare arguments
#     use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
#     # Robot description
#     robot_description = ParameterValue(Command([
#         FindExecutable(name='xacro'), ' ',
#         os.path.join(get_package_share_directory('magbot_gazebo'), 'description','urdf', 'dingo.urdf.xacro')
#     ]), value_type=str)

#     # Load joint state publisher
#     joint_state_publisher = Node(
#         package='joint_state_publisher',
#         executable='joint_state_publisher',
#         name='joint_state_publisher',
#         output='screen',
#         parameters=[{'use_sim_time': use_sim_time}]
#     )

#     # Load controllers
#     load_joint_state_controller = Node(
#         package='controller_manager',
#         executable='spawner',
#         arguments=['joint_state_broadcaster'],
#         output='screen',
#     )

#     load_robot_controller = Node(
#         package='controller_manager',
#         executable='spawner',
#         arguments=['magbot_controller'],
#         output='screen',
#     )

#     # Robot State Publisher
#     robot_state_publisher = Node(
#         package='robot_state_publisher',
#         executable='robot_state_publisher',
#         name='robot_state_publisher',
#         output='screen',
#         parameters=[{'robot_description': robot_description,
#                      'publish_frequency': 30.0,
#                      'use_sim_time': use_sim_time}],
#     )
    
#     return LaunchDescription([
#         DeclareLaunchArgument('use_sim_time', default_value='true', description='Use sim time'),
#         joint_state_publisher,
#         robot_state_publisher,
#         RegisterEventHandler(
#             event_handler=OnProcessExit(
#                 target_action=load_joint_state_controller,
#                 on_exit=[load_robot_controller],
#             )
#         ),
#     ])

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command, FindExecutable
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Declare arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    # Paths to required files
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    urdf_file = os.path.join(get_package_share_directory('magbot_gazebo'), 'description', 'urdf', 'dingo.urdf.xacro')
    world_file = os.path.join(get_package_share_directory('magbot_gazebo'), 'description', 'worlds', 'normal.world')
    
    # Robot description from xacro
    robot_description = ParameterValue(Command([
        FindExecutable(name='xacro'), ' ', urdf_file
    ]), value_type=str)

    # Gazebo launch file
    # gazebo = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')
    #     ),
    #     launch_arguments={'world': world_file, 'use_sim_time': use_sim_time}.items()
    # )
    
    # Node to spawn the robot in Gazebo
    # spawn_robot = Node(
    #     package='gazebo_ros',
    #     executable='spawn_entity.py',
    #     arguments=['-topic', 'robot_description', '-entity', 'magbot', '-z', '0.1'],
    #     output='screen'
    # )

    # Load joint state publisher
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # Load controllers
    load_joint_state_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen',
    )

    load_robot_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['magbot_controller'],
        output='screen',
    )

    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description,
                     'publish_frequency': 30.0,
                     'use_sim_time': use_sim_time}],
    )
    
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true', description='Use sim time'),
        
        # Start Gazebo
        # gazebo,

        # Start joint state publisher and robot state publisher
        joint_state_publisher,
        robot_state_publisher,

        # Spawn the robot in Gazebo
        # spawn_robot,

        # # Load joint state broadcaster and robot controller after spawning the robot
        # RegisterEventHandler(
        #     event_handler=OnProcessExit(
        #         target_action=spawn_robot,
        #         on_exit=[load_joint_state_controller]
        #     )
        # ),

        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_joint_state_controller,
                on_exit=[load_robot_controller]
            )
        ),
    ])
