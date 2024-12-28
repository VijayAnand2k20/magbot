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

# import os
# from launch import LaunchDescription
# from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, RegisterEventHandler
# from launch.event_handlers import OnProcessExit
# from launch.launch_description_sources import PythonLaunchDescriptionSource
# from launch.substitutions import LaunchConfiguration, Command, FindExecutable
# from launch_ros.actions import Node
# from launch.substitutions import FindExecutable, PathJoinSubstitution
# from launch_ros.substitutions import FindPackageShare
# from launch_ros.parameter_descriptions import ParameterValue
# from ament_index_python.packages import get_package_share_directory

# def generate_launch_description():
#     # Declare arguments
#     use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
#     # Paths to required files
#     pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
#     urdf_file = os.path.join(get_package_share_directory('magbot_gazebo'), 'description', 'urdf', 'dingo.urdf.xacro')
#     world_file = os.path.join(get_package_share_directory('magbot_gazebo'), 'description', 'worlds', 'normal.world')
    
#     # Robot description from xacro
#     robot_description = ParameterValue(Command([
#         FindExecutable(name='xacro'), ' ', urdf_file
#     ]), value_type=str)

#     # Gazebo launch file
#     gazebo = IncludeLaunchDescription(
#         PythonLaunchDescriptionSource(
#             os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')
#         ),
#         launch_arguments={'world': world_file, 'use_sim_time': use_sim_time}.items()
#     )
    
#     # Node to spawn the robot in Gazebo
#     spawn_robot = Node(
#         package='gazebo_ros',
#         executable='spawn_entity.py',
#         arguments=['-topic', 'robot_description', '-entity', 'magbot', '-z', '0.1'],
#         output='screen'
#     )

#     controller_manager = Node(
#         package='controller_manager',
#         executable='ros2_control_node',
#         parameters=[PathJoinSubstitution([
#             FindPackageShare('magbot_gazebo'), 'config', 'dingo_controllers.yaml'
#         ])],
#         output='screen',
#         namespace='dingo_controller'
#     )

#     # Load joint state publisher
#     # joint_state_publisher = Node(
#     #     package='joint_state_publisher',
#     #     executable='joint_state_publisher',
#     #     name='joint_state_publisher',
#     #     output='screen',
#     #     parameters=[{'use_sim_time': use_sim_time}]
#     # )

#     # # Load controllers
#     # load_joint_state_controller = Node(
#     #     package='controller_manager',
#     #     executable='spawner',
#     #     arguments=['joint_state_broadcaster'],
#     #     output='screen',
#     # )

#     load_robot_controller = Node(
#             package='controller_manager',
#             executable='spawner',
#             name='controller_spawner',
#             output='screen',
#             namespace='dingo_controller',
#             parameters=[PathJoinSubstitution([
#                 FindPackageShare('magbot_gazebo'), 'config', 'dingo_controllers.yaml'
#             ])],
#             arguments=[
#                 'FR_theta1', 'FR_theta2', 'FR_theta3',
#                 'FL_theta1', 'FL_theta2', 'FL_theta3',
#                 'RR_theta1', 'RR_theta2', 'RR_theta3',
#                 'RL_theta1', 'RL_theta2', 'RL_theta3'
#             ]
#         )
#     # Node(
#     #     package='controller_manager',
#     #     executable='spawner',
#     #     arguments=['magbot_controller'],
#     #     output='screen',
#     # )

#     # Robot State Publisher
#     robot_state_publisher = Node(
#             package='robot_state_publisher',
#             executable='robot_state_publisher',
#             name='robot_state_publisher',
#             output='screen',
#             parameters=[{'publish_frequency': 10.0,
#                          'robot_description': robot_description,
#                          'use_sim_time': use_sim_time}],
#             remappings=[('/joint_states', '/dingo_gazebo/joint_states')]
#         )
#     # Node(
#     #     package='robot_state_publisher',
#     #     executable='robot_state_publisher',
#     #     name='robot_state_publisher',
#     #     output='screen',
#     #     parameters=[{'robot_description': robot_description,
#     #                  'publish_frequency': 30.0,
#     #                  'use_sim_time': use_sim_time}],
#     # )

    

#     # Robot State Publisher

    
#     # return LaunchDescription([
#     #     DeclareLaunchArgument('use_sim_time', default_value='true', description='Use sim time'),
        
#     #     # Start Gazebo
#     #     gazebo,

#     #     # Start joint state publisher and robot state publisher
#     #     # joint_state_publisher,
#     #     controller_manager,
#     #     robot_state_publisher,
#     #     # Spawn the robot in Gazebo
#     #     spawn_robot,
#     #     load_robot_controller,


#     #     # # Load joint state broadcaster and robot controller after spawning the robot
#     #     # RegisterEventHandler(
#     #     #     event_handler=OnProcessExit(
#     #     #         target_action=spawn_robot,
#     #     #         on_exit=[load_joint_state_controller]
#     #     #     )
#     #     # ),

#     #     # RegisterEventHandler(
#     #     #     event_handler=OnProcessExit(
#     #     #         target_action=load_joint_state_controller,
#     #     #         on_exit=[load_robot_controller]
#     #     #     )
#     #     # ),
#     # ])
#     return LaunchDescription([
#         DeclareLaunchArgument('use_sim_time', default_value='true', description='Use sim time'),
#         gazebo,
#         robot_state_publisher,
#         controller_manager,
#         RegisterEventHandler(
#             event_handler=OnProcessExit(
#                 target_action=gazebo,
#                 on_exit=[spawn_robot]
#             )
#         ),
#     ]) 
# import os
# from launch import LaunchDescription
# from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, RegisterEventHandler, ExecuteProcess, TimerAction
# from launch.event_handlers import OnProcessStart
# from launch.launch_description_sources import PythonLaunchDescriptionSource
# from launch.substitutions import LaunchConfiguration, Command, FindExecutable
# from launch_ros.actions import Node
# from launch.substitutions import PathJoinSubstitution
# from launch_ros.substitutions import FindPackageShare
# from launch_ros.parameter_descriptions import ParameterValue
# from ament_index_python.packages import get_package_share_directory

# def generate_launch_description():
#     # Declare arguments
#     use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
#     # Paths to required files
#     pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
#     urdf_file = os.path.join(get_package_share_directory('magbot_gazebo'), 'description', 'urdf', 'dingo.urdf.xacro')
#     # world_file = os.path.join(get_package_share_directory('magbot_gazebo'), 'description', 'worlds', 'normal.world')
    
#     # Robot description from xacro
#     robot_description = ParameterValue(Command([
#         FindExecutable(name='xacro'), ' ', urdf_file
#     ]), value_type=str)

#     # Gazebo launch file
#     gazebo = IncludeLaunchDescription(
#         PythonLaunchDescriptionSource(
#             os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')
#         ),
#         # launch_arguments={'world': world_file, 'use_sim_time': use_sim_time}.items()
#     )
    
#     # Node to spawn the robot in Gazebo
#     spawn_robot = Node(
#         package='gazebo_ros',
#         executable='spawn_entity.py',
#         arguments=['-topic', 'robot_description', '-entity', 'magbot', '-z', '3'
#         ],
#         output='screen'
#     )
#     # spawn_robot = ExecuteProcess(
#     #     cmd=[
#     #         'ros2', 'run', 'gazebo_ros', 'spawn_entity.py',
#     #         '-entity', 'magbot',
#     #         '-file', urdf_file,
#     #         '-z', '3',
#     #         '-J', 'FR_theta2', '0.72646626',
#     #         '-J', 'FL_theta2', '0.72646626',
#     #         '-J', 'RR_theta2', '0.72646626',
#     #         '-J', 'RL_theta2', '0.72646626',
#     #         '-J', 'FR_theta1', '0.0',
#     #         '-J', 'FL_theta1', '0.0',
#     #         '-J', 'RR_theta1', '0.0',
#     #         '-J', 'RL_theta1', '0.0',
#     #         '-J', 'FR_theta3', '0.0',
#     #         '-J', 'FL_theta3', '0.0',
#     #         '-J', 'RR_theta3', '0.0',
#     #         '-J', 'RL_theta3', '0.0'
#     #     ],
#     #     output='screen'
#     # )

#     controller_manager = Node(
#         package='controller_manager',
#         executable='ros2_control_node',
#         namespace='dingo_controller',
#         parameters=[PathJoinSubstitution([
#             FindPackageShare('magbot_gazebo'), 'config', 'dingo_controllers.yaml'
#         ])],
#         output='screen',
#     )

#     joint_state_broadcaster_spawner = Node(
#         package='controller_manager',
#         executable='spawner',
#         namespace='dingo_controller',
#         # arguments=['joint_state_broadcaster'],
#         arguments=['joint_state_broadcaster', '--controller-manager', '/dingo_controller/controller_manager'],
#         output='screen'
#     )

#     # Spawner node to activate the dingo_controller
#     dingo_controller_spawner = Node(
#         package='controller_manager',
#         executable='spawner',
#         namespace='dingo_controller',
#         arguments=['dingo_controller', '--controller-manager', '/dingo_controller/controller_manager'],
#         output='screen'
#     )

#     # Robot State Publisher
#     robot_state_publisher = Node(
#         package='robot_state_publisher',
#         executable='robot_state_publisher',
#         name='robot_state_publisher',
#         # namespace='dingo_controller',
#         output='screen',
#         parameters=[{'publish_frequency': 10.0,
#                      'robot_description': robot_description,
#                      'use_sim_time': use_sim_time}],
#         remappings=[('/joint_states', '/dingo_gazebo/joint_states')]
#     )

#     return LaunchDescription([
#         DeclareLaunchArgument('use_sim_time', default_value='true', description='Use sim time'),
#         robot_state_publisher,
#         gazebo,
#         controller_manager,
#         RegisterEventHandler(
#             event_handler=OnProcessStart(
#                 target_action=controller_manager,
#                 on_start=[
#                     # Add a delay before spawning controllers
#                     TimerAction(
#                         period=5.0,
#                         actions=[
#                             joint_state_broadcaster_spawner,
#                             dingo_controller_spawner
#                         ]
#                     )
#                 ]
#             )
#         ),
#         # joint_state_broadcaster_spawner,
#         # dingo_controller_spawner,
#         # load_robot_controller,
#         spawn_robot,
#     ])
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
    # Declare arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    # Paths to required files
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    urdf_file = os.path.join(get_package_share_directory('magbot_gazebo'), 'description', 'urdf', 'dingo.urdf.xacro')
    # world_file = os.path.join(get_package_share_directory('magbot_gazebo'), 'description', 'worlds', 'normal.world')
    
    # Robot description from xacro
    robot_description = ParameterValue(Command([
        FindExecutable(name='xacro'), ' ', urdf_file
    ]), value_type=str)

    # Gazebo launch file
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')
        ),
        # launch_arguments={'world': world_file, 'use_sim_time': use_sim_time}.items()
    )
    # gazebo = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')
    #     ),
    #     launch_arguments={'verbose': 'true'}.items()
    # )
    
    # Node to spawn the robot in Gazebo
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        # namespace='dingo_controller',
        arguments=['-topic', 'robot_description', '-entity', 'magbot', '-z', '3'],
        output='screen'
    )
    # spawn_robot = ExecuteProcess(
    #     cmd=[
    #         'ros2', 'run', 'gazebo_ros', 'spawn_entity.py',
    #         '-entity', 'magbot',
    #         '-file', urdf_file,
    #         '-z', '3',
    #         '-J', 'FR_theta2', '0.72646626',
    #         '-J', 'FL_theta2', '0.72646626',
    #         '-J', 'RR_theta2', '0.72646626',
    #         '-J', 'RL_theta2', '0.72646626',
    #         '-J', 'FR_theta1', '0.0',
    #         '-J', 'FL_theta1', '0.0',
    #         '-J', 'RR_theta1', '0.0',
    #         '-J', 'RL_theta1', '0.0',
    #         '-J', 'FR_theta3', '0.0',
    #         '-J', 'FL_theta3', '0.0',
    #         '-J', 'RR_theta3', '0.0',
    #         '-J', 'RL_theta3', '0.0'
    #     ],
    #     output='screen'
    # )

    # controller_manager = Node(
    #     package='controller_manager',
    #     executable='ros2_control_node',
    #     namespace='dingo_controller',
    #     parameters=[PathJoinSubstitution([
    #         FindPackageShare('magbot_gazebo'), 'config', 'dingo_controllers.yaml'
    #     ])],
    #     output='screen',
    # )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        # namespace='dingo_controller',
        # arguments=['joint_state_broadcaster'],
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen'
    )

    # Spawner node to activate the dingo_controller
    dingo_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        # namespace='dingo_controller',
        arguments=['dingo_controller', '--controller-manager', '/controller_manager'],
        output='screen'
    )

    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        # namespace='dingo_controller',
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

        # controller_manager,
        # RegisterEventHandler(
        #     event_handler=OnProcessStart(
        #         target_action=controller_manager,
        #         on_start=[
        #             # Add a delay before spawning controllers
        #             TimerAction(
        #                 period=5.0,
        #                 actions=[
        #                     joint_state_broadcaster_spawner,
        #                     dingo_controller_spawner
        #                 ]
        #             )
        #         ]
        #     )
        # ),


        joint_state_broadcaster_spawner,
        dingo_controller_spawner,
        # load_robot_controller,
    ])