# import numpy as np
# import time

# import logging
# logging.basicConfig(level=logging.INFO)

# # import rospy
# import rclpy
# import rclpy.logging
# from rclpy.node import Node
# from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

# import sys
# from std_msgs.msg import Float64
# from std_msgs.msg import Float64MultiArray
# import signal

# #Fetching is_sim and is_physical from arguments
# args = sys.argv
# if len(args) != 4: #arguments have not been provided, go to defaults (not sim, is physical)
#     is_sim = 0
#     is_physical = 1
#     use_imu = 1
# else:
#     is_sim = int(args[1])
#     is_physical = int(args[2])
#     use_imu = int(args[3])

# # is_sim = 1
# # is_physical = 0

# from magbot_control.Controller import Controller
# from magbot_input_interfacing.InputInterface import InputInterface
# from magbot_control.State import State, BehaviorState
# from magbot_control.Kinematics import four_legs_inverse_kinematics
# from magbot_control.Config import Configuration
# from interfaces_pkg.msg import TaskSpace, JointSpace, Angle
# from std_msgs.msg import Bool
# from control_msgs.msg import MultiDOFCommand

# if is_physical:
#     from magbot_servo_interfacing.HardwareInterface import HardwareInterface
#     from magbot_peripheral_interfacing.IMU import IMU
#     from magbot_control.Config import Leg_linkage

# class MagbotDriver(Node):
#     def __init__(self,is_sim, is_physical, use_imu):
#         super().__init__('magbot_driver')
#         self.message_rate = 50
#         logging.info("MagbotDriver node initialized")
#         self.get_logger().info("Initializing node")

#         # self.is_sim = is_sim
#         self.is_sim = True
#         # self.is_physical = is_physical
#         self.is_physical = False
#         self.use_imu = use_imu
#         # self.use_imu = False

#         self.timer_callback_group = MutuallyExclusiveCallbackGroup()

#         # Create a timer that will call the run method at 50 Hz
#         self.timer = self.create_timer(
#             1.0 / self.message_rate,
#             self.run,
#             callback_group=self.timer_callback_group
#         )

#         # self.joint_command_sub = rospy.Subscriber("/joint_space_cmd", JointSpace, self.run_joint_space_command)
#         self.joint_command_sub = self.create_subscription(JointSpace, '/joint_space_cmd', self.run_joint_space_command, 10)
#         # self.task_command_sub = rospy.Subscriber("/task_space_cmd", TaskSpace, self.run_task_space_command)
#         self.task_command_sub = self.create_subscription(TaskSpace, '/task_space_cmd', self.run_task_space_command, 10)
#         # self.estop_status_sub = rospy.Subscriber("/emergency_stop_status", Bool, self.update_emergency_stop_status)
#         self.estop_status_sub = self.create_subscription(Bool, '/emergency_stop_status', self.update_emergency_stop_status, 10)
#         self.external_commands_enabled = 0

#         if self.is_sim:
#             # self.sim_command_topics = ["/dingo_controller/FR_theta1/command",
#             #         "/dingo_controller/FR_theta2/command",
#             #         "/dingo_controller/FR_theta3/command",
#             #         "/dingo_controller/FL_theta1/command",
#             #         "/dingo_controller/FL_theta2/command",
#             #         "/dingo_controller/FL_theta3/command",
#             #         "/dingo_controller/RR_theta1/command",
#             #         "/dingo_controller/RR_theta2/command",
#             #         "/dingo_controller/RR_theta3/command",
#             #         "/dingo_controller/RL_theta1/command",
#             #         "/dingo_controller/RL_theta2/command",
#             #         "/dingo_controller/RL_theta3/command"]

#             # self.sim_publisher_array = []
#             # for i in range(len(self.sim_command_topics)):
#             #     # self.sim_publisher_array.append(rospy.Publisher(self.sim_command_topics[i], Float64, queue_size = 0))
#             #     self.sim_publisher_array.append(self.create_publisher(Float64, self.sim_command_topics[i], 0))
#             self.sim_command_topic = "/dingo_controller/reference"
#             self.sim_publisher = self.create_publisher(MultiDOFCommand, self.sim_command_topic, 10)

#         # Create config
#         self.config = Configuration()
#         if is_physical:
#             self.linkage = Leg_linkage(self.config)
#             self.hardware_interface = HardwareInterface(self.linkage)
#             # rclpy.spin(self.hardware_interface)
#             # Create imu handle
        
#         if self.use_imu:
#             self.imu = IMU()

#         # Create controller and user input handles
#         self.controller = Controller(
#             self.config,
#             four_legs_inverse_kinematics,
#         )

#         self.state = State()
#         self.get_logger().info("Creating input listener...")
#         self.input_interface = InputInterface(self.config)
#         # rclpy.spin(self.input_interface)
#         # rospy.loginfo("Input listener successfully initialised... Robot will now receive commands via Joy messages")
#         self.input_interface.get_logger().info("Input listener successfully initialised... Robot will now receive commands via Joy messages")

#         self.get_logger().info("Summary of current gait parameters:")
#         self.get_logger().info(f"overlap time: {self.config.overlap_time}")
#         self.get_logger().info(f"swing time: {self.config.swing_time}")
#         self.get_logger().info(f"z clearance: {self.config.z_clearance}")
#         self.get_logger().info(f"back leg x shift: {self.config.rear_leg_x_shift}")
#         self.get_logger().info(f"front leg x shift: {self.config.front_leg_x_shift}")

#         self.initial_setup_done = False
#         self.in_manual_control = True

        
    
#     # def run(self):
#     #     # Wait until the activate button has been pressed
#     #     # while not rospy.is_shutdown():

#     #     while rclpy.ok():
#     #         # if self.state.currently_estopped == 1:
#     #         #     self.get_logger().warn("E-stop pressed. Controlling code now disabled until E-stop is released")
#     #         #     self.state.trotting_active = 0
#     #         #     while self.state.currently_estopped == 1:
#     #         #         self.rate.sleep()
#     #         #     self.get_logger().info("E-stop released")
            
#     #         self.get_logger().info("Manual robot control active. Currently not accepting external commands")
#     #         #Always start Manual control with the robot standing still. Send default positions once
#     #         command = self.input_interface.get_command(self.state,self.message_rate)
#     #         self.state.behavior_state = BehaviorState.REST
#     #         self.controller.run(self.state, command)
#     #         self.controller.publish_joint_space_command(self.state.joint_angles)
#     #         self.controller.publish_task_space_command(self.state.rotated_foot_locations)
#     #         if self.is_sim:
#     #                 self.publish_joints_to_sim(self.state.joint_angles)
#     #         if self.is_physical:
#     #             # Update the pwm widths going to the servos
#     #             self.hardware_interface.set_actuator_postions(self.state.joint_angles)
#     #         while self.state.currently_estopped == 0:
#     #             # time.start = rospy.Time.now()
#     #             time.start = self.get_clock().now()

#     #             #Update the robot controller's parameters
#     #             command = self.input_interface.get_command(self.state,self.message_rate)
#     #             if command.joystick_control_event == 1:
#     #                 if self.state.currently_estopped == 0:
#     #                     self.external_commands_enabled = 1
#     #                     break
#     #                 else:
#     #                     # self.get_logger().info("Received Request to enable external control, but e-stop is pressed so the request has been ignored. Please release e-stop and try again")
#     #                     self.get_logger().error("Received Request to enable external control, but e-stop is pressed so the request has been ignored. Please release e-stop and try again")
                
#     #             # Read imu data. Orientation will be None if no data was available
#     #             # rospy.loginfo(imu.read_orientation())
#     #             self.state.euler_orientation = (
#     #                 self.imu.read_orientation() if self.use_imu else np.array([0, 0, 0])
#     #             )
#     #             [yaw,pitch,roll] = self.state.euler_orientation
#     #             # print('Yaw: ',np.round(yaw,2),'Pitch: ',np.round(pitch,2),'Roll: ',np.round(roll,2))
#     #             # Step the controller forward by dt
#     #             self.controller.run(self.state, command)

#     #             if self.state.behavior_state == BehaviorState.TROT or self.state.behavior_state == BehaviorState.REST:
#     #                 self.controller.publish_joint_space_command(self.state.joint_angles)
#     #                 self.controller.publish_task_space_command(self.state.rotated_foot_locations)
#     #                 # rospy.loginfo(state.joint_angles)
#     #                 # rospy.loginfo('State.height: ', state.height)

#     #                 #If running simulator, publish joint angles to gazebo controller:
#     #                 if self.is_sim:
#     #                     self.publish_joints_to_sim(self.state.joint_angles)
#     #                 if self.is_physical:
#     #                     # Update the pwm widths going to the servos
#     #                     self.hardware_interface.set_actuator_postions(self.state.joint_angles)
                    
#     #                 # rospy.loginfo('All angles: \n',np.round(np.degrees(state.joint_angles),2))
#     #                 time.end = self.get_clock().now()
#     #                 #Uncomment following line if want to see how long it takes to execute a control iteration
#     #                 #rospy.loginfo(str(time.start-time.end))

#     #                 # rospy.loginfo('State: \n',state)
#     #             else:
#     #                 if self.is_sim:
#     #                     self.publish_joints_to_sim(self.state.joint_angles)
#     #             self.rate.sleep()

#     #         if self.state.currently_estopped == 0:
#     #             self.get_logger().info("Manual Control deactivated. Now accepting external commands")
#     #             command = self.input_interface.get_command(self.state,self.message_rate)
#     #             self.state.behavior_state = BehaviorState.REST
#     #             self.controller.run(self.state, command)
#     #             self.controller.publish_joint_space_command(self.state.joint_angles)
#     #             self.controller.publish_task_space_command(self.state.rotated_foot_locations)
#     #             if self.is_sim:
#     #                     self.publish_joints_to_sim(self.state.joint_angles)
#     #             if self.is_physical:
#     #                 # Update the pwm widths going to the servos
#     #                 self.hardware_interface.set_actuator_postions(self.state.joint_angles)
#     #             while self.state.currently_estopped == 0:
#     #                 command = self.input_interface.get_command(self.state,self.message_rate)
#     #                 if command.joystick_control_event == 1:
#     #                     self.external_commands_enabled = 0
#     #                     break
#     #                 self.rate.sleep()
    
#     def run(self):
#         if not self.initial_setup_done:
#             self.get_logger().info("Manual robot control active. Currently not accepting external commands")
#             command = self.input_interface.get_command(self.state, self.message_rate)
#             self.state.behavior_state = BehaviorState.REST
#             self.controller.run(self.state, command)
#             self.controller.publish_joint_space_command(self.state.joint_angles)
#             self.controller.publish_task_space_command(self.state.rotated_foot_locations)
#             if self.is_sim:
#                 self.publish_joints_to_sim(self.state.joint_angles)
#             if self.is_physical:
#                 self.hardware_interface.set_actuator_postions(self.state.joint_angles)
#             self.initial_setup_done = True
#             return

#         if self.state.currently_estopped == 1:
#             self.get_logger().warn("E-stop pressed. Controlling code now disabled until E-stop is released")
#             self.state.trotting_active = 0
#             return

#         if self.in_manual_control:
#             command = self.input_interface.get_command(self.state, self.message_rate)
#             if command.joystick_control_event == 1:
#                 if self.state.currently_estopped == 0:
#                     self.external_commands_enabled = 1
#                     self.in_manual_control = False
#                     self.get_logger().info("Manual Control deactivated. Now accepting external commands")
#                     return
#                 else:
#                     self.get_logger().error("Received Request to enable external control, but e-stop is pressed so the request has been ignored. Please release e-stop and try again")

#             self.state.euler_orientation = (
#                 self.imu.read_orientation() if self.use_imu else np.array([0, 0, 0])
#             )
#             self.controller.run(self.state, command)

#             if self.state.behavior_state == BehaviorState.TROT or self.state.behavior_state == BehaviorState.REST:
#                 self.controller.publish_joint_space_command(self.state.joint_angles)
#                 self.controller.publish_task_space_command(self.state.rotated_foot_locations)
#                 if self.is_sim:
#                     self.publish_joints_to_sim(self.state.joint_angles)
#                 if self.is_physical:
#                     self.hardware_interface.set_actuator_postions(self.state.joint_angles)
#             elif self.is_sim:
#                 self.publish_joints_to_sim(self.state.joint_angles)
#         else:
#             command = self.input_interface.get_command(self.state, self.message_rate)
#             if command.joystick_control_event == 1:
#                 self.external_commands_enabled = 0
#                 self.in_manual_control = True
#                 self.get_logger().info("Manual robot control active. Currently not accepting external commands")
#                 return

#             self.state.behavior_state = BehaviorState.REST
#             self.controller.run(self.state, command)
#             self.controller.publish_joint_space_command(self.state.joint_angles)
#             self.controller.publish_task_space_command(self.state.rotated_foot_locations)
#             if self.is_sim:
#                 self.publish_joints_to_sim(self.state.joint_angles)
#             if self.is_physical:
#                 self.hardware_interface.set_actuator_postions(self.state.joint_angles)

#     def update_emergency_stop_status(self, msg):
#         if msg.data == 1:
#             self.state.currently_estopped = 1
#         if msg.data == 0:
#             self.state.currently_estopped = 0
#         return

#     def run_task_space_command(self, msg):
#         if self.external_commands_enabled == 1 and self.currently_estopped == 0:
#             foot_locations = np.zeros((3,4))
#             j = 0
#             for i in range(3):
#                 foot_locations[i] = [msg.FR_foot[j], msg.FL_foot[j], msg.RR_foot[j], msg.RL_foot[j]]
#                 j = j+1
#             print(foot_locations)
#             joint_angles = self.controller.inverse_kinematics(foot_locations, self.config)
#             if self.is_sim:
#                 self.publish_joints_to_sim(self, joint_angles)
            
#             if self.is_physical:
#                 self.hardware_interface.set_actuator_postions(joint_angles)
            
#         elif self.external_commands_enabled == 0:
#             self.get_logger().error("ERROR: Robot not accepting commands. Please deactivate manual control before sending control commands")
#         elif self.currently_estopped == 1:
#             self.get_logger().error("ERROR: Robot currently estopped. Please release before trying to send commands")

#     def run_joint_space_command(self, msg):
#         # if self.external_commands_enabled == 1 and self.currently_estopped == 0:
#         #     joint_angles = np.zeros((3,4))
#         #     j = 0
#         #     for i in 3:
#         #         joint_angles[i] = [msg.FR_foot[j], msg.FL_foot[j], msg.RR_foot[j], msg.RL_foot[j]]
#         #         j = j+1
#         #     print(joint_angles)

#         #     if self.is_sim:
#         #         self.publish_joints_to_sim(self, joint_angles)
            
#         #     if self.is_physical:
#         #         self.hardware_interface.set_actuator_postions(joint_angles)
            
#         # elif self.external_commands_enabled == 0:
#         #     self.get_logger().info("ERROR: Robot not accepting commands. Please deactivate manual control before sending control commands")
#         # elif self.currently_estopped == 1:
#         #     self.get_logger().info("ERROR: Robot currently estopped. Please release before trying to send commands")
#         if self.external_commands_enabled == 1 and self.state.currently_estopped == 0:
#             joint_angles = np.zeros((3,4))
#             j = 0
#             for i in range(3):
#                 joint_angles[i] = [msg.FR_foot[j], msg.FL_foot[j], msg.RR_foot[j], msg.RL_foot[j]]
#                 j = j + 1
#             print(joint_angles)

#             if self.is_sim:
#                 self.publish_joints_to_sim(joint_angles)
#             if self.is_physical:
#                 self.hardware_interface.set_actuator_postions(joint_angles)

#         elif self.external_commands_enabled == 0:
#             self.get_logger().info("ERROR: Robot not accepting commands. Please deactivate manual control before sending control commands")
#         elif self.state.currently_estopped == 1:
#             self.get_logger().info("ERROR: Robot currently estopped. Please release before trying to send commands")
    
#     # def publish_joints_to_sim(self, joint_angles):
#     #     rows, cols = joint_angles.shape
#     #     i = 0
#     #     for col in range(cols):
#     #         for row in range(rows):
#     #             self.sim_publisher_array[i].publish(joint_angles[row,col])
#     #             i = i + 1
#     def publish_joints_to_sim(self, joint_angles):
#         # rows, cols = joint_angles.shape
#         # i = 0
#         # for col in range(cols):
#         #     for row in range(rows):
#         #         msg = Float64()
#         #         msg.data = float(joint_angles[row, col])
#         #         self.sim_publisher_array[i].publish(msg)
#         #         i = i + 1
#         joint_angle_list = [
#             joint_angles[0,0], # FR_theta1
#             joint_angles[1,0], # FR_theta2
#             joint_angles[2,0], # FR_theta3
#             joint_angles[0,1], # FL_theta1
#             joint_angles[1,1], # FL_theta2
#             joint_angles[2,1], # FL_theta3
#             joint_angles[0,2], # RR_theta1
#             joint_angles[1,2], # RR_theta2
#             joint_angles[2,2], # RR_theta3
#             joint_angles[0,3], # RL_theta1
#             joint_angles[1,3], # RL_theta2
#             joint_angles[2,3]  # RL_theta3
#         ]
#         # joint_angle_list = joint_angles.flatten().tolist()

#         # Create Float64MultiArray message
#         # msg = Float64MultiArray()
#         # msg.data = joint_angle_list
#         msg = MultiDOFCommand()
#         msg.dof_names = ['FR_theta1', 'FR_theta2', 'FR_theta3', 'FL_theta1', 'FL_theta2', 'FL_theta3', 'RR_theta1', 'RR_theta2', 'RR_theta3', 'RL_theta1', 'RL_theta2', 'RL_theta3']
#         msg.values = [float(val) for val in joint_angle_list]
#         # msg.values_dot = [0.0] * len(msg.dof_names)  # Velocity references (set to 0)
#         # Publish to the command topic
        
#         self.get_logger().debug(f"Publishing joint angles: {msg.values}")
#         self.sim_publisher.publish(msg)


# def signal_handler(sig, frame):
#     sys.exit(0)

# def main():
#     """Main program
#     """
#     # rospy.init_node("magbot_driver") 
#     rclpy.init(args=args)
#     signal.signal(signal.SIGINT, signal_handler)
#     magbot = MagbotDriver(1, 0, 0)
#     # magbot.run()

#     executor = rclpy.executors.MultiThreadedExecutor()
#     executor.add_node(magbot)
#     executor.add_node(magbot.input_interface)
#     # executor.add_node(magbot.hardware_interface)
#     executor.add_node(magbot.controller)
    
#     try:
#         magbot.run()  # This will start the main logic of the MagbotDriver
#         executor.spin()  # This will handle callbacks for both nodes
#     except KeyboardInterrupt:
#         logging.info("Keybord Interrupt!")
#     finally:
#         executor.shutdown()
#         magbot.destroy_node()
#         magbot.input_interface.destroy_node()
#         magbot.controller.destroy_node()
#         # magbot.hardware_interface.destroy_node()
#         rclpy.shutdown()


#     # rclpy.spin(magbot)
#     # rclpy.spin(magbot.input_interface)
#     # rclpy.shutdown()
    
# # main()

import numpy as np
import logging
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

from std_msgs.msg import Float64, Bool
from control_msgs.msg import MultiDOFCommand
from interfaces_pkg.msg import TaskSpace, JointSpace

from magbot_control.Controller import Controller
from magbot_control.State import State, BehaviorState
from magbot_control.Kinematics import four_legs_inverse_kinematics
from magbot_control.Config import Configuration

class MagbotDriver(Node):
    def __init__(self, is_sim=True, is_physical=False, use_imu=False):
        super().__init__('magbot_driver')
        
        # Initialize logging
        logging.basicConfig(level=logging.INFO)
        self.get_logger().info("Initializing MagbotDriver node")
        
        # Configuration
        self.message_rate = 50
        self.is_sim = is_sim
        self.is_physical = is_physical
        self.use_imu = use_imu
        self.external_commands_enabled = False
        self.initial_setup_done = False
        self.in_manual_control = True

        # Create callback group for timer
        self.timer_callback_group = MutuallyExclusiveCallbackGroup()
        self.timer = self.create_timer(
            1.0 / self.message_rate,
            self.run,
            callback_group=self.timer_callback_group
        )

        # Initialize subscribers
        self._init_subscribers()
        
        # Initialize publishers
        if self.is_sim:
            self._init_sim_publisher()

        # Initialize robot components
        self._init_robot_components()

        # Log configuration
        self._log_config()

    def _init_subscribers(self):
        """Initialize all ROS subscribers"""
        self.joint_command_sub = self.create_subscription(
            JointSpace, '/joint_space_cmd', self.run_joint_space_command, 10)
        self.task_command_sub = self.create_subscription(
            TaskSpace, '/task_space_cmd', self.run_task_space_command, 10)
        self.estop_status_sub = self.create_subscription(
            Bool, '/emergency_stop_status', self.update_emergency_stop_status, 10)

    def _init_sim_publisher(self):
        """Initialize simulation publisher"""
        self.sim_command_topic = "/dingo_controller/reference"
        self.sim_publisher = self.create_publisher(
            MultiDOFCommand, self.sim_command_topic, 10)

    def _init_robot_components(self):
        """Initialize robot configuration, controller, and interfaces"""
        self.config = Configuration()
        
        if self.is_physical:
            from magbot_servo_interfacing.HardwareInterface import HardwareInterface
            from magbot_control.Config import Leg_linkage
            self.linkage = Leg_linkage(self.config)
            self.hardware_interface = HardwareInterface(self.linkage)

        if self.use_imu:
            from magbot_peripheral_interfacing.IMU import IMU
            self.imu = IMU()

        self.controller = Controller(self.config, four_legs_inverse_kinematics)
        self.state = State()
        
        self.get_logger().info("Creating input listener...")
        from magbot_input_interfacing.InputInterface import InputInterface
        self.input_interface = InputInterface(self.config)
        self.input_interface.get_logger().info(
            "Input listener successfully initialised... Robot will now receive commands via Joy messages")

    def _log_config(self):
        """Log current configuration parameters"""
        self.get_logger().info("Summary of current gait parameters:")
        self.get_logger().info(f"overlap time: {self.config.overlap_time}")
        self.get_logger().info(f"swing time: {self.config.swing_time}")
        self.get_logger().info(f"z clearance: {self.config.z_clearance}")
        self.get_logger().info(f"back leg x shift: {self.config.rear_leg_x_shift}")
        self.get_logger().info(f"front leg x shift: {self.config.front_leg_x_shift}")

    def run(self):
        """Main control loop"""
        if not self.initial_setup_done:
            self._perform_initial_setup()
            return

        if self.state.currently_estopped:
            self.get_logger().warn("E-stop pressed. Controlling code now disabled until E-stop is released")
            self.state.trotting_active = 0
            return

        if self.in_manual_control:
            self._handle_manual_control()
        else:
            self._handle_external_control()

    def _perform_initial_setup(self):
        """Perform initial robot setup"""
        self.get_logger().info("Manual robot control active. Currently not accepting external commands")
        command = self.input_interface.get_command(self.state, self.message_rate)
        self.state.behavior_state = BehaviorState.REST
        self._execute_command(command)
        self.initial_setup_done = True

    def _handle_manual_control(self):
        """Handle manual control mode"""
        command = self.input_interface.get_command(self.state, self.message_rate)
        
        if command.joystick_control_event == 1:
            if self.state.currently_estopped == 0:
                self._switch_to_external_control()
                return
            else:
                self.get_logger().error("Cannot enable external control while e-stop is pressed")

        self.state.euler_orientation = (
            self.imu.read_orientation() if self.use_imu else np.array([0, 0, 0])
        )
        self._execute_command(command)

    def _handle_external_control(self):
        """Handle external control mode"""
        command = self.input_interface.get_command(self.state, self.message_rate)
        if command.joystick_control_event == 1:
            self._switch_to_manual_control()
            return

        self.state.behavior_state = BehaviorState.REST
        self._execute_command(command)

    def _execute_command(self, command):
        """Execute a command and update robot state"""
        self.controller.run(self.state, command)
        self.controller.publish_joint_space_command(self.state.joint_angles)
        self.controller.publish_task_space_command(self.state.rotated_foot_locations)
        
        if self.is_sim:
            self.publish_joints_to_sim(self.state.joint_angles)
        if self.is_physical:
            self.hardware_interface.set_actuator_postions(self.state.joint_angles)

    def _switch_to_external_control(self):
        """Switch to external control mode"""
        self.external_commands_enabled = 1
        self.in_manual_control = False
        self.get_logger().info("Manual Control deactivated. Now accepting external commands")

    def _switch_to_manual_control(self):
        """Switch to manual control mode"""
        self.external_commands_enabled = 0
        self.in_manual_control = True
        self.get_logger().info("Manual robot control active. Currently not accepting external commands")

    def update_emergency_stop_status(self, msg):
        """Update emergency stop status"""
        self.state.currently_estopped = msg.data

    def run_task_space_command(self, msg):
        """Handle task space commands"""
        if not self._can_accept_commands():
            return

        foot_locations = np.zeros((3,4))
        for i in range(3):
            foot_locations[i] = [msg.FR_foot[i], msg.FL_foot[i], msg.RR_foot[i], msg.RL_foot[i]]
        
        joint_angles = self.controller.inverse_kinematics(foot_locations, self.config)
        self._send_joint_commands(joint_angles)

    def run_joint_space_command(self, msg):
        """Handle joint space commands"""
        if not self._can_accept_commands():
            return

        joint_angles = np.zeros((3,4))
        for i in range(3):
            joint_angles[i] = [msg.FR_foot[i], msg.FL_foot[i], msg.RR_foot[i], msg.RL_foot[i]]
        
        self._send_joint_commands(joint_angles)

    def _can_accept_commands(self):
        """Check if robot can accept commands"""
        if not self.external_commands_enabled:
            self.get_logger().error("Robot not accepting commands. Please deactivate manual control")
            return False
        if self.state.currently_estopped:
            self.get_logger().error("Robot currently estopped. Please release before sending commands")
            return False
        return True

    def _send_joint_commands(self, joint_angles):
        """Send joint commands to robot"""
        if self.is_sim:
            self.publish_joints_to_sim(joint_angles)
        if self.is_physical:
            self.hardware_interface.set_actuator_postions(joint_angles)

    def publish_joints_to_sim(self, joint_angles):
        """Publish joint angles to simulation"""
        joint_angle_list = [
            joint_angles[0,0], # FR_theta1
            joint_angles[1,0], # FR_theta2
            joint_angles[2,0], # FR_theta3
            joint_angles[0,1], # FL_theta1
            joint_angles[1,1], # FL_theta2
            joint_angles[2,1], # FL_theta3
            joint_angles[0,2], # RR_theta1
            joint_angles[1,2], # RR_theta2
            joint_angles[2,2], # RR_theta3
            joint_angles[0,3], # RL_theta1
            joint_angles[1,3], # RL_theta2
            joint_angles[2,3]  # RL_theta3
        ]

        msg = MultiDOFCommand()
        msg.dof_names = [
            'FR_theta1', 'FR_theta2', 'FR_theta3',
            'FL_theta1', 'FL_theta2', 'FL_theta3',
            'RR_theta1', 'RR_theta2', 'RR_theta3',
            'RL_theta1', 'RL_theta2', 'RL_theta3'
        ]
        msg.values = [float(val) for val in joint_angle_list]
        
        self.get_logger().debug(f"Publishing joint angles: {msg.values}")
        self.sim_publisher.publish(msg)

def main():
    """Main entry point of the program"""
    rclpy.init()
    
    magbot = MagbotDriver(is_sim=True, is_physical=False, use_imu=False)
    
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(magbot)
    executor.add_node(magbot.input_interface)
    executor.add_node(magbot.controller)
    
    try:
        magbot.run()
        executor.spin()
    except KeyboardInterrupt:
        logging.info("Keyboard Interrupt!")
    finally:
        executor.shutdown()
        magbot.destroy_node()
        magbot.input_interface.destroy_node()
        magbot.controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
