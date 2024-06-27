import numpy as np
import time
import rclpy
from rclpy.node import Node

import sys
from std_msgs.msg import Float64
import signal
from magbot_peripheral_interfacing.msg import ElectricalMeasurements


#Fetching is_sim and is_physical from arguments
# args = rospy.myargv(argv=sys.argv)
args = sys.argv
if len(args) != 3: #arguments have not been provided, go to defaults (not sim, is physical)
    is_sim = 0
    is_physical = 0
else:
    is_sim = int(args[1])
    is_physical = int(args[2])

from magbot_peripheral_interfacing.IMU import IMU
from magbot_control.Controller import Controller
from magbot_input_interfacing.InputInterface import InputInterface
from magbot_control.State import State
from magbot_control.Kinematics import four_legs_inverse_kinematics
from magbot_control.Config import Configuration

if is_physical:
    from magbot_servo_interfacing.HardwareInterface import HardwareInterface
    from magbot_control.Config import Leg_linkage

def signal_handler(sig, frame):
    sys.exit(0)

class MagBotNode(Node):
    def __init__(self, use_imu=False):
        super().__init__('magbot')
        self.use_imu = use_imu
        self.initialize()
    
    def initialize(self):
        self.get_logger().info("Initializing MagBot Node...")

        if is_sim:
            command_topics = ["/notspot_controller/FR1_joint/command",
                    "/notspot_controller/FR2_joint/command",
                    "/notspot_controller/FR3_joint/command",
                    "/notspot_controller/FL1_joint/command",
                    "/notspot_controller/FL2_joint/command",
                    "/notspot_controller/FL3_joint/command",
                    "/notspot_controller/RR1_joint/command",
                    "/notspot_controller/RR2_joint/command",
                    "/notspot_controller/RR3_joint/command",
                    "/notspot_controller/RL1_joint/command",
                    "/notspot_controller/RL2_joint/command",
                    "/notspot_controller/RL3_joint/command"]

            self.publishers_ = []
            for i in range(len(command_topics)):
                # publishers.append(rospy.Publisher(command_topics[i], Float64, queue_size = 0))
                self.publishers_.append(self.create_publisher(Float64, command_topics[i], 0))
        
        self.config = Configuration()
        if is_physical:
            self.linkage = Leg_linkage(self.config)
            self.hardware_interface = HardwareInterface(self.linkage)
            # Create imu handle
            if self.use_imu:
                # imu = IMU(port="/dev/ttyACM0")
                # imu.flush_buffer()
                self.imu = IMU()
        
        self.controller = Controller(
            self.config,
            four_legs_inverse_kinematics
        )
        self.state = State()
        self.get_logger().info("Creating input listener...")
        self.input_interface = InputInterface(self.config)
        rclpy.spin(self.input_interface)
        self.get_logger().info("Done.")

    def run(self):
        message_rate = 50
        rate = self.create_rate(message_rate)

        signal.signal(signal.SIGINT, signal_handler)

        self.get_logger().info("Summary of gait parameters:")

        last_loop = time.time()

        self.get_logger().info(f"Summary of gait parameters:")
        self.get_logger().info(f"overlap time: {config.overlap_time}")
        self.get_logger().info(f"swing time: {config.swing_time}")
        self.get_logger().info(f"z clearance: {config.z_clearance}")
        self.get_logger().info(f"x shift: {config.x_shift}")

        # Wait until the activate button has been pressed
        loop = 0
        while rclpy.ok():
            self.get_logger().info("Waiting for L1 to activate robot.")
            while True:
                command = self.input_interface.get_command(self.state, message_rate)
                if command.joystick_control_event == 1:
                    break
                rate.sleep()
            self.get_logger().info("Robot activated.")
        
            while True:
                # start_time = self.get_clock().now()
                command = self.input_interface.get_command(self.state, message_rate)
                if command.joystick_control_event == 1:
                    self.get_logger().info("Deactivating Robot")
                    break
                    
                quat_orientation = (
                    self.imu.read_orientation() if self.use_imu else np.array([1, 0, 0, 0])
                )
                self.state.quat_orientation = quat_orientation
                self.controller.run(self.state, command)

                if is_sim:
                    # for i, publisher in enumerate(self.publishers):
                    #     msg = Float64()
                    #     msg.data = self.state.joint_angles.flatten()[i]
                    #     publisher.publish(msg)
                    rows, cols = self.state.joint_angles.shape
                    for row in range(rows):
                        for col in range(cols):
                            index = rows * row + col
                            self.publishers_[index].publish(self.state.joint_angles[index])
                if is_physical:
                    self.hardware_interface.set_actuator_postions(self.state.joint_angles)

                # end_time = self.get_clock().now()
                loop+=1
                rate.sleep()
                
        # #input_interface.set_color(config.ps4_color)

        #     #TODO here: publish the joint values (in state.joint_angles) to a publisher
        #     #If running simulator, publish joint angles to gazebo controller:
        #     if is_sim:
        #         rows, cols = state.joint_angles.shape
        #         print(rows)
        #         print(cols)
        #         for row in rows:
        #             for col in cols:
        #                 publishers[rows*row+col].publish(state.joint_angles[rows*row+col])
            
        #     if is_physical:
        #         # Update the pwm widths going to the servos
                
        #         hardware_interface.set_actuator_postions(state.joint_angles)
            
        #     # print('All angles: \n',np.round(np.degrees(state.joint_angles),2))
        #     # print('All angles: \n',np.round(np.degrees(state.joint_angles[:,0] - state.joint_angles[:,3]),2))
        #     time.end = rospy.Time.now()
        #     #Uncomment following line if want to see how long it takes to execute a control iteration
        #     #print(str(time.start-time.end))
        #     loop +=1
        #     # print('Iteration: ',loop)
        #     # print('State: \n',state)
        #     rate.sleep()


def main(args=None):
    rclpy.init(args=args)
    magbot_node = MagBotNode(use_imu=True)
    magbot_node.run()