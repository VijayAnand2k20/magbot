#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, Bool
import RPi.GPIO as GPIO
import sys, signal, subprocess, time

def signal_handler(sig, frame):
    GPIO.cleanup()
    sys.exit(0)

def shutdown():
    GPIO.cleanup()
    print("BATTERY VOLTAGE TOO LOW. COMMENCING SHUTDOWN PROCESS")
    time.sleep(5)
    subprocess.run(["sudo", "shutdown", "-h", "now"])

class BatteryMonitor(Node):
    """
    Class to monitor the battery voltage and publish the battery percentage.

    This class initializes the necessary GPIO pins and creates publishers for battery percentage and emergency stop status.
    It also defines a timer callback function to read the GPIO inputs and calculate the battery percentage.
    """

    def __init__(self):
        super().__init__('battery_monitor')
        self.battery_percentage_publisher = self.create_publisher(Float64, "/battery_percentage", 10)
        self.estop_publisher = self.create_publisher(Bool, "/emergency_stop_status", 10)
        self.current_estop_bit = 0
        self.number_of_low_battery_detections = 0

        signal.signal(signal.SIGINT, signal_handler)

        GPIO.setmode(GPIO.BCM)

        self.estop_pin_number = 5
        self.battery_pin1_number = 6
        self.battery_pin2_number = 13
        self.battery_pin3_number = 19

        GPIO.setup(self.estop_pin_number, GPIO.IN)
        GPIO.setup(self.battery_pin1_number, GPIO.IN)
        GPIO.setup(self.battery_pin2_number, GPIO.IN)
        GPIO.setup(self.battery_pin3_number, GPIO.IN)

        self.timer = self.create_timer(0.02, self.timer_callback)  # 50Hz

    def timer_callback(self):
        """
        Timer callback function to read the GPIO inputs and calculate the battery percentage.

        This function reads the GPIO inputs for emergency stop status and battery voltage.
        It calculates the battery percentage based on the binary representation of the battery voltage.
        The calculated battery percentage is then published.
        If the battery voltage is low for a certain duration, the system is shut down.
        """
        estop_bit = GPIO.input(self.estop_pin_number)
        battery_bit1 = GPIO.input(self.battery_pin1_number)
        battery_bit2 = GPIO.input(self.battery_pin2_number)
        battery_bit3 = GPIO.input(self.battery_pin3_number)

        battery_bits = [battery_bit1, battery_bit2, battery_bit3]

        if estop_bit == 1 and self.current_estop_bit == 0:
            self.current_estop_bit = 1
            self.estop_publisher.publish(Bool(data=True))

        if estop_bit == 0 and self.current_estop_bit == 1:
            self.current_estop_bit = 0
            self.estop_publisher.publish(Bool(data=False))

        num = int("".join([str(b) for b in battery_bits]), 2)

        value = 0.0

        if num == 0:
            value = 0.0
        elif num == 1:
            value = 0.125
        elif num == 2:
            value = 0.25
        elif num == 3:
            value = 0.375
        elif num == 4:
            value = 0.5
        elif num == 5:
            value = 0.625
        elif num == 6:
            value = 0.75
        elif num == 7:
            value = 1

        self.battery_percentage_publisher.publish(Float64(data=value))

        if value == 0.0:
            self.number_of_low_battery_detections += 1
            if self.number_of_low_battery_detections > 30:
                shutdown()
        else:
            if self.number_of_low_battery_detections > 0:
                self.number_of_low_battery_detections -= 1

def main(args=None):
    rclpy.init(args=args)

    battery_monitor = BatteryMonitor()

    rclpy.spin(battery_monitor)

    battery_monitor.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()