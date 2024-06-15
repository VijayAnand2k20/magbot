#!usr/env/bin python3

# Python Imports
import os
import socket
from PIL import Image, ImageDraw, ImageFont
import time


# ROS imports
import rclpy
import rclpy.exceptions
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory

# PI imports
# import LCD_1inch47
import spidev as SPI

from interfaces_pkg.msg import ElectricalMeasurements
from magbot_peripheral_interfacing.scripts import LCD_1inch47


class MagbotDisplayNode(Node):

    def __init__(self):
        super().__init__("magbot_display_node")
        self.rate = self.create_rate(50)

        self.RST = 27
        self.DC = 25
        self.BL = 18
        self.bus = 0
        self.device = 0
        self.ssid = ''
        self.ipAddress = ''
        self.disp = LCD_1inch47.LCD_1inch47()
        # Initialize library.
        self.disp.Init()
        # Clear display.
        self.disp.clear()
        self.battery_voltage_subscriber = self.create_subscription(
            ElectricalMeasurements, "/electrical_measurements", self.update_battery_percentage, 10)

        self.battery_percentage = 0.7

    def update_battery_percentage(self, message):
        max_voltage = 16.8
        min_voltage = 14.0

        battery_voltage_level = max(min(message.battery_voltage_level - min_voltage) /
                                    (max_voltage - min_voltage))

        self.battery_percentage = (
            battery_voltage_level - min_voltage) / (max_voltage - min_voltage)

    def run(self):
        try:
            image1 = Image.new(
                "RGB", (self.disp.height, self.disp.width), "black")
            draw = ImageDraw.Draw(image1)

            # Font1 = ImageFont.truetype(
            #     "/home/pi/Font02.ttf", 25)
            # Font1_small = ImageFont.truetype(
            #     "/home/pi/Font02.ttf", 20)
            # Font1_large = ImageFont.truetype(
            #     "/home/pi/Font02.ttf", 60)
            Font1 = ImageFont.load_default()
            Font1_small = ImageFont.load_default()
            Font1_large = ImageFont.load_default()
            # Font2 = ImageFont.truetype(
            #     "/usr/share/fonts/truetype/Font01.ttf", 35)
            # Font3 = ImageFont.truetype(
            #     "/usr/share/fonts/truetype/Font02.ttf", 120)

            draw.text((20, 110), 'SSID: ' + self.ssid,
                      fill="WHITE", font=Font1)
            draw.text((20, 135), 'IP: ' + self.ipAddress,
                      fill="WHITE", font=Font1)
            current_time = time.strftime("%I:%M:%S%p")  # HR:MIN:SEC AM/PM
            draw.text((220, 0), current_time, fill="WHITE", font=Font1_small)

            black = Image.new("RGB", (320, 172), "black")

            batt_status = Image.open(get_package_share_directory(
                "magbot_peripheral_interfacing") + "/lib/emptybatterystatus_white.png")

            batt_draw = ImageDraw.Draw(batt_status)

            if self.battery_percentage <= 0.20:
                batt_fill = "RED"
            elif 0.20 < self.battery_percentage <= 0.60:
                batt_fill = "d49b00"
            else:
                batt_fill = "#09ab00"

            batt_draw.rounded_rectangle(
                [(42, 92), (42+(153*self.battery_percentage), 170)], 8, fill=batt_fill)
            batt_draw.text((68, 95), str(
                int(self.battery_percentage*100))+"%", fill="WHITE", font=Font1_large)
            batt_scale_factor = 0.8
            resized_batt_status = batt_status.resize(
                (int(batt_status.size[0]*batt_scale_factor), int(batt_status.size[1]*batt_scale_factor)))
            image1.paste(resized_batt_status, (62, -40),
                         resized_batt_status.convert('RGBA'))

            image1 = image1.rotate(0)
            image1 = image1.transpose(Image.ROTATE_270)
            self.disp.ShowImage(image1)

            try:
                self.ssid = os.popen('iwgetid -r').read().strip()
            except rclpy.exceptions.ROSInterruptException as e:
                self.get_logger().error(str(e))
                self.ssid = "N/A"

            # rospy.loginfo("SSID: " + str(self.ssid))
            self.get_logger().info("SSID: " + str(self.ssid))
            # rospy.loginfo("Getting IP address...")
            self.get_logger().info("Getting IP address...")

            try:
                hostname = socket.gethostname()
                self.ipAddress = socket.gethostbyname(hostname)
            except rclpy.exceptions.ROSInterruptException as e:
                self.get_logger().error(str(e))
                self.ipAddress = '-:-:-:-'

            # rospy.loginfo("IP: {self.ipAddress}")
            self.get_logger().info(f"IP: {self.ipAddress}")

        except rclpy.exceptions.ROSInterruptException as e:
            self.get_logger().error(str(e))

    def loop(self):
        while rclpy.ok():
            self.run()
            self.rate.sleep()
        else:
            self.get_logger().info("Quitting...")
            self.disp.clear()
            self.disp.module_exit()

def main(args=None):
    rclpy.init(args=args)
    
    node = MagbotDisplayNode()
    node.get_logger().info("Display node started, outputting to display")
    node.loop()

    rclpy.shutdown()

if __name__ == "__main__":
    main()