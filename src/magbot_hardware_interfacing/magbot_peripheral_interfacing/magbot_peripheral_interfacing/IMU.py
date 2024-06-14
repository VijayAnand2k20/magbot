# Python imports
import numpy as np
import time
import math as m

# PI imports
import board
import adafruit_bno055

# ROS imports
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class IMU(Node):
    def __init__(self):
        super().__init__('IMU')
        self.i2c = board.I2C()  # uses board.SCL and board.SDA
        self.sensor = adafruit_bno055.BNO055_I2C(self.i2c)
        self.last_euler = np.array([ 0, 0, 0])
        self.start_time = time.time()
        
        #! Additional setup for debugging only
        self.pub = self.create_publisher(String, 'imu_data', 10)
        self.create_timer(0.1, self.timer_callback)
    
    def timer_callback(self):
        euler = self.read_orientation()
        self.get_logger().info("Euler: " + str(euler[0]) + " " + str(euler[1]) + " " + str(euler[2]))


    def read_orientation(self):
        """Reads quaternion measurements from the IMU until . Returns the last read Euler angle.
        
        Parameters
        ----------
        None
        
        Returns
        -------
        np array (3,)
            If there was quaternion data to read on the serial port returns the quaternion as a numpy array, otherwise returns the last read quaternion.
        """
        try: 
            [yaw,pitch,roll] = self.sensor.euler
            yaw = m.radians(360-yaw) 
            pitch = m.radians(-pitch)
            roll = m.radians(roll - 30) # fixed offset to account for the IMU being off by 30 degrees
            self.last_euler = [yaw,pitch,roll]
        except:
            self.last_euler = np.array([ 0, 0, 0])
        return self.last_euler


def main(args=None):
    rclpy.init(args=args)
    imu = IMU()
    rclpy.spin(imu)
    rclpy.shutdown()

if __name__ == '__main__':
    main()