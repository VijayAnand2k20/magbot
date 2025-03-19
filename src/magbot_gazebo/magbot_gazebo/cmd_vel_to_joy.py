import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy

class CmdVelToJoy(Node):
    def __init__(self):
        super().__init__("cmd_vel_to_joy")
        self.subscription = self.create_subscription(Twist, "/cmd_vel", self.cmd_vel_callback, 10)
        self.publisher = self.create_publisher(Joy, "joy", 10)
        self.get_logger().info("CmdVel to Joy node started!")

    def cmd_vel_callback(self, msg):
        joy_msg = Joy()
        joy_msg.axes = [0.0] * 8  
        joy_msg.buttons = [0] * 12  # Ensure buttons exist

        joy_msg.axes[1] = msg.linear.x  # Forward/Backward
        joy_msg.axes[0] = -msg.angular.z  # Left/Right rotation

        # Simulate R1 button press (index 5) when moving forward
        if msg.linear.x > 0:
            joy_msg.buttons[5] = 1  # Press R1
        else:
            joy_msg.buttons[5] = 0  # Release R1

        self.publisher.publish(joy_msg)


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelToJoy()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
