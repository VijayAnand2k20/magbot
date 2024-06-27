from rclpy.node import Node
from std_msgs.msg import String


class StatusPublisher(Node):

    def __init__(self):
        self.__init__("status_publisher")
        # self.status_publisher = rospy.Publisher("/robot_status_messages", String, queue_size = 10)
        self.status_publisher = self.create_publisher(String, "/robot_status_messages", 10)

    def publish_message(self, message):
        self.status_publisher.publish(message)
