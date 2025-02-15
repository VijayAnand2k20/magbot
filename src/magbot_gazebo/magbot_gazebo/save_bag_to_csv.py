#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import csv
import time

class CSVExporter(Node):
    def __init__(self):
        super().__init__('csv_exporter')
        self.odom_file = open('odom_export.csv', 'w', newline='')
        self.gt_file = open('ground_truth_export.csv', 'w', newline='')
        self.odom_writer = csv.writer(self.odom_file)
        self.gt_writer = csv.writer(self.gt_file)
        # Write CSV headers
        self.odom_writer.writerow(["timestamp", "x", "y", "z", "qx", "qy", "qz", "qw"])
        self.gt_writer.writerow(["timestamp", "x", "y", "z", "qx", "qy", "qz", "qw"])
        
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_subscription(Odometry, '/ground_truth', self.gt_callback, 10)

    def odom_callback(self, msg: Odometry):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        pos = msg.pose.pose.position
        ori = msg.pose.pose.orientation
        self.odom_writer.writerow([t, pos.x, pos.y, pos.z, ori.x, ori.y, ori.z, ori.w])

    def gt_callback(self, msg: Odometry):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        pos = msg.pose.pose.position
        ori = msg.pose.pose.orientation
        self.gt_writer.writerow([t, pos.x, pos.y, pos.z, ori.x, ori.y, ori.z, ori.w])

    def destroy_node(self):
        self.odom_file.close()
        self.gt_file.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = CSVExporter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
