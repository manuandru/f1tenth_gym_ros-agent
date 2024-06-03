#!/usr/bin/env python
# from __future__ import print_function
import numpy as np
import rclpy
from rclpy.node import Node

#ROS Imports
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped

class CarNode(Node):

    def __init__(self):
        super().__init__('car_node')

        #Topics & Subs, Pubs
        lidarscan_topic = '/scan'
        drive_topic = '/drive'

        # ROS 
        self.lidar_sub = self.create_subscription(LaserScan, lidarscan_topic, self.lidar_callback, 10)
        self.drive_pub = self.create_publisher(AckermannDriveStamped, drive_topic, 10)

    def lidar_callback(self, data):
        d = np.array(data.ranges, dtype=np.float64)

        steer = 1.0 # calculate steering angle
        speed = 1.0 # calculate speed

        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = self.get_clock().now().to_msg()
        drive_msg.header.frame_id = "laser"
        drive_msg.drive.steering_angle = steer
        drive_msg.drive.speed = speed
        self.drive_pub.publish(drive_msg)

def main(args=None):
    rclpy.init(args=args)
    node = CarNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()