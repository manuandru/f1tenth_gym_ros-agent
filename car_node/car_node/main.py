#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

import numpy as np
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped

class WallFollow(Node):
    """
    Implement Wall Following on the car
    """
    def __init__(self):
        super().__init__('wall_follow_node')

        lidarscan_topic = '/scan'
        drive_topic = '/drive'

        # TODO: create subscribers and publishers
        self.sub_scan = self.create_subscription(LaserScan, lidarscan_topic, self.scan_callback, 10)
        self.pub_drive = self.create_publisher(AckermannDriveStamped, drive_topic, 10)

        # TODO: set PID gains
        self.kp = 0.9
        self.ki = 0.0
        self.kd = 0.1

        # TODO: store history
        self.error = 0
        self.prev_error = 0
        self.integral = 0
        self.delta_t = 0.01

        # TODO: store any necessary values you think you'll need
        self.alpha = None
        self.prev_time = 0.0


    def get_range(self, range_data, angle):
        """
        Simple helper to return the corresponding range measurement at a given angle. Make sure you take care of NaNs and infs.

        Args:
            range_data: single range array from the LiDAR
            angle: between angle_min and angle_max of the LiDAR

        Returns:
            range: range measurement in meters at the given angle
        """

        # Açıyı radyandan dereceye çevir
        angle_in_degrees = np.rad2deg(angle)

        # Açıyı pozitif hale getir (örneğin -135 derece yerine 225 derece)
        positive_angle = angle_in_degrees + 135

        # Dönüşüm faktörünü uygulayın
        index = int(positive_angle * 4 - 1)

        return range_data[index]

    def get_error(self, range_data, dist):
        """
        Calculates the error to the wall. Follow the wall to the left (going counter clockwise in the Levine loop). You potentially will need to use get_range()

        Args:
            range_data: single range array from the LiDAR
            dist: desired distance to the wall

        Returns:
            error: calculated error
        """

        theta = 45
        b = self.get_range(range_data, np.pi / 2)
        a = self.get_range(range_data, np.pi / 2 - np.deg2rad(theta))
        divided = a * np.cos(np.deg2rad(theta)) - b
        dividing = a * np.sin(np.deg2rad(theta))
        self.alpha = np.arctan(divided / dividing)
        
        L = 0.7  # lookahead distance
        D_t = b * np.cos(self.alpha)
        D_t1 = D_t + L * np.sin(self.alpha)
        error = -1.0 * (dist - D_t1)

        return error

    def pid_control(self, error, velocity, delta_t):
        """
        Based on the calculated error, publish vehicle control

        Args:
            error: calculated error
            velocity: desired velocity

        Returns:
            None
        """
        drive_msg = AckermannDriveStamped()

        # TODO: Use kp, ki & kd to implement a PID controller
        self.error = error
        self.integral += self.ki * self.error * delta_t
        angle = self.kp * self.error + np.clip(self.integral, -1, +1) + self.kd * (self.error - self.prev_error) / delta_t
        self.prev_error = self.error

        # TODO: fill in drive message and publish
        drive_msg.drive.speed = velocity
        drive_msg.drive.steering_angle = angle
        self.pub_drive.publish(drive_msg)

    def scan_callback(self, scan_msg):
        """
        Callback function for LaserScan messages. Calculate the error and publish the drive message in this function.

        Args:
            msg: Incoming LaserScan message

        Returns:
            None
        """
        time_stamp_sec = float(scan_msg.header.stamp.sec)
        time_stamp_nanosec = float(scan_msg.header.stamp.nanosec) * pow(10, -9)
        curr_time = time_stamp_sec + time_stamp_nanosec
        delta_t = curr_time - self.prev_time

        error = self.get_error(np.array(scan_msg.ranges), 0.8) #threshold error


        # TODO: calculate desired car velocity based on heading angle
        if abs(self.alpha) <= np.deg2rad(10):
            velocity = 3.5
        elif abs(self.alpha) <= np.deg2rad(20):
            velocity = 2.0
        else:
            velocity = 0.5

        self.pid_control(error, velocity, delta_t)
        self.prev_time = curr_time

def main(args=None):
    rclpy.init(args=args)
    print("WallFollow Initialized")
    wall_follow_node = WallFollow()
    rclpy.spin(wall_follow_node)
    wall_follow_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
