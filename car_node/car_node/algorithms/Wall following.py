import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped

class WallFollower(Node):
    def __init__(self):
        super().__init__('wall_follower')
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.lidar_callback,
            10)
        self.publisher_ = self.create_publisher(AckermannDriveStamped, '/drive', 10)
        self.target_distance = 1.0
        self.current_distance = 0.0
        self.kp = 1.0
        self.speed = 1.0

    def lidar_callback(self, msg):

        left_distances = msg.ranges[:90]
        self.current_distance = min(left_distances)
        self.wall_following_control()

    def wall_following_control(self):
        
        error = self.target_distance - self.current_distance
        steering_angle = self.kp * error
        drive_msg = AckermannDriveStamped()
        drive_msg.drive.speed = self.speed
        drive_msg.drive.steering_angle = steering_angle
        self.publisher_.publish(drive_msg)

def main(args=None):
    rclpy.init(args=args)
    wall_follower = WallFollower()
    rclpy.spin(wall_follower)
    wall_follower.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
