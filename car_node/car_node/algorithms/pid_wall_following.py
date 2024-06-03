import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped

class PIDController:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.previous_error = 0.0
        self.integral = 0.0

    def control(self, error, dt):
        self.integral += error * dt
        derivative = (error - self.previous_error) / dt
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        self.previous_error = error
        return output

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
        self.speed = 1.0
        self.pid_controller = PIDController(kp=1.0, ki=0.0, kd=0.1)

    def lidar_callback(self, msg):

        left_distances = msg.ranges[:90]
        current_distance = min(left_distances)
        self.wall_following_control(current_distance)

    def wall_following_control(self, current_distance):

        error = self.target_distance - current_distance
        dt = 0.1
        steering_angle = self.pid_controller.control(error, dt)
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

