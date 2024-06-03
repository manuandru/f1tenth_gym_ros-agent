import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped

class PIDController(Node):
    def __init__(self):
        super().__init__('pid_controller')
        self.publisher_ = self.create_publisher(AckermannDriveStamped, '/drive', 10)
        self.subscription = self.create_subscription(LaserScan, '/scan', self.laser_callback, 10)
        self.setpoint = 0
        self.Kp = 0.5  # Proportional gain
        self.Ki = 0.1  # Integral gain
        self.Kd = 0.2  # Derivative gain
        self.prev_error = 0
        self.integral = 0

    def laser_callback(self, msg):
        feedback = min(msg.ranges)
        control_signal = self.compute_control_signal(feedback)
        self.publish_control_signal(control_signal)

    def compute_control_signal(self, feedback):
        error = self.setpoint - feedback
        self.integral += error
        derivative = error - self.prev_error
        output = self.Kp * error + self.Ki * self.integral + self.Kd * derivative
        self.prev_error = error
        return output

    def publish_control_signal(self, control_signal):
        drive_msg = AckermannDriveStamped()
        drive_msg.drive.speed = 1.0
        drive_msg.drive.steering_angle = control_signal
        self.publisher_.publish(drive_msg)

def main(args=None):
    rclpy.init(args=args)
    pid_controller = PIDController()
    rclpy.spin(pid_controller)
    pid_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
