import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
import requests

class ClientNode(Node):
    def __init__(self):
        super().__init__('client_node')
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.lidar_callback,
            10)
        self.publisher_ = self.create_publisher(AckermannDriveStamped, '/drive', 10)

    def lidar_callback(self, msg):

        calculated_values = requests.post(
            'http://server:5000/calculate',
            json = {
                'stamp': {
                    'sec': float(msg.header.stamp.sec),
                    'nanosec': float(msg.header.stamp.nanosec),
                },
                'ranges': list(msg.ranges),
            }
        ).json()

        # print(calculated_values)

        drive_msg = AckermannDriveStamped()
        drive_msg.drive.speed = float(calculated_values['speed'])
        drive_msg.drive.steering_angle = float(calculated_values['steering_angle'])
        self.publisher_.publish(drive_msg)

def main(args=None):
    rclpy.init(args=args)
    wall_follower = ClientNode()
    rclpy.spin(wall_follower)
    wall_follower.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
