import numpy as np
import cvxpy as cp
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped

class MPCController(Node):
    def __init__(self):
        super().__init__('mpc_controller')
        self.publisher_ = self.create_publisher(AckermannDriveStamped, '/drive', 10)
        self.timer = self.create_timer(0.1, self.control_loop)

        # MPC parameters
        self.N = 10  # Prediction horizon
        self.dt = 0.1  # Time step
        self.L = 0.3  # Wheelbase

        # Initialize state and control input
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.v = 0.0
        self.delta = 0.0

        # Reference trajectory (for simplicity, let's assume a straight line)
        self.x_ref = np.linspace(0, 10, self.N)
        self.y_ref = np.zeros(self.N)
        self.theta_ref = np.zeros(self.N)
        self.v_ref = np.ones(self.N) * 1.0  # Constant speed

    def control_loop(self):
        # Solve the MPC optimization problem
        x = cp.Variable(self.N)
        y = cp.Variable(self.N)
        theta = cp.Variable(self.N)
        v = cp.Variable(self.N)
        delta = cp.Variable(self.N - 1)

        # Objective function
        cost = cp.sum_squares(x - self.x_ref) + cp.sum_squares(y - self.y_ref) + cp.sum_squares(theta - self.theta_ref) + cp.sum_squares(v - self.v_ref)

        # Constraints
        constraints = []
        for t in range(self.N - 1):
            constraints += [
                x[t + 1] == x[t] + v[t] * cp.cos(theta[t]) * self.dt,
                y[t + 1] == y[t] + v[t] * cp.sin(theta[t]) * self.dt,
                theta[t + 1] == theta[t] + v[t] / self.L * cp.tan(delta[t]) * self.dt,
                v[t] >= 0,  # Speed must be non-negative
                cp.abs(delta[t]) <= np.pi / 4  # Steering angle constraints
            ]

        # Initial conditions
        constraints += [x[0] == self.x, y[0] == self.y, theta[0] == self.theta, v[0] == self.v]

        # Solve the optimization problem
        prob = cp.Problem(cp.Minimize(cost), constraints)
        prob.solve()

        # Extract the control inputs
        self.v = v.value[1]
        self.delta = delta.value[0]

        # Update the state
        self.x += self.v * np.cos(self.theta) * self.dt
        self.y += self.v * np.sin(self.theta) * self.dt
        self.theta += self.v / self.L * np.tan(self.delta) * self.dt

        # Publish the control commands
        drive_msg = AckermannDriveStamped()
        drive_msg.drive.speed = self.v
        drive_msg.drive.steering_angle = self.delta
        self.publisher_.publish(drive_msg)

def main(args=None):
    rclpy.init(args=args)
    mpc_controller = MPCController()
    rclpy.spin(mpc_controller)
    mpc_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
