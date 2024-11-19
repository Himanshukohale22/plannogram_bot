#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist

class RpmToCmdVelConverter(Node):
    def __init__(self):
        super().__init__('rpm_to_cmd_vel_converter')

        # Subscribe to RPM topics for left and right wheels
        self.left_rpm_sub = self.create_subscription(Float32, 'left_wheel_rpm', self.left_rpm_callback, 10)
        self.right_rpm_sub = self.create_subscription(Float32, 'right_wheel_rpm', self.right_rpm_callback, 10)

        # Publisher for cmd_vel to control the other robot
        self.cmd_vel_pub = self.create_publisher(Twist, 'bcr_bot/cmd_vel', 10)

        # Parameters for the differential drive system
        self.wheel_radius = 0.1   # Radius of the wheels (in meters)
        self.wheel_base = 0.5     # Distance between the wheels (in meters)

        # Store RPM values from the two wheels
        self.left_rpm = 0.0
        self.right_rpm = 0.0

    def left_rpm_callback(self, msg: Float32):
        self.left_rpm = msg.data
        self.calculate_and_publish_cmd_vel()

    def right_rpm_callback(self, msg: Float32):
        self.right_rpm = msg.data
        self.calculate_and_publish_cmd_vel()

    def calculate_and_publish_cmd_vel(self):
        # Convert RPM to m/s for each wheel
        v_left = (self.left_rpm / 60.0) * (2 * 3.1416 * self.wheel_radius)
        v_right = (self.right_rpm / 60.0) * (2 * 3.1416 * self.wheel_radius)

        # Calculate linear and angular velocities
        linear_velocity = (v_right + v_left) / 2.0
        angular_velocity = (v_right - v_left) / self.wheel_base

        # Create and publish the cmd_vel message
        cmd_vel_msg = Twist()
        cmd_vel_msg.linear.x = linear_velocity
        cmd_vel_msg.angular.z = angular_velocity
        self.cmd_vel_pub.publish(cmd_vel_msg)

        # Logging the converted velocities
        self.get_logger().info(f"Published cmd_vel: linear={linear_velocity:.2f} m/s, angular={angular_velocity:.2f} rad/s")

def main(args=None):
    rclpy.init(args=args)
    rpm_to_cmd_vel_converter = RpmToCmdVelConverter()
    rclpy.spin(rpm_to_cmd_vel_converter)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
