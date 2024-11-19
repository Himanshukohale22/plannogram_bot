#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32

class DifferentialDriveController(Node):
    def __init__(self):
        super().__init__('differential_drive_controller')

        # Subscribe to the cmd_vel topic
        self.cmd_vel_subscriber = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10
        )

        # Publishers for left and right wheel RPM
        self.left_wheel_rpm_publisher = self.create_publisher(Float32, 'left_wheel_rpm', 10)
        self.right_wheel_rpm_publisher = self.create_publisher(Float32, 'right_wheel_rpm', 10)

        # Parameters for differential drive
        self.wheel_radius = 0.05  # Wheel radius in meters
        self.wheel_base = 0.550    # Distance between wheels in meters

    def cmd_vel_callback(self, msg: Twist):

        # Extract linear and angular velocities from cmd_vel message
        linear_velocity = msg.linear.x     # Forward/backward speed (m/s)
        angular_velocity = msg.angular.z   # Rotation speed (rad/s)

        # Calculate the wheel velocities in m/s for left and right wheels
        v_left = linear_velocity - (angular_velocity * self.wheel_base / 2.0)
        v_right = linear_velocity + (angular_velocity * self.wheel_base / 2.0)

        # Convert wheel velocities from m/s to RPM
        left_wheel_rpm = (v_left / (2 * 3.1416 * self.wheel_radius)) * 60
        right_wheel_rpm = (v_right / (2 * 3.1416 * self.wheel_radius)) * 60

        # Log the calculated RPM values
        self.get_logger().info(f"Left Wheel RPM: {left_wheel_rpm:.2f}, Right Wheel RPM: {right_wheel_rpm:.2f}")

        # Publish the RPM values to their respective topics
        self.left_wheel_rpm_publisher.publish(Float32(data=left_wheel_rpm))
        self.right_wheel_rpm_publisher.publish(Float32(data=right_wheel_rpm))


def main(args=None):
    rclpy.init(args=args)
    differential_drive_controller = DifferentialDriveController()
    rclpy.spin(differential_drive_controller)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
