#!/usr/bin/env python3
#
# Authors: Himanshu kohale
#
# This script is used to send serial data to port ACM0
# in form of  Right_wheel_RPM and Left_wheel_RPM

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
import serial
import time

class DifferentialDriveController(Node):
    def __init__(self):
        super().__init__('differential_drive_controller')

        # Initialize serial communication (replace 'COM3' with your serial port, or '/dev/ttyUSB0' on Linux)
        self.ser = serial.Serial('/dev/ttyACM0', 9600, timeout=1)
        time.sleep(2)  # Wait for serial connection to initialize

        # Subscribe to the cmd_vel topic
        self.cmd_vel_subscriber = self.create_subscription(
            Twist,
            '/bcr_bot/cmd_vel',
            self.cmd_vel_callback,
            10
        )

        # Publishers for left and right wheel RPM
        self.left_wheel_rpm_publisher = self.create_publisher(Float32, 'left_wheel_rpm', 10)
        self.right_wheel_rpm_publisher = self.create_publisher(Float32, 'right_wheel_rpm', 10)

        # Parameters for differential drive
        self.wheel_radius = 0.05  # Wheel radius in meters
        self.wheel_base = 0.550   # Distance between wheels in meters

        # self.timers = self.create_timer(0.1,self.cmd_vel_callback)

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

        # Send RPM values over serial
        self.send_rpm_over_serial(left_wheel_rpm, right_wheel_rpm)

    def send_rpm_over_serial(self, left_rpm, right_rpm):
        # Format the data as a string, then encode and send it over serial
        rpm_data = f"{left_rpm:.2f},{right_rpm:.2f}\n"
        self.ser.write(rpm_data.encode())
        self.get_logger().info(f"Sent over serial: {rpm_data.strip()}")

    def __del__(self):
        # Close the serial connection on object deletion
        if self.ser.is_open:
            self.ser.close()

def main(args=None):
    rclpy.init(args=args)
    differential_drive_controller = DifferentialDriveController()
    rclpy.spin(differential_drive_controller)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
