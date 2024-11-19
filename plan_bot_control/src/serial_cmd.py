#!/usr/bin/env python3
#
# Authors: Himanshu kohale

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial
import time

class SerialToCmdVelPublisher(Node):
    def __init__(self):
        super().__init__('serial_to_cmd_vel_publisher')

        # Initialize serial communication (update port for your setup)
        self.ser = serial.Serial('/dev/ttyACM0', 9600, timeout=1)
        time.sleep(2)  # Allow time for serial connection to initialize

        # Publisher for cmd_vel topic
        self.cmd_vel_publisher = self.create_publisher(Twist, 'bcr_bot/cmd_vel', 10)

        # Differential drive parameters (same as in the sending node)
        self.wheel_radius = 0.05  # Wheel radius in meters
        self.wheel_base = 0.550   # Distance between wheels in meters

        # Timer for reading serial data and publishing cmd_vel at 10 Hz
        self.timer = self.create_timer(0.1, self.read_and_publish_cmd_vel)

    def read_and_publish_cmd_vel(self):
        
        if self.ser.in_waiting > 0:
            line = self.ser.readline().decode().strip()  # Read and decode serial line
            self.get_logger().info(f"Received from serial: {line}")

            try:
                # Split and parse RPM values
                left_rpm, right_rpm = map(float, line.split(','))

                # Convert RPM to m/s for each wheel
                v_left = (left_rpm / 60) * (2 * 3.1416 * self.wheel_radius)
                v_right = (right_rpm / 60) * (2 * 3.1416 * self.wheel_radius)

                # Compute linear and angular velocities
                linear_velocity = (v_left + v_right) / 2
                angular_velocity = (v_right - v_left) / self.wheel_base

                # Create and publish cmd_vel message
                cmd_vel_msg = Twist()
                cmd_vel_msg.linear.x = linear_velocity
                cmd_vel_msg.angular.z = angular_velocity

                self.cmd_vel_publisher.publish(cmd_vel_msg)
                self.get_logger().info(f"Published cmd_vel: linear={linear_velocity:.2f}, angular={angular_velocity:.2f}")

            except ValueError:
                self.get_logger().error("Error parsing serial data: ensure data is in 'left_rpm,right_rpm' format.")
            except Exception as e:
                self.get_logger().error(f"Unexpected error: {e}")

    def close_serial_connection(self):
        if self.ser.is_open:
            self.ser.close()
            self.get_logger().info("Closed serial connection.")

    def __del__(self):
        self.close_serial_connection()

def main(args=None):
    rclpy.init(args=args)
    serial_to_cmd_vel_publisher = SerialToCmdVelPublisher()
    rclpy.spin(serial_to_cmd_vel_publisher)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
