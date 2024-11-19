#!/usr/bin/env python3
#
# Authors: Himanshu kohale

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
import tf_transformations
import serial
import time
import math

class localpose_receive(Node):
    def __init__(self):
        super().__init__('serial_to_cmd_vel_publisher')

        # Initialize serial communication (update port for your setup)
        self.ser = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)
        time.sleep(2)  # Allow time for serial connection to initialize

        # Publishers for cmd_vel, odom, and TF
        self.cmd_vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)

        ## publisher for odometry 
        self.odom_publisher = self.create_publisher(Odometry, 'odom', 10)

        # Initialize TF broadcaster
        self.tf_broadcaster = self.create_publisher(TransformStamped, 'tf', 10)

        # Differential drive parameters
        self.wheel_radius = 0.05  # Wheel radius in meters
        self.wheel_base = 0.550   # Distance between wheels in meters

        # Robot pose and velocities

        self.x = 0.0  # x position in meters
        self.y = 0.0  # y position in meters
        self.theta = 0.0  # Orientation in radians
        self.last_time = self.get_clock().now()   ## current time 

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

                # Update and publish odom and TF
                self.update_odometry(linear_velocity, angular_velocity)

            except ValueError:
                self.get_logger().error("Error parsing serial data: ensure data is in 'left_rpm,right_rpm' format.")
            except Exception as e:
                self.get_logger().error(f"Unexpected error: {e}")

    def update_odometry(self, linear_velocity, angular_velocity):
        current_time = self.get_clock().now()
        delta_time = (current_time - self.last_time).nanoseconds / 1e9  # Convert to seconds
        self.last_time = current_time

        # Update pose based on velocities
        delta_x = linear_velocity * math.cos(self.theta) * delta_time
        delta_y = linear_velocity * math.sin(self.theta) * delta_time
        delta_theta = angular_velocity * delta_time

        self.x += delta_x
        self.y += delta_y
        self.theta += delta_theta

        # Normalize theta
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))

        # Publish odometry
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_footprint'

        # Pose
        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.orientation = self.quaternion_from_yaw(self.theta)

        # Twist
        odom_msg.twist.twist.linear.x = linear_velocity
        odom_msg.twist.twist.angular.z = angular_velocity

        self.odom_publisher.publish(odom_msg)

        # Publish TF transformation
        self.publish_tf(current_time)

    def quaternion_from_yaw(self, yaw):
        """Convert yaw to quaternion for orientation."""
        q = tf_transformations.quaternion_from_euler(0, 0, yaw)
        return {
            'x': q[0],
            'y': q[1],
            'z': q[2],
            'w': q[3]
        }

    def publish_tf(self, time):
        tf_msg = TransformStamped()
        tf_msg.header.stamp = time.to_msg()
        tf_msg.header.frame_id = 'odom'
        tf_msg.child_frame_id = 'base_footprint'

        tf_msg.transform.translation.x = self.x
        tf_msg.transform.translation.y = self.y
        tf_msg.transform.translation.z = 0.0

        quaternion = tf_transformations.quaternion_from_euler(0, 0, self.theta)
        tf_msg.transform.rotation.x = quaternion[0]
        tf_msg.transform.rotation.y = quaternion[1]
        tf_msg.transform.rotation.z = quaternion[2]
        tf_msg.transform.rotation.w = quaternion[3]

        self.tf_broadcaster.publish(tf_msg)

    def close_serial_connection(self):
        if self.ser.is_open:
            self.ser.close()
            self.get_logger().info("Closed serial connection.")

    def __del__(self):
        self.close_serial_connection()


def main(args=None):
    rclpy.init(args=args)
    serial_to_cmd_vel_publisher = localpose_receive()
    rclpy.spin(serial_to_cmd_vel_publisher)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
