#!/usr/bin/env python3
#
# Authors: Himanshu kohale

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32
from tf2_ros import Buffer, TransformListener
from tf_transformations import euler_from_quaternion
from math import atan2
import serial
import time

WHEEL_RADIUS = 0.05  # in meters
WHEEL_SEPARATION = 0.550  # in meters

class LocalPoseSend(Node):
    def __init__(self):
        super().__init__('local_pose_robot')

        # # Set up serial communication
        # self.ser = serial.Serial('/dev/ttyACM0', 9600, timeout=10)
        # time.sleep(2)  # Allow time to initialize serial port

        # Subscribers
        self.create_subscription(Twist, 'bcr_bot/cmd_vel', self.cmd_vel_callback, 10)
        self.timer = self.create_timer(0.1, self.cmd_vel_callback)

        # self.create_subscription(Odometry, 'bcr_bot/odom', self.odom_callback, 10)
        # self.timer = self.create_timer(0.1, self.odom_callback)


        # TF listener
        # self.tf_buffer = Buffer()
        # self.tf_listener = TransformListener(self.tf_buffer, self)
        # self.timer = self.create_timer(0.1, self.get_transform)

        # Publishers
        self.left_wheel_rpm_publisher = self.create_publisher(Float32, 'left_wheel_rpm', 10)
        self.right_wheel_rpm_publisher = self.create_publisher(Float32, 'right_wheel_rpm', 10)

    def cmd_vel_callback(self, msg):
        # Extract linear and angular velocities
        linear_vel = msg.linear.x
        angular_vel = msg.angular.z

        # Calculate wheel velocities
        v_left = linear_vel - (angular_vel * WHEEL_SEPARATION / 2.0)
        v_right = linear_vel + (angular_vel * WHEEL_SEPARATION / 2.0)

        # Convert velocities to RPM
        left_wheel_rpm = (v_left / (2 * 3.1416 * WHEEL_RADIUS)) * 60
        right_wheel_rpm = (v_right / (2 * 3.1416 * WHEEL_RADIUS)) * 60

        # Publish wheel RPMs
        self.left_wheel_rpm_publisher.publish(Float32(data=left_wheel_rpm))
        self.right_wheel_rpm_publisher.publish(Float32(data=right_wheel_rpm))

        # Log and send data over serial
        self.get_logger().info(f"Left Wheel RPM: {left_wheel_rpm:.2f}, Right Wheel RPM: {right_wheel_rpm:.2f}")
        # self.send_rpm_over_serial(left_wheel_rpm, right_wheel_rpm)

    # def odom_callback(self, msg):
    #     # Extract position and orientation
    #     position = msg.pose.pose.position
    #     orientation = msg.pose.pose.orientation

    #     x, y, z = position.x, position.y, position.z
    #     quaternion = (orientation.x, orientation.y, orientation.z, orientation.w)
    #     yaw = atan2(2.0 * (quaternion[3] * quaternion[2] + quaternion[0] * quaternion[1]),
    #                 1.0 - 2.0 * (quaternion[1]**2 + quaternion[2]**2))

    #     # # Log odometry data
    #     self.get_logger().info(f"Position -> x: {x:.2fs}, y: {y:.2f}, z: {z:.2f}")
    #     self.get_logger().info(f"Yaw: {yaw:.2f} rad")

    # def get_transform(self):
    #     try:
    #         # Query the transform
    #         transform = self.tf_buffer.lookup_transform('base_footprint', 'bcr_bot/odom', rclpy.time.Time())
    #         translation = transform.transform.translation
    #         rotation = transform.transform.rotation

    #         x, y, z = translation.x, translation.y, translation.z
    #         quaternion = (rotation.x, rotation.y, rotation.z, rotation.w)
    #         roll, pitch, yaw = euler_from_quaternion(quaternion)

    #     #     # Log TF data
    #         self.get_logger().info(f"TF Translation -> x: {x:.2f}, y: {y:.2f}, z: {z:.2f}")
    #         self.get_logger().info(f"TF Rotation -> roll: {roll:.2f}, pitch: {pitch:.2f}, yaw: {yaw:.2f}")

    #     except Exception as e:
    #         self.get_logger().warn(f"TF lookup failed: {e}")

    # def send_rpm_over_serial(self, left_rpm, right_rpm):
    #     # Send RPM values over serial
    #     serial_data = f"{left_rpm:.2f},{right_rpm:.2f}\n"
    #     self.ser.write(serial_data.encode())
    #     self.get_logger().info(f"Sent over serial: {serial_data.strip()}")

    # def close_serial_connection(self):
    #     if self.ser.is_open:
    #         self.ser.close()

    # def __del__(self):
    #     self.close_serial_connection()

def main(args=None):
    rclpy.init(args=args)
    node = LocalPoseSend()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
