#!/usr/bin/env python3
#
# Authors: Himanshu Kohale

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from math import atan2


class OdomSubscriber(Node):
    def __init__(self):
        super().__init__('odom_subscriber')

        # Subscribe to the 'odom' topic
        self.create_subscription(Odometry, 'bcr_bot/odom', self.sub_odometry, 10)

    def sub_odometry(self, msg):
        # Extract position
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        z = msg.pose.pose.position.z

        # Extract orientation (quaternion)
        qx = msg.pose.pose.orientation.x
        qy = msg.pose.pose.orientation.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w

        # Compute yaw from the quaternion
        yaw = atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy**2 + qz**2))

        # Log position and yaw
        self.get_logger().info(f"Position -> x: {x:.2f}, y: {y:.2f}, z: {z:.2f}")
        self.get_logger().info(f"Orientation -> yaw: {yaw:.2f} rad")


def main(args=None):
    rclpy.init(args=args)
    node = OdomSubscriber()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
