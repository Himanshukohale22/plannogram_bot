#!/usr/bin/env python3
#
# Authors: Himanshu kohale

import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener
from geometry_msgs.msg import TransformStamped
from tf_transformations import euler_from_quaternion

class TFSubscriber(Node):
    def __init__(self):
        super().__init__('tf_subscriber')

        # Create a TF2 buffer and listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Timer to periodically query TF at 10 Hz
        self.timer = self.create_timer(0.1, self.get_transform)

        # Frames to query
        self.source_frame = "odom"          # Replace with your source frame
        self.target_frame = "base_footprint"  # Replace with your target frame

    def get_transform(self):
        try:
            # Lookup the transformation
            transform: TransformStamped = self.tf_buffer.lookup_transform(
                self.target_frame,
                self.source_frame,
                rclpy.time.Time()
            )

            # Extract translation
            translation = transform.transform.translation
            x, y, z = translation.x, translation.y, translation.z

            # Extract rotation (quaternion) and convert to Euler angles
            rotation = transform.transform.rotation
            quaternion = (rotation.x, rotation.y, rotation.z, rotation.w)
            roll, pitch, yaw = euler_from_quaternion(quaternion)

            # Log the transformation data
            self.get_logger().info(f"Translation -> x: {x:.2f}, y: {y:.2f}, z: {z:.2f}")
            self.get_logger().info(f"Rotation -> roll: {roll:.2f}, pitch: {pitch:.2f}, yaw: {yaw:.2f}")

        except Exception as e:
            self.get_logger().warn(f"Could not get transform: {e}")

def main(args=None):
    rclpy.init(args=args)
    tf_subscriber = TFSubscriber()
    rclpy.spin(tf_subscriber)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
