#!/usr/bin/env python3
#
# Authors: Himanshu kohale
# This script is for serial data receive in form of 
# Right_wheel_RMP and Left_wheel_RPM and 
# use them for cmd_vel, odom and TF calculation which will 
# further use in nav2 

import rclpy  
from rclpy.node import Node  
from geometry_msgs.msg import Twist 
from nav_msgs.msg import Odometry  
from tf_transformations import quaternion_from_euler  # To convert yaw angle to quaternion for orientation in tf
import serial  
import math 
import time 
from std_msgs.msg import Float32  


class RPMToCmdVelTFodom(Node):

    def __init__(self):

        super().__init__('rpm_to_cmd_vel_tf')

        self.declare_parameter('wheel_radius', 0.1)  
        self.declare_parameter('wheel_base', 0.5)    
        self.declare_parameter('serial_port', '/dev/ttyUSB0') 
        self.declare_parameter('baudrate', 9600)  

        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.wheel_base = self.get_parameter('wheel_base').value
        self.serial_port = self.get_parameter('serial_port').value
        self.baudrate = self.get_parameter('baudrate').value
        
        self.serial_connection = serial.Serial(self.serial_port, self.baudrate, timeout=1)
        time.sleep(2)  

        self.cmd_vel_pub = self.create_publisher(Twist, 'bcr_bot/cmd_vel', 10)
        
        self.odom_pub = self.create_publisher(Odometry, 'bcr_bot/odom', 10)
        
        self.x = 0.0  # Initial x position
        self.y = 0.0  # Initial y position
        self.theta = 0.0  # Initial orientation (yaw)

        self.create_timer(0.1, self.update)  # Calls the `update` function at 10 Hz

    def update(self):

        """
        Reads RPM values from serial, converts them to linear and angular velocities,
        publishes the velocities to /cmd_vel, and updates the robot's odometry.
        """

        rpm_data = self.serial_connection.readline().decode().strip()

        if rpm_data:
            try:

                left_rpm, right_rpm = map(float, rpm_data.split(','))
            except ValueError:

                self.get_logger().error("Failed to parse RPM data")
                return
            
            # Convert RPM to linear velocity for each wheel using the formula:
            # v = (RPM / 60) * (2 * π * wheel_radius)
            
            v_left = (left_rpm / 60) * 2 * math.pi * self.wheel_radius
            v_right = (right_rpm / 60) * 2 * math.pi * self.wheel_radius

            linear_velocity = (v_right + v_left) / 2.0
            angular_velocity = (v_right - v_left) / self.wheel_base


            twist_msg = Twist()
            twist_msg.linear.x = linear_velocity  # Set forward velocity
            twist_msg.angular.z = angular_velocity  # Set rotational velocity
            self.cmd_vel_pub.publish(twist_msg)

            # Update robot's position and orientation over the time step (dt = 0.1 seconds)
            dt = 0.1  # Time interval between updates (in seconds)
            # Update x and y positions based on current velocities and orientation
            self.x += linear_velocity * math.cos(self.theta) * dt
            self.y += linear_velocity * math.sin(self.theta) * dt
            self.theta += angular_velocity * dt  # Update orientation (theta)

            # Create an Odometry message to publish the updated position and velocity
            odom_msg = Odometry()
            odom_msg.header.stamp = self.get_clock().now().to_msg() 
            odom_msg.header.frame_id = 'odom'  
            odom_msg.child_frame_id = 'base_footprint'  

            # Set the position part of the odometry message
            odom_msg.pose.pose.position.x = self.x
            odom_msg.pose.pose.position.y = self.y

            # Convert the orientation angle (theta) to a quaternion for compatibility with ROS
            q = quaternion_from_euler(0, 0, self.theta)
            odom_msg.pose.pose.orientation.x = q[0]
            odom_msg.pose.pose.orientation.y = q[1]
            odom_msg.pose.pose.orientation.z = q[2]
            odom_msg.pose.pose.orientation.w = q[3] # Timestamp for synchronization

            odom_msg.twist.twist.linear.x = linear_velocity  
            odom_msg.twist.twist.angular.z = angular_velocity 

            self.odom_pub.publish(odom_msg)  

    def destroy_node(self):
        self.serial_connection.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = RPMToCmdVelTFodom()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
if __name__ == '__main__':
    main()