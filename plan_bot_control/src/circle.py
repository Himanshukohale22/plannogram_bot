#! /usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class circle(Node):
    def __init__(self):
        super().__init__('circle_run')
        # Create a publisher for the bcr_bot/cmd_vel topic
        self.publisher = self.create_publisher(Twist, '/cmd_vel',10)
        self.timer = self.create_timer(1,self.cmd_vel_callback)  # 0.1 hz 
     
    def cmd_vel_callback(self):

        msg = Twist()
        msg.linear.x = 0.8
        msg.linear.y = 0.0 
        msg.angular.z = -1.0
        # Republish the received message to bcr_bot/cmd_vel
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node_driver = circle()
    rclpy.spin(node_driver)
    node_driver.destroy_node()

    rclpy.shutdown()

if __name__ == '__main__':
    main()

