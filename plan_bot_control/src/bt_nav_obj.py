#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from sensor_msgs.msg import Range
from geometry_msgs.msg import PoseStamped
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import Twist
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult


class ObjectAvoidanceNavigator(Node):
    def __init__(self):
        super().__init__('object_avoidance_navigator')

        # Action client for navigating to a goal
        self.navigate_action_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')

        # Subscriber for the ultrasonic sensor
        self.ultrasonic_subscriber = self.create_subscription(
            Range, '/ultrasonic', self.ultrasonic_callback, 10)
        
        self.twist_publisher = self.create_publisher(Twist, '/bcr_bot/cmd_vel', 10)

        # Threshold distance for object detection (1 meter)
        self.distance_threshold = 1.0

        # Track whether there is an active goal
        self.goal_active = False

        # Start navigation to a specified goal
        self.navigate_to_goal(2.0, 2.0)  # example goal

    def navigate_to_goal(self, x, y):
        # Wait until the action server is available
        navigator = BasicNavigator()
        self.get_logger().info("Waiting for 'navigate_to_pose' action server...")
        self.navigate_action_client.wait_for_server()

        # Create the goal pose message
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.pose.position.x = x
        goal_pose.pose.position.y = y
        goal_pose.pose.orientation.w = 1.0  # Facing forward

        # Create the goal message for NavigateToPose
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = goal_pose

        # Send the goal
        self.get_logger().info(f"Sending goal to position: ({x}, {y})")
        self.goal_handle = self.navigate_action_client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback)
        self.goal_active = True

    def feedback_callback(self, feedback):
        self.get_logger().info(f"Navigation feedback received.")

    def ultrasonic_callback(self, msg):
        # Get the distance reading from the ultrasonic sensor
        distance = msg.range
        self.get_logger().info(f'Ultrasonic distance: {distance:.2f} m')

        # Check if the distance is below the threshold
        if distance < self.distance_threshold and self.goal_active:
            self.get_logger().warn("Obstacle detected within 1 meter! Cancelling goal...")
            goal_msg = NavigateToPose
            # Cancel the current navigation goal
            # self.cancel_navigation()
            BasicNavigator.cancelTask()
            
        

    def cancel_navigation(self):
        if self.goal_handle:
            self.goal_handle.cancel_goal_async()
            self.goal_active = False
            self.get_logger().info("Navigation goal cancelled.")

    def vel_cancel(self):
        if self.goal_handle:
            self.goal_handle

def main(args=None):
    rclpy.init(args=args)
    
    # Create the ObjectAvoidanceNavigator node
    navigator = ObjectAvoidanceNavigator()

    # Run the node with a multi-threaded executor to handle callbacks concurrently
    executor = MultiThreadedExecutor()
    executor.add_node(navigator)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass

    navigator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
