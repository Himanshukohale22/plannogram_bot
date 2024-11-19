import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from sensor_msgs.msg import Range
from geometry_msgs.msg import PoseStamped

class ObstacleAvoidanceNode(Node):
    def __init__(self):
        super().__init__('obstacle_avoidance_node')

        # Action client to interact with bt_navigator
        self.navigator_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        
        # Subscriber to ultrasonic sensor
        self.ultrasonic_subscriber = self.create_subscription(
            Range, '/ultrasonic', self.ultrasonic_callback, 10)

        # Threshold distance to detect obstacles (1 meter)
        self.distance_threshold = 1.0

        # Track if there is an active navigation goal
        self.goal_active = False

        # Start navigating to a goal position
        self.send_navigation_goal(2.0, 2.0)  # Example coordinates

    def send_navigation_goal(self, x, y):
        # Wait for the action server to be available
        self.get_logger().info("Waiting for navigation action server...")
        self.navigator_client.wait_for_server()

        # Set up the goal pose
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.pose.position.x = x
        goal_pose.pose.position.y = y
        goal_pose.pose.orientation.w = 1.0  # Set facing direction

        # Create the NavigateToPose goal message
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = goal_pose

        # Send the goal to the action server
        self.get_logger().info(f"Sending goal to position: ({x}, {y})")
        self.goal_future = self.navigator_client.send_goal_async(goal_msg)
        self.goal_future.add_done_callback(self.goal_response_callback)
        self.goal_active = True

    def goal_response_callback(self, future):
        # Check if the goal was accepted
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn("Goal was rejected by the action server.")
            self.goal_active = False
            return

        self.get_logger().info("Goal accepted.")
        self.result_future = goal_handle.get_result_async()
        self.result_future.add_done_callback(self.result_callback)

    def result_callback(self, future):
        result = future.result().result
        self.get_logger().info("Goal reached or navigation stopped.")
        self.goal_active = False

    def ultrasonic_callback(self, msg):
        # Get the distance from the ultrasonic sensor
        distance = msg.range
        self.get_logger().info(f"Ultrasonic distance: {distance:.2f} m")

        # Check if the distance is less than the threshold
        if distance < self.distance_threshold and self.goal_active:
            self.get_logger().warn("Obstacle detected within 1 meter! Cancelling navigation.")
            self.cancel_navigation()

    def cancel_navigation(self):
        if self.goal_active:
            # Cancel the active navigation goal
            self.goal_future.cancel()
            self.get_logger().info("Navigation goal cancelled.")
            self.goal_active = False

def main(args=None):
    rclpy.init(args=args)
    obstacle_avoidance_node = ObstacleAvoidanceNode()
    rclpy.spin(obstacle_avoidance_node)
    obstacle_avoidance_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
