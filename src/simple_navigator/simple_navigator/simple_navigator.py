import rclpy
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
import math
import time

class SimpleNavigator(Node):
    def __init__(self):
        super().__init__('simple_navigator')
        self.client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        # Wait a bit longer before sending the first goal
        self.timer = self.create_timer(5.0, self.send_goal)
        self.goal_sent = False
        self.get_logger().info('Navigator initialized, waiting 5 seconds before sending goal...')
        
    def create_pose(self, x, y, yaw):
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        # Fully define the quaternion for proper orientation
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.0  # Add explicit z coordinate
        # Calculate quaternion from yaw properly
        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.orientation.w = math.cos(yaw / 2.0)
        return pose
        
    def send_goal(self):
        if self.goal_sent:
            return
            
        # Wait longer for the server
        if not self.client.wait_for_server(timeout_sec=5.0):
            self.get_logger().warn('Waiting for NavigateToPose action server...')
            return
            
        goal = NavigateToPose.Goal()
        goal.pose = self.create_pose(2.0, 0.0, 0.0)  # Forward 2m
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        
        # Add behavior tree for navigation
        goal.behavior_tree = ""  # Use default behavior
        
        self.get_logger().info(f'Sending forward goal to (2.0, 0.0)')
        self.goal_sent = True
        
        # Cancel the timer to prevent additional goal attempts
        self.timer.cancel()
        
        send_goal_future = self.client.send_goal_async(goal)
        send_goal_future.add_done_callback(self.goal_response_callback)
        
    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected!')
            self.goal_sent = False  # Reset to allow retry
            # Restart timer to try again
            self.timer = self.create_timer(5.0, self.send_goal)
            return
            
        self.get_logger().info('Goal accepted. Waiting for result...')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)
        
    def result_callback(self, future):
        status = future.result().status
        self.get_logger().info(f'Navigation complete with status: {status}')
        
        # Allow sending another goal if desired
        self.goal_sent = False
        # Optionally restart timer to send another goal
        # self.timer = self.create_timer(5.0, self.send_goal)

def main(args=None):
    rclpy.init(args=args)
    node = SimpleNavigator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()