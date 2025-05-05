import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
import math
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

class Forward2Meters(Node):
    def __init__(self):
        super().__init__('forward_2_meters')
        self.client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        # Setup transform listener to get robot's current position
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        self.timer = self.create_timer(2.0, self.send_goal)  # Delay so ROS is ready

    def send_goal(self):
        self.timer.cancel()  # Send only one goal
        
        if not self.client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('NavigateToPose action server not available.')
            return
        
        try:
            # Get the robot's current position in the map frame
            transform = self.tf_buffer.lookup_transform(
                'map',
                'base_link',  # This should be your robot's base frame
                rclpy.time.Time())
            
            current_x = transform.transform.translation.x
            current_y = transform.transform.translation.y
            
            # Get current orientation as Euler angles (yaw)
            qz = transform.transform.rotation.z
            qw = transform.transform.rotation.w
            current_yaw = 2 * math.atan2(qz, qw)
            
            # Calculate goal position 2 meters forward of current position
            goal_x = current_x + 2.0 * math.cos(current_yaw)
            goal_y = current_y + 2.0 * math.sin(current_yaw)
            
            # Create and send goal
            goal_msg = NavigateToPose.Goal()
            goal_msg.pose = self.create_pose(goal_x, goal_y, current_yaw)
            
            self.get_logger().info(f'Sending goal to move 2 meters forward from current position')
            self.get_logger().info(f'Current: ({current_x:.2f}, {current_y:.2f}), Goal: ({goal_x:.2f}, {goal_y:.2f})')
            
            send_goal_future = self.client.send_goal_async(goal_msg)
            send_goal_future.add_done_callback(self.goal_response_callback)
            
        except TransformException as ex:
            self.get_logger().error(f'Could not get robot position: {ex}')
            return

    def create_pose(self, x, y, yaw):
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.0
        
        # Set orientation using quaternion
        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.orientation.w = math.cos(yaw / 2.0)
        
        return pose

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected!')
            return
        
        self.get_logger().info('Goal accepted. Waiting for result...')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    def result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Navigation finished.')

def main(args=None):
    rclpy.init(args=args)
    node = Forward2Meters()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()