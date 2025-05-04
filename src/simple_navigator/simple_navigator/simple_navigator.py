import rclpy
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
import math

class SimpleNavigator(Node):
    def __init__(self):
        super().__init__('simple_navigator')
        self.client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.goals = [
            self.create_pose(1.0, 0.0, 0.0),
            self.create_pose(0.0, 0.0, 3.14)
        ]
        self.current_goal_index = 0
        self.goal_active = False
        self.timer = self.create_timer(1.0, self.send_next_goal)

    def create_pose(self, x, y, yaw):
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.orientation.w = math.cos(yaw / 2.0)
        return pose

    def send_next_goal(self):
        if self.goal_active:
            return

        if not self.client.wait_for_server(timeout_sec=1.0):
            self.get_logger().warn('Waiting for NavigateToPose action server...')
            return

        goal = NavigateToPose.Goal()
        goal.pose = self.goals[self.current_goal_index]
        goal.pose.header.stamp = self.get_clock().now().to_msg()

        self.get_logger().info(f'Sending goal #{self.current_goal_index + 1} to position '
                               f'({goal.pose.pose.position.x}, {goal.pose.pose.position.y})')
        self.goal_active = True
        self.current_goal_index = (self.current_goal_index + 1) % len(self.goals)

        send_goal_future = self.client.send_goal_async(goal)
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected!')
            self.goal_active = False
            return

        self.get_logger().info('Goal accepted. Waiting for result...')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    def result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Goal completed with status: {result}')
        self.goal_active = False

def main(args=None):
    rclpy.init(args=args)
    node = SimpleNavigator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()