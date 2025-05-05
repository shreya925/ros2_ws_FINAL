import rclpy
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from rcl_interfaces.srv import SetParameters
import time

class SimpleNavigator(Node):
    def __init__(self):
        super().__init__('simple_navigator')
        self.client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.param_client = self.create_client(
            SetParameters, '/controller_server/set_parameters')
        self.goals = [
            self.create_pose(1.0, 0.0, 0.0),
            self.create_pose(1.0, 1.0, 1.57),
            self.create_pose(0.0, 1.0, 3.14)
        ]
        self.current_goal_index = 0
        self.timer = self.create_timer(2.0, self.send_next_goal)

    def create_pose(self, x, y, yaw):
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.orientation.w = math.cos(yaw / 2.0)
        return pose

    def send_next_goal(self):
        if not self.client.wait_for_server(timeout_sec=5.0):
            self.get_logger().warn('Waiting for NavigateToPose action server...')
            return

        if self.current_goal_index >= len(self.goals):
            self.get_logger().info('All goals sent.')
            self.timer.cancel()
            return

        goal = NavigateToPose.Goal()
        goal.pose = self.goals[self.current_goal_index]
        self.get_logger().info(f'Sending goal #{self.current_goal_index + 1}...')
        self.current_goal_index += 1

        send_goal_future = self.client.send_goal_async(goal)
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected!')
            return

        self.get_logger().info('Goal accepted, waiting for result...')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    def result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Goal result: {result}')
        self.send_next_goal()

import math
def main(args=None):
    rclpy.init(args=args)
    node = SimpleNavigator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()