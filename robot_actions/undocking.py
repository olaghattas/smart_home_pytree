#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse

from shr_msgs.action import DockingRequest  # using as a placeholder action


class UnDockingActionServer(Node):

    def __init__(self):
        super().__init__('undocking_action_server')

        self._action_server = ActionServer(
            self,
            DockingRequest,                     # action type
            'undock',                     # action name
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback
        )
        self.get_logger().info('Undocking action server is up and running.')

    def goal_callback(self, goal_request):
        self.get_logger().info('Received undocking goal request')
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().info('Received undocking cancel request')
        return CancelResponse.ACCEPT

    async def execute_callback(self, goal_handle):
        self.get_logger().info('Executing undocking goal...')

        # Ask user whether to succeed or fail
        user_input = input("Undocking goal received. Type 'y' to succeed, 'n' to fail: ").strip().lower()

        feedback_msg = DockingRequest.Feedback()

        # Publish one feedback to show it's working
        goal_handle.publish_feedback(feedback_msg)
        self.get_logger().info('Feedback sent.')

        if user_input == 'y':
            goal_handle.succeed()
            result = DockingRequest.Result()
            self.get_logger().info('Docking succeeded.')
            return result
        else:
            goal_handle.abort()
            result = DockingRequest.Result()
            self.get_logger().info('Docking failed.')
            return result


def main(args=None):
    rclpy.init(args=args)
    node = UnDockingActionServer()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
