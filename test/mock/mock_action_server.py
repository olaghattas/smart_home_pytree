import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.action.server import GoalResponse, CancelResponse

import time
class BaseMockActionServer(Node):
    """
    Generic mock action server for testing.
    You can configure it to succeed, fail, or cancel goals.
    """

    def __init__(self, action_name, action_type, result_cls, succeed=True, cancel=False, wait_time=0.0):
        """
        Args:
            action_name (str): Name of the action
            action_type (Action): Action type 
            result_cls: The Result message type 
            succeed (bool): Whether to succeed the goal
            cancel (bool): Whether to simulate cancellation
            wait_time (float): Time (seconds) to wait before finishing
        """
        super().__init__(f'mock_{action_name.strip("/")}_server')
        self._succeed = succeed
        self._cancel = cancel
        self._result_cls = result_cls
        self._wait_time = wait_time

        self._check_period = 1
        # Create the Action Server
        self._action_server = ActionServer(
            self,
            action_type,
            action_name,
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback
        )

        self.get_logger().info(
            f"Mock action server '{action_name}' ready "
            f"(succeed={succeed}, cancel={cancel}, wait_time={wait_time}s)"
        )

    def goal_callback(self, goal_request):
        """Handle incoming goal requests."""
        self.get_logger().info('Goal received.')
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        """Handle goal cancellation requests."""
        self.get_logger().info('Cancel request received.')
        return CancelResponse.ACCEPT

    
    def execute_callback(self, goal_handle):
        self.get_logger().info(f"Executing goal, emulating {self._wait_time}s duration...")
        start = time.time()

        # Wait in small increments so we can respond to cancel requests
        elapsed = 0.0
        while elapsed < self._wait_time:
            if goal_handle.is_cancel_requested:
                self.get_logger().info('Goal canceled by client.')
                goal_handle.canceled()
                result = self._result_cls()
                return result

            time.sleep(self._check_period)  # non-blocking-ish delay
            elapsed = time.time() - start

        # Post-wait: determine outcome
        if self._cancel:
            self.get_logger().info('Simulating server-side cancellation.')
            goal_handle.canceled()
        elif self._succeed:
            self.get_logger().info('Simulating goal success.')
            goal_handle.succeed()
        else:
            self.get_logger().info('Simulating goal failure.')
            goal_handle.abort()

        result = self._result_cls()
        return result

