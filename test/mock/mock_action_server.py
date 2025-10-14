import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer

class MockActionServer(Node):
    """
    Mock action server that can succeed or fail.
    """
    def __init__(self, action_name, action_type, succeed=True):
        super().__init__(f'mock_{action_name}_server')
        self._succeed = succeed
        self._action_server = ActionServer(
            self,
            action_type,
            action_name,
            self.execute_callback
        )
 

    def execute_callback(self, goal_handle):
        goal_handle.accept_goal()
        if self._succeed:
            goal_handle.succeed()
        else:
            goal_handle.abort()
        return goal_handle.get_result()
    
    # def generate_feedback_message(self) -> py_trees_actions.Dock.Feedback:
    #     """
    #     Create a feedback message that populates the percent completed.

    #     Returns:
    #         :class:`py_trees_actions.Dock_Feedback`: the populated feedback message
    #     """
    #     msg = py_trees_actions.Dock.Feedback(
    #         percentage_completed=self.percent_completed
    #     )
    #     return msg
