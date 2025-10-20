#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse

from shr_msgs.action import DockingRequest  # using as a placeholder action
from .generic_action_server import run_action_server, GenericActionServer
from rclpy.executors import MultiThreadedExecutor
import time

class DockingActionServer(GenericActionServer):
    def __init__(self):
        super().__init__(DockingRequest, "undock")

    def execute_callback(self, goal_handle):
        # Do the docking logic here
        feedback = self._action_type.Feedback()
        for i in range(100):
            print("i",i)
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                return self._action_type.Result()
            feedback.percentage_completed = float(i)
            goal_handle.publish_feedback(feedback)
        time.sleep(0.01)
        goal_handle.succeed()
        print(self._action_type.Result())
        return self._action_type.Result()
    
  
def main():
    run_action_server(DockingActionServer)  
    
# ----------------------------
# Use generic main to run the server
# ----------------------------
if __name__ == "__main__":
    run_action_server(DockingActionServer)  
    
    
## this works
# if __name__ == "__main__":
#      # Replace with your actual action
#     rclpy.init()

#     server = DockingActionServer()
#     executor = MultiThreadedExecutor()
#     try:
#         executor.add_node(server)
#         executor.spin()
#     except KeyboardInterrupt:
#         pass
#     finally:
#         server.shutdown()
#         rclpy.try_shutdown()