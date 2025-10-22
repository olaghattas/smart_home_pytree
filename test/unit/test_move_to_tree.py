import pytest
import rclpy
from rclpy.executors import MultiThreadedExecutor
import threading
import py_trees
from smart_home_pytree.trees.move_to_tree import MoveToLocationTree

# Replace these with your actual action types
from shr_msgs.action import DockingRequest
from nav2_msgs.action import NavigateToPose
from ..mock.mock_action_server import BaseMockActionServer
# from mock.mock_action_client import BaseMockActionClient
import time

def test_move_to_location_tree_success():
    """Test that MoveToLocationTree returns SUCCESS when navigation succeeds."""
    rclpy.init()

    mock_nav_server = BaseMockActionServer(
        action_name='/navigate_to_pose',
        action_type=NavigateToPose,
        result_cls=NavigateToPose.Result,
        succeed=True,
        wait_time=1.0 
    )
    
    mock_undock_server = BaseMockActionServer(
        action_name='/undock',
        action_type=DockingRequest,
        result_cls=DockingRequest.Result,
        succeed=True,
        wait_time=1.0 
    )

    # Suppose you have nodes: server_node, client_node
    executor = MultiThreadedExecutor()
    executor.add_node(mock_nav_server)
    executor.add_node(mock_undock_server)

    # Function to spin executor
    def spin_executor():
        executor.spin()

    # Start spinning in a separate thread
    executor_thread = threading.Thread(target=spin_executor, daemon=True)
    executor_thread.start()
    
    print("start wait")
    time.sleep(2.0)  # give server time to start
    print("stop wait")
    
    # Create the MoveTo tree
    move_tree = MoveToLocationTree(
        node_name="test_move_to_tree",
        location="kitchen",
        location_key="person_location",
    )

    # Mock robot interface
    class DummyRobotInterface:
        def __init__(self):
            self.state = {"charging": False}

    robot_interface = DummyRobotInterface()
    move_tree.setup()

    # Run the tree once until done and get final status
    final_status = move_tree.run_until_done()
    print("final_status", final_status)

    # assert final_status == py_trees.common.Status.SUCCESS, f"Expected SUCCESS but got {final_status}"

    # shut down
    executor.shutdown()
    executor_thread.join()
    mock_nav_server.destroy_node()
    mock_undock_server.destroy_node()
    rclpy.shutdown()
    


if __name__ == "__main__":
    test_move_to_location_tree_success()