import pytest
import rclpy
from rclpy.executors import MultiThreadedExecutor
import threading

from smart_home_pytree.move_to_tree import create_move_to_tree
from test.mock.mock_robot_interface import MockRobotInterface
from test.mock.mock_action_server import MockActionServer

# Replace these with your actual action types
from shr_msgs.action import DockingRequest, NavigateToPose


@pytest.fixture(scope="module")
def rclpy_setup():
    rclpy.init()
    executor = MultiThreadedExecutor()
    
    # Keep track of nodes to spin (your mock nodes)
    nodes_to_spin = []

    # Helper function to spin executor in background
    def spin():
        executor.spin()
    
    spin_thread = threading.Thread(target=spin, daemon=True)
    spin_thread.start()
    
    yield executor, nodes_to_spin  # pass executor and nodes to the tests
    
    executor.shutdown()
    rclpy.shutdown()
    spin_thread.join(timeout=1.0)


def add_nodes_to_executor(executor, nodes):
    for node in nodes:
        executor.add_node(node)


def test_move_to_tree_success(rclpy_setup):
    executor, nodes_to_spin = rclpy_setup

    robot = MockRobotInterface(charging=True)
    undock_server = MockActionServer('undock', DockingRequest, succeed=True)
    move_server = MockActionServer('navigate_to_pose', NavigateToPose, succeed=True)

    nodes_to_spin.extend([undock_server, move_server])
    add_nodes_to_executor(executor, nodes_to_spin)

    tree = create_move_to_tree(robot)
    tree.setup(timeout=5.0)
    tree.tick_once()

    assert tree.status == tree.SUCCESS


# def test_move_to_tree_fail_move(rclpy_setup):
#     executor, nodes_to_spin = rclpy_setup

#     robot = MockRobotInterface(charging=False)
    
#     undock_server = MockActionServer('undock', DockingRequest, succeed=True)
#     move_server = MockActionServer('navigate_to_pose', NavigateToPose, succeed=False)

#     nodes_to_spin.extend([undock_server, move_server])
#     add_nodes_to_executor(executor, nodes_to_spin)

#     tree = create_move_to_tree(robot)
#     tree.setup(timeout=5.0)
#     tree.tick_once()
    
#     assert tree.status == tree.FAILURE # "Tree should fail if move_to_pose fails"
