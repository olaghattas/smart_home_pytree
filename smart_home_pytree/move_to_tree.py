#!/usr/bin/env python3

"""
This script is repsponsible for creating the robot tree to move the robot to a location.

The tree receives an (x,y) position and a quaternion representing the target pose for the robot. Before initiating movement, it checks whether the robot is currently charging and, if so, commands it to undock first.
"""

import py_trees
import py_trees_ros.trees
from nav2_msgs.action import NavigateToPose
import py_trees.console as console
import rclpy
import sys
from geometry_msgs.msg import PoseStamped, Quaternion
from std_msgs.msg import Bool
import operator
from shr_msgs.action import DockingRequest
import py_trees_ros_interfaces.action as py_trees_actions  # noqa

from py_trees import display

from smart_home_pytree.util_behaviors import CheckRobotStateKey, LoggingBehavior
from smart_home_pytree.robot_interface import RobotInterface

import time
# Root (Sequence)
#  ├─ Charging Selector
#  │   ├─ Check not charging
#  │   └─ Undock action
#  └─ MoveToHome action
 
## todo: setup move to work correctly with xyz
## todo register positions in blackboard
## how to pass time for move to header, shoud we pass node 

def create_move_to_tree(robot_interface, x: float = -0.56, y: float = 0.60,
                  quat: Quaternion = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)) -> py_trees.behaviour.Behaviour:
    """
    Create a tree to handle moving the robot 

    Returns:
        the root of the tree
    """
    
    root = py_trees.composites.Sequence(name="MoveTo", memory=True)
    
    state = robot_interface.state
    
    # Already charging check
    ## fallback
    ## if charging undock 
    undocking_selector = py_trees.composites.Selector(name="undocking_selector", memory=True)
    
    not_charging_status = CheckRobotStateKey(
        name="Check_Charging",
        state=state,
        key="charging",
        expected_value=False,
        comparison=operator.eq
    )
    
    undocking_goal = DockingRequest.Goal()
    undock_robot = py_trees_ros.actions.ActionClient(
        name="Undock_Robot",
        action_type=DockingRequest,
        action_name="undock",
        action_goal=undocking_goal,
        wait_for_server_timeout_sec=120.0
    )
    
    root.add_child(undocking_selector)
    undocking_selector.add_children([not_charging_status, undock_robot])
    
    # Move to position 
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.orientation = quat
    move_to_position = py_trees_ros.actions.ActionClient(
        name="Move_to_Pose",
        action_type=NavigateToPose,
        action_name="navigate_to_pose",
        action_goal=NavigateToPose.Goal(pose=pose),
        wait_for_server_timeout_sec=120.0
    )
    
    # move_to_position = py_trees.behaviours.Success(name="Move_to_Pose_Success")  # Placeholder for actual move action
    root.add_child(move_to_position)
    return root

## Runs until root gives success or failure
def create_move_to_tree_main():
    """
    Entry point for the script.
    """
    rclpy.init(args=None)
    
    robot_interface = RobotInterface()
    
    executor = rclpy.executors.MultiThreadedExecutor()  
    executor.add_node(robot_interface)
    
    root = create_move_to_tree(robot_interface=robot_interface)
    
    tree = py_trees_ros.trees.BehaviourTree(
        root=root,
        unicode_tree_debug=True
    )
    
    try:
        tree.setup(node_name="move_to_tree", timeout=15.0)
    except py_trees_ros.exceptions.TimedOutError as e:
        console.logerror(console.red + "failed to setup the tree, aborting [{}]".format(str(e)) + console.reset)
        tree.shutdown()
        robot_interface.destroy_node()
        executor.shutdown()
        rclpy.try_shutdown()
        sys.exit(1)
    except KeyboardInterrupt:
        # not a warning, nor error, usually a user-initiated shutdown
        console.logerror("tree setup interrupted")
        tree.shutdown()
        robot_interface.destroy_node()
        executor.shutdown()
        rclpy.try_shutdown()
        sys.exit(1)
    
    executor.add_node(tree.node)
    
    def tick_tree_until_done(timer: rclpy.timer.Timer):
        """
        Callback for the timer to tick the tree and check for termination.
        """
        # The key change is here: call tick_once() on the root node.
        tree.root.tick_once()
        
        # Print the current state of the tree after each tick
        print("="*25 + " TREE STATE " + "="*25)
        print(display.unicode_tree(root=tree.root, show_status=True))
        print("\n")
        
        # Check if the root of the tree has reached a terminal state
        if tree.root.status in [py_trees.common.Status.SUCCESS, py_trees.common.Status.FAILURE]:
            final_status = tree.root.status
            console.loginfo(
                console.green + 
                f"Tree has finished with status: {final_status}" + 
                console.reset
            )
            timer.cancel()  # Stop the timer
            # Trigger a clean shutdown of the ROS executor
            rclpy.shutdown()

    # Create a timer that calls the tick_tree function every 1 second (1000 ms)
    timer_period = 1.0  # seconds
    tree_timer = tree.node.create_timer(timer_period, lambda: tick_tree_until_done(tree_timer))
    
    try:
        executor.spin()
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        console.logwarn("Executor interrupted or externally shutdown.")
    finally:
        # Clean shutdown
        tree.shutdown()
        robot_interface.destroy_node()
        executor.shutdown()
        rclpy.try_shutdown()
        print("Shutdown complete.")

## Runs infinitely
def create_move_to_tree_main_continuous():
    """
    Entry point for the script.
    """
    rclpy.init(args=None)
    
    robot_interface = RobotInterface()
    
    executor = rclpy.executors.MultiThreadedExecutor()  
    executor.add_node(robot_interface)
    
    root = create_move_to_tree(robot_interface=robot_interface)
    
    tree = py_trees_ros.trees.BehaviourTree(
        root=root,
        unicode_tree_debug=True
    )
    
    try:
        tree.setup(node_name="move_to_tree", timeout=15.0)
    except py_trees_ros.exceptions.TimedOutError as e:
        console.logerror(console.red + "failed to setup the tree, aborting [{}]".format(str(e)) + console.reset)
        tree.shutdown()
        robot_interface.destroy_node()
        executor.shutdown()
        rclpy.try_shutdown()
        sys.exit(1)
    except KeyboardInterrupt:
        # not a warning, nor error, usually a user-initiated shutdown
        console.logerror("tree setup interrupted")
        tree.shutdown()
        robot_interface.destroy_node()
        executor.shutdown()
        rclpy.try_shutdown()
        sys.exit(1)
    
    executor.add_node(tree.node)
    print(type(tree))
    print(tree.tick_tock)

    tree.tick_tock(period_ms=1000.0)
    
    try:
        executor.spin()
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        console.logwarn("Executor interrupted or externally shutdown.")
    finally:
        # Clean shutdown
        tree.shutdown()
        robot_interface.destroy_node()
        executor.shutdown()
        rclpy.try_shutdown()
        print("Shutdown complete.")


if __name__ == '__main__':
    create_move_to_tree_main()