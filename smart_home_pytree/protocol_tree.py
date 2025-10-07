#!/usr/bin/env python3

"""
This script is repsponsible for creating the charge robot tree.

The tree should check if the is charging and exit. else it moves the robot to home position then dock the robot and checks if it succcessfully charged. It will try for num_attempts then log if it fails
"""

import py_trees
import py_trees_ros.trees
from shr_msgs.action import DockingRequest
import py_trees.console as console
import rclpy
import sys
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool
import operator
from .move_to_tree import create_move_to_tree

## todo: check if every tree needs to have the subscribers or one subscriber for all.
def protocol_tree() -> py_trees.behaviour.Behaviour:
    """
    Create a basic tree and start a 'Topics2BB' work sequence that
    will become responsible for data gathering behaviours.

    Returns:
        the root of the tree
    """
    
    root = py_trees.composites.Parallel(
        name="Charge Robot",
        policy=py_trees.common.ParallelPolicy.SuccessOnAll(
            synchronise=False
        )
    )
    
    # --- Subscriptions to Blackboard ---
    subscriber_tree = create_subscriber_tree()
    root.add_child(subscriber_tree)
  
    #  # Retry decorator around charge sequence
    # charge_sequence_with_retry = py_trees.decorators.Retry(
    #     name="Charge Sequence with Retry",
    #     child=charge_sequence,
    #     num_failures=num_attempts
    # )

    
    # Charge sequence
    protocol_sequence = py_trees.composites.Sequence(name="Charge Sequence", memory=True)

    protocol_sequence.add_children([move_to_home, dock_robot, charging_status])

    charge_robot= create_charge_robot_tree()
    move_to = create_move_to_tree(x,y,z)
    # Dock robot action (empty goal)
    wait = WaitRequest.Goal()
    dock_robot = py_trees_ros.actions.ActionClient(
        name="Dock_Robot",
        action_type=WaitRequest,
        action_name="wait",
        action_goal=wait,
        wait_for_server_timeout_sec=120.0
    )
    
    # charge_sequence.add_children([move_to_home, dock_robot, charging_status, log_message_success])
    protocol_sequence.add_children([move_to_home, charge_robot, wait])



    


    return root


def charge_robot_tree_main():
    """
    Entry point for the demo script.
    """
    rclpy.init(args=None)
    root = create_charge_robot_tree()
    tree = py_trees_ros.trees.BehaviourTree(
        root=root,
        unicode_tree_debug=True
    )
    try:
        tree.setup(node_name="charge_robot_tree", timeout=15.0)
    except py_trees_ros.exceptions.TimedOutError as e:
        console.logerror(console.red + "failed to setup the tree, aborting [{}]".format(str(e)) + console.reset)
        tree.shutdown()
        rclpy.try_shutdown()
        sys.exit(1)
    except KeyboardInterrupt:
        # not a warning, nor error, usually a user-initiated shutdown
        console.logerror("tree setup interrupted")
        tree.shutdown()
        rclpy.try_shutdown()
        sys.exit(1)

    tree.tick_tock(period_ms=1000.0)

    try:
        rclpy.spin(tree.node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        tree.shutdown()
        rclpy.try_shutdown()