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
from .subscription_tree import create_subscription_tree
from .logging_behavior import LoggingBehavior

## A failure case but should be solved when using robot interface
# : 0 failure from 3]
#             {-} Charge Sequence [*]
#                 [-] MoveTo [*]
#                     [o] undocking_selector [*]
#                         --> Charging_Status [✕] -- key 'charging' does not yet exist on the blackboard
#                         --> Undock_Robot [*] -- sent goal request

## todo: check if every tree needs to have the subscribers or one subscriber for all.
def create_charge_robot_tree(num_attempts: int = 3) -> py_trees.behaviour.Behaviour:
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
    topics2bb = create_subscription_tree()
    
    root.add_child(topics2bb)
    
    # --- Task Selector Equivalent to Fallback---
    charge_robot = py_trees.composites.Selector(name="Tasks", memory=True)
    root.add_child(charge_robot)

    # Already charging check
    charging_status = py_trees.behaviours.CheckBlackboardVariableValue(
        name="Charging_Status",
        check=py_trees.common.ComparisonExpression(
            variable="charging",
            value=True,
            operator=operator.eq
        )
    )
    
    charging_status_2 = py_trees.behaviours.CheckBlackboardVariableValue(
        name="Charging_Status",
        check=py_trees.common.ComparisonExpression(
            variable="charging",
            value=True,
            operator=operator.eq
        )
    )

    ## takes position as input x, y , quat default 0 0 0 for now
    move_to_home = create_move_to_tree()
    # move_to_home = py_trees.behaviours.Success(name="Move_to_Pose")  # Placeholder for actual move action


    # Dock robot action (empty goal)
    docking_goal = DockingRequest.Goal()
    dock_robot = py_trees_ros.actions.ActionClient(
        name="Dock_Robot",
        action_type=DockingRequest,
        action_name="dock",
        action_goal=docking_goal,
        wait_for_server_timeout_sec=120.0
    )
    
    # dock_robot = py_trees.behaviours.Success(name="Dock_Robot")  # Placeholder for actual move action

    
    ## has to be a behavior
    # # Logging behaviors
    # Logging behaviors
    log_message_success = LoggingBehavior(
        name="Log_Success",
        message="Charging sequence completed successfully"
    )

    log_message_fail = LoggingBehavior(
        name="Log_Fail",
        message="Failed to charge after retry attempts"
    )

    # Charge sequence
    charge_sequence = py_trees.composites.Sequence(name="Charge Sequence", memory=True)
    charge_sequence.add_children([move_to_home, dock_robot, charging_status, log_message_success])

    # Retry decorator around charge sequence
    charge_sequence_with_retry = py_trees.decorators.Retry(
        name="Charge Sequence with Retry",
        child=charge_sequence,
        num_failures=num_attempts
    )

    # Final selector order: if already charging -> success, else run retry sequence, else log failure
    charge_robot.add_children([charging_status_2, charge_sequence_with_retry, log_message_fail])

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