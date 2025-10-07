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
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool
import operator
from shr_msgs.action import DockingRequest

# Root (Sequence)
#  ├─ Charging Selector
#  │   ├─ Check not charging
#  │   └─ Undock action
#  └─ MoveToHome action
 
## todo: check if every tree needs to have the subscribers or one subscriber for all.
## todo: setup move to work correctly
## how to pass time for move to header, shoud we pass node 

def create_move_to_tree(x=0,y=0,quat=0) -> py_trees.behaviour.Behaviour:
    """
    Create a basic tree and start 

    Returns:
        the root of the tree
    """
    
    root = py_trees.composites.Sequence(name="MoveTo", memory=False)
    
    # Already charging check
    ## fallback
    ## if charging undock 
    undocking_selector = py_trees.composites.Selector(name="undocking_selector", memory=False)
    not_charging_status = py_trees.behaviours.CheckBlackboardVariableValue(
        name="Charging_Status",
        check=py_trees.common.ComparisonExpression(
            variable="charging",
            value=False,
            operator=operator.eq
        )
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
    
    # Move to home action
    # pose_goal = PoseStamped()
    # pose_goal.header.frame_id = 'map'
    # pose_goal.pose.position.x = x
    # pose_goal.pose.position.y = y
    # pose_goal.pose.orientation = quat

    # move_to_position = py_trees_ros.actions.ActionClient(
    #     name="Move_to_Pose",
    #     action_type=NavigateToPose,
    #     action_name="navigate_to_pose",
    #     action_goal=NavigateToPose.Goal(pose=pose_goal),
    #     wait_for_server_timeout_sec=120.0
    # )
    
    move_to_position = py_trees.behaviours.Success(name="Move_to_Pose_Success")  # Placeholder for actual move action
    root.add_child(move_to_position)
    return root

'''

Same as the one above but includes theh subscription tree so i can run alone

'''

from .subscription_tree import create_subscription_tree

def create_standalone_move_to_tree(x=0,y=0,quat=0) -> py_trees.behaviour.Behaviour:
    """
    Create a basic tree and start a 'Topics2BB' work sequence that
    will become responsible for data gathering behaviours.

    Returns:
        the root of the tree
    """
    
    root = py_trees.composites.Sequence(name="MoveTo", memory=False)
    
    topics2bb = create_subscription_tree()
    root.add_child(topics2bb)
    
    # Already charging check
    ## fallback
    ## if charging undock 
    undocking_selector = py_trees.composites.Selector(name="undocking_selector", memory=False)
    not_charging_status = py_trees.behaviours.CheckBlackboardVariableValue(
        name="Charging_Status",
        check=py_trees.common.ComparisonExpression(
            variable="charging",
            value=False,
            operator=operator.eq
        )
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
    
    # Move to home action
    # pose_goal = PoseStamped()
    # pose_goal.header.frame_id = 'map'
    # pose_goal.pose.position.x = x
    # pose_goal.pose.position.y = y
    # pose_goal.pose.orientation = quat

    # move_to_position = py_trees_ros.actions.ActionClient(
    #     name="Move_to_Pose",
    #     action_type=NavigateToPose,
    #     action_name="navigate_to_pose",
    #     action_goal=NavigateToPose.Goal(pose=pose_goal),
    #     wait_for_server_timeout_sec=120.0
    # )
    
    move_to_position = py_trees.behaviours.Success(name="Move_to_Pose_Success")  # Placeholder for actual move action
    root.add_child(move_to_position)
    return root


def create_move_to_tree_main():
    """
    Entry point for the script.
    """
    rclpy.init(args=None)
    root = create_standalone_move_to_tree()
    
    tree = py_trees_ros.trees.BehaviourTree(
        root=root,
        unicode_tree_debug=True
    )
    try:
        tree.setup(node_name="move_to_tree", timeout=15.0)
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