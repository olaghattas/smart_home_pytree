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

from py_trees import display

from smart_home_pytree.util_behaviors import CheckRobotStateKey, LoggingBehavior
from smart_home_pytree.robot_interface import RobotInterface
from smart_home_pytree.trees.base_tree_runner import BaseTreeRunner
from smart_home_pytree.trees.move_to_tree import MoveToLocationTree

import argparse

def required_actions_():
        return {
            "smart_home_pytree": ["docking", "undocking"]
        }
        

class ChargeRobotTree(BaseTreeRunner):      
    def __init__(self, node_name: str, **kwargs):
        """
        Initialize the ChargeRobotTree.

        Args:
            node_name (str): name of the ROS node.
            **kwargs: extra arguments such as location.
        """
        super().__init__(
            node_name=node_name,
            **kwargs
        )
    
    def create_tree(self, robot_interface) -> py_trees.behaviour.Behaviour:
        """
        Create a tree to handle charging the robot 

        Returns:
            the root of the tree
        """
        
        # Get the blackboard
        blackboard = py_trees.blackboard.Blackboard()

        target_location = "home"
        num_attempts = self.kwargs.get("num_attempts", 3)
        print("num_attempts", num_attempts)

        # --- Task Selector Equivalent to Fallback---
        charge_robot = py_trees.composites.Selector(name="Tasks", memory=True)
        
        state = robot_interface.state

        # Behavior to check charging
        charging_status = CheckRobotStateKey(
            name="Check_Charging",
            state=state,
            key="charging",
            expected_value=True,
            comparison=operator.eq
        )

        # In py_trees, a single behavior instance cannot be added to the tree in more than one place — it must be cloned or instantiated again, because each node maintains its own state
        # Same as above
        check_charging_charge_seq = CheckRobotStateKey(
            name="Check_Charging_ChargeSeq",
            state=state,
            key="charging",
            expected_value=True,
            comparison=operator.eq
        )

        ## takes position as input x, y , quat default 0 0 0 for now
        move_to_home_tree = MoveToLocationTree(
            node_name="move_to_location_tree",
            location=target_location  # pass any location here
        )

        move_to_home = move_to_home_tree.create_tree(robot_interface)
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
        
        ## Logging behaviors has to be a behavior
        # 
        log_message_success = LoggingBehavior(
            name="Log_Success",
            message="Charging sequence completed successfully",
            status=py_trees.common.Status.SUCCESS
        )
        ## its actually the default so no need to use status

        log_message_fail = LoggingBehavior(
            name="Log_Fail",
            message="Failed to charge after retry attempts",
            status=py_trees.common.Status.FAILURE
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
        charge_robot.add_children([check_charging_charge_seq, charge_sequence_with_retry, log_message_fail])

        return charge_robot
    

    def required_actions(self):
        return required_actions_()

    def required_topics(self):
        return [
            "/charging"
        ]
        
    
def str2bool(v):
    return str(v).lower() in ('true', '1', 't', 'yes')

def main(args=None):    
    parser = argparse.ArgumentParser(
        description="""Robot Charging Behavior Tree 
        
        Handles automatic robot charging sequence:
        1. Checks if robot is already charging
        2. If not, moves to home position
        3. Attempts docking
        4. Retries up to num_attempts times if needed
                """,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )

    parser.add_argument('--run_continuous', type=str2bool, default=False, help="Run tree continuously (default: False)")
    parser.add_argument("--num_attempts", type=int, default=3, help="Docking retry attempts (default: 3)")


    args, unknown = parser.parse_known_args()

    tree_runner = ChargeRobotTree(
        node_name="charge_robot_tree",
    )
    tree_runner.setup()

    print("run_continuous", args.run_continuous)
    try:
        if args.run_continuous:
            tree_runner.run_continuous()
        else:
            tree_runner.run_until_done()
    finally:
        tree_runner.cleanup()

    rclpy.shutdown()


if __name__ == "__main__":
    main()