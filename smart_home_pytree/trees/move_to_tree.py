#!/usr/bin/env python3

"""
This script is responsible for creating the robot tree to move the robot to a location.

The tree receives an (x,y) position and a quaternion representing the target pose for the robot. Before initiating movement, it checks whether the robot is currently charging and, if so, commands it to undock first.
"""

import py_trees
import py_trees_ros.trees

import py_trees.console as console
import rclpy
import sys
from geometry_msgs.msg import PoseStamped, Quaternion
from std_msgs.msg import Bool
import operator
from shr_msgs.action import DockingRequest

from py_trees import display

from smart_home_pytree.util_behaviors import CheckRobotStateKey, LoggingBehavior
from smart_home_pytree.robot_interface import RobotInterface
from smart_home_pytree.trees.base_tree_runner import BaseTreeRunner
from smart_home_pytree.behaviors.move_to_bh import MoveToLandmark
import argparse


def required_actions_():
        return {
            "smart_home_pytree": ["undocking"]
        }
        
# Root (Sequence)
#  ├─ Charging Selector
#  │   ├─ Check not charging
#  │   └─ Undock action
#  └─ MoveToHome action
 
## todo register positions in blackboard
class MoveToLocationTree(BaseTreeRunner):      
    def __init__(self, node_name: str, **kwargs):
        """
        Initialize the MoveToLocationTree.

        Args:
            node_name (str): name of the ROS node.
            **kwargs: extra arguments such as x, y, quat.
        """
        super().__init__(
            node_name=node_name,
            **kwargs
        )

    
    def create_tree(self, robot_interface) -> py_trees.behaviour.Behaviour:
        """
        Create a tree to handle moving the robot 

        Returns:
            the root of the tree
        """
        # # Get the blackboard
        # blackboard = py_trees.blackboard.Blackboard()
       
        location = self.kwargs.get("location", "")
        location_key = self.kwargs.get("location_key", "person_location")

        root = py_trees.composites.Sequence(name="MoveTo", memory=True)
        
        state = robot_interface.state
        
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
            
        move_to_position = MoveToLandmark(state, robot_interface, location=location, location_key=location_key)
        
        # move_to_position = py_trees.behaviours.Success(name="Move_to_Pose_Success")  # Placeholder for actual move action
        root.add_child(move_to_position)
        return root
    
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
        description="Run move_to_location tree for robot navigation",
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )

    parser.add_argument('--run_continuous', type=str2bool, default=False, help="Run tree continuously (default: False)")
    parser.add_argument("--location", type=str, default="", help="Target location name (from YAML)")
    parser.add_argument("--location_key", type=str, default="person_location", help="key to use with blackboard")

    args, unknown = parser.parse_known_args()

    tree_runner = MoveToLocationTree(
        node_name="move_to_location_tree",
        location=args.location,
        location_key=args.location_key,
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
    
    

  
# python3 move_to_tree.py --location kitchen --run_actions True --run_simulator False --run_continuous False
