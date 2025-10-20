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
from smart_home_pytree.trees.base_tree_runner import BaseTreeRunner
import argparse


# Root (Sequence)
#  ├─ Charging Selector
#  │   ├─ Check not charging
#  │   └─ Undock action
#  └─ MoveToHome action
 
## todo register positions in blackboard
class MoveToLocationTree(BaseTreeRunner):      
    def __init__(self, node_name: str, run_actions: bool = False, run_simulator: bool = False, **kwargs):
        """
        Initialize the MoveToLocationTree.

        Args:
            node_name (str): name of the ROS node.
            run_actions (bool): whether to check for and require actions.
            run_simulator (bool): whether to enable simulation mode.
            **kwargs: extra arguments such as x, y, quat.
        """
        super().__init__(
            node_name=node_name,
            run_actions=run_actions,
            run_simulator=run_simulator,
            **kwargs
        )

    
    def create_tree(self, robot_interface) -> py_trees.behaviour.Behaviour:
        """
        Create a tree to handle moving the robot 

        Returns:
            the root of the tree
        """
        
        x = self.kwargs.get("x", -0.56)
        y = self.kwargs.get("y", 0.60)
        quat = self.kwargs.get("quat", Quaternion(x=0.0, y=0.0, z=0.0, w=1.0))

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
        
        # if not undock_robot.client.wait_for_server(timeout_sec=10.0):
        #     console.logerror("Undock action server not available.")

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
    
    def required_actions(self):
        return {
            "smart_home_pytree": ["docking", "undocking"]
        }

    def required_topics(self):
        return [
            "/charging"
        ]
        
        

        
if __name__ == "__main__":    
    parser = argparse.ArgumentParser(
        description="Run move_to_location tree for robot navigation",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Run until done with custom coordinates
  python3 move_to_tree.py --run_actions --x -0.56 --y 0.60 --qx 0.0 --qy 0.0 --qz 0.0 --qw 1.0

  # Run continuously with default coordinates
  python3 move_to_tree.py --run_actions --run_continuous 

  # Run with simulator (continuous mode)
  python3 move_to_tree.py --run_simulator --run_actions --run_continuous 

  # Show this help message
  python3 move_to_tree.py --help
        """
    )
    
    # Execution mode arguments
    parser.add_argument(
        "--run_simulator",
        action="store_true",
        default=False,
        help="Launch TB3 simulator with Nav2 (default: False)"
    )
    parser.add_argument(
        "--run_actions",
        action="store_true", 
        default=False,
        help="Launch action nodes (default: False)"
    )
    parser.add_argument(
        "--run_continuous",
        action="store_true",
        default=False,
        help="Run tree continuously (default: True)"
    )
    
    # Position arguments
    parser.add_argument(
        "--x",
        type=float,
        default=0.0,
        help="X coordinate to move to (default: 0.0)"
    )
    parser.add_argument(
        "--y",
        type=float,
        default=0.0,  # Fixed: was True, should be float
        help="Y coordinate to move to (default: 0.0)"
    )
    
    # Quaternion arguments (individual components)
    parser.add_argument(
        "--qx",
        type=float,
        default=0.0,
        help="X component of quaternion orientation (default: 0.0)"
    )
    parser.add_argument(
        "--qy", 
        type=float,
        default=0.0,
        help="Y component of quaternion orientation (default: 0.0)"
    )
    parser.add_argument(
        "--qz",
        type=float, 
        default=0.0,
        help="Z component of quaternion orientation (default: 0.0)"
    )
    parser.add_argument(
        "--qw",
        type=float,
        default=1.0,
        help="W component of quaternion orientation (default: 1.0)"
    )
    
    args = parser.parse_args()
    
    # Create Quaternion object from individual components
    target_quat = Quaternion(x=args.qx, y=args.qy, z=args.qz, w=args.qw)
    
    # Initialize tree runner
    tree_runner = MoveToLocationTree(
        node_name="move_to_location_tree", 
        run_simulator=args.run_simulator,
        run_actions=args.run_actions, 
        x=args.x, 
        y=args.y,
        quat=target_quat
    )
    
    tree_runner.setup()
    
    # Run based on mode
    if args.run_continuous:
        print("Running tree until completion...")
        tree_runner.run_continuous()
        
    else:
        print("Running tree continuously...")
        tree_runner.run_until_done()
        
  