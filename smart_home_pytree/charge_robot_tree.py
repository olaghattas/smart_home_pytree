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
from smart_home_pytree.move_to_tree import create_move_to_tree

from py_trees import display

from smart_home_pytree.util_behaviors import CheckRobotStateKey, LoggingBehavior
from smart_home_pytree.robot_interface import RobotInterface


def create_charge_robot_tree(robot_interface, num_attempts: int = 3) -> py_trees.behaviour.Behaviour:
    """
    Create a basic tree and start a 'Topics2BB' work sequence that
    will become responsible for data gathering behaviours.

    Returns:
        the root of the tree
    """
    
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
    move_to_home = create_move_to_tree(robot_interface)
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
    charge_robot.add_children([check_charging_charge_seq, charge_sequence_with_retry, log_message_fail])

    return charge_robot

def create_charge_robot_tree_main():
    """
    Entry point for the script.
    """
    rclpy.init(args=None)
    
    robot_interface = RobotInterface()
    
    executor = rclpy.executors.MultiThreadedExecutor()  
    executor.add_node(robot_interface)
    
    root = create_charge_robot_tree(robot_interface=robot_interface)
    
    tree = py_trees_ros.trees.BehaviourTree(
        root=root,
        unicode_tree_debug=True
    )
    
    try:
        tree.setup(node_name="charge_robot_tree", timeout=15.0)
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


def create_charge_robot_tree_main_continuous():
    """
    Entry point for the script.
    """
    rclpy.init(args=None)
    
    robot_interface = RobotInterface()
    
    executor = rclpy.executors.MultiThreadedExecutor()  
    executor.add_node(robot_interface)
    
    root = create_charge_robot_tree(robot_interface=robot_interface)
    
    tree = py_trees_ros.trees.BehaviourTree(
        root=root,
        unicode_tree_debug=True
    )
    
    try:
        tree.setup(node_name="charge_robot_tree", timeout=15.0)
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
    create_charge_robot_tree_main()