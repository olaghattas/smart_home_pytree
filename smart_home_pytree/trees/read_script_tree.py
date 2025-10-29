import py_trees
from shr_msgs.action import DockingRequest
import py_trees.console as console
import rclpy
import py_trees_ros
import operator

from smart_home_pytree.behaviors.check_robot_state_key import CheckRobotStateKey
from smart_home_pytree.behaviors.logging_behavior import LoggingBehavior
from smart_home_pytree.behaviors.check_robot_state_key import CheckRobotStateKey
from smart_home_pytree.robot_interface import RobotInterface
from smart_home_pytree.trees.base_tree_runner import BaseTreeRunner
from smart_home_pytree.trees.move_to_tree import MoveToLocationTree
import yaml
import argparse
from smart_home_pytree.behaviors.action_behaviors import play_audio, read_script, wait
from smart_home_pytree.trees.move_to_person_location import MoveToPersonLocationTree
from smart_home_pytree.trees.charge_robot_tree import ChargeRobotTree
from smart_home_pytree.behaviors.set_protocol_bb import SetProtocolBB
         
#!/usr/bin/env python3

"""

This script is responsible for reading a script to the person at their location and then charging the robot.

"""

class ReadScriptTree(BaseTreeRunner):      
    def __init__(self, node_name: str, robot_interface=None, **kwargs):
        """
        Initialize the TwoReminderProtocol.

        Args:
            node_name (str): name of the ROS node.
            **kwargs: extra arguments such as location.
        """
        super().__init__(
            node_name=node_name,
            robot_interface=robot_interface,
            **kwargs
        )
    
    def create_tree(self, protocol_name: str, text_number: str, wait_time: float = 0.0) -> py_trees.behaviour.Behaviour:
        """
        Creates the TwoReminderProtocol tree:
        Sequence:
            MoveToPersonLocation -> ReadScript -> ChargeRobot -> Wait (optional)
        
        Args:
            protocol_name (str): which protocol does this read script belong to (same name as in yaml)
            text_number: which text to read (ex:first_text or second_text) has to be same as in yaml and part of the protoocl
            wait_time: how long should the robot wait after charging (default: 0.0s)
            ex yaml:
            
            medicine_am:
                first_text: "please take your morning medicine"
                second_text: "This is the second reminder to take your morning medicine"
            medicine_pm:
                first_text: "please take your night medicine"
                second_text: "This is the second reminder to take your night medicine"

        Returns:
            the root of the tree
        """
        blackboard = py_trees.blackboard.Blackboard()
        
        protocol_info = blackboard.get(protocol_name)
        text = protocol_info[text_number]
        
        move_to_person_tree = MoveToPersonLocationTree(node_name=f"{protocol_name}_move_to_person", robot_interface=self.robot_interface)
        move_to_person = move_to_person_tree.create_tree()
        
        charge_robot_tree = ChargeRobotTree(node_name=f"{protocol_name}_charge_robot", robot_interface=self.robot_interface)
        charge_robot = charge_robot_tree.create_tree()

        # Custom behaviors
        read_script_reminder = read_script.ReadScript(name=f"{protocol_name}_read_script", text=text)
        
        ## Set blackboard to indicate reading script is done
        # variable_name: name of the variable to set, may be nested, e.g. battery.percentage
        # variable_value: value of the variable to set
        # overwrite: when False, do not set the variable if it already exists
        # name: name of the behaviour
        set_read_script_success = SetProtocolBB(name = "read_script_set_bb", key=f"{protocol_name}_done.{text_number}_done", value = True)
        
        wait_behavior = wait.Wait(name="wait", duration_in_sec=wait_time)

        # Root sequence
        root_sequence = py_trees.composites.Sequence(name=f"{protocol_name}_read_script", memory=True)

        if wait_time > 0.0:
            # Add behaviors in order
            root_sequence.add_children([
            move_to_person,
            read_script_reminder,
            set_read_script_success,
            charge_robot,
            wait_behavior,
        ])
            
        else:
            root_sequence.add_children([
            move_to_person,
            read_script_reminder,
            set_read_script_success,
            charge_robot,
        ])
        

        return root_sequence
    
    
def str2bool(v):
    return str(v).lower() in ('true', '1', 't', 'yes')

def main(args=None):    
    parser = argparse.ArgumentParser(
        description="""Two Reminder Protocol Tree 
        
        Handles Playing the Two Reminder Protocol:
        1. Retries up to num_attempts times if needed
        """,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )

    parser.add_argument('--run_continuous', type=str2bool, default=False, help="Run tree continuously (default: False)")
    parser.add_argument("--num_attempts", type=int, default=3, help="retry attempts (default: 3)")
    parser.add_argument("--protocol_name", type=str, default="", help="name of the protocol that needs to run (ex: medicine_am)")


    args, unknown = parser.parse_known_args()
    protocol_name = args.protocol_name
    print("protocol_name: ", protocol_name)
    
    yaml_path = "/home/olagh48652/smart_home_pytree_ws/src/smart_home_pytree/config/house_info.yaml"
    
    load_protocol_info_from_bb(yaml_path, protocol_name)
    
    
    tree_runner = TwoReminderProtocolTree(
        node_name="two_reminder_protocol_tree",
        protocol_name=protocol_name,
    )
    tree_runner.setup()
    
    print("run_continuous", args.run_continuous)
    try:
        if args.run_continuous:
            tree_runner.run_continuous()
        else:
            tree_runner.run_until_done()
    finally:
        remove_protocol_info_from_bb(yaml_path, protocol_name)
        tree_runner.cleanup()

    rclpy.shutdown()


if __name__ == "__main__":
    main()