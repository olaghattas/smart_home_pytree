    # <BehaviorTree ID="MoveToPersonLocation">
    #     <Sequence name="MoveToPersonSequence">
    #         <Fallback name="RetryMoveToPersonProcess">
    #             <RetryUntilSuccessful num_attempts="5">
    #                 <Sequence name="GotoPersonRoutine">
    #                     <GetPersonLocation output_person_location="{person_location}"/>
    #                     <!-- Pass the location into the MoveTo SubTree -->
    #                     <SubTree ID="MoveTo" target_location="{person_location}" />
    #                     <RobotPersonSameLocation/> <!-- Condition node -->
    #                 </Sequence>
    #             </RetryUntilSuccessful>
    #             <LogMessage message=" Failed to reach person after 5 attempts" status="0"/>
    #         </Fallback>
    #     </Sequence>
    # </BehaviorTree>

#!/usr/bin/env python3

"""
This script is repsponsible for getting the person location and moving the robot there.

The tree should move to the person if it reaches the location and the person has moved
"""


import py_trees
import rclpy

from py_trees import display
from smart_home_pytree.util_behaviors import RobotPersonSameLocation, GetPersonLocation
from smart_home_pytree.trees.base_tree_runner import BaseTreeRunner
from smart_home_pytree.trees.move_to_tree import MoveToLocationTree

import argparse

def required_actions_():
        return {
            "smart_home_pytree": ["docking", "undocking"]
        }
        
        
class MoveToPersonLocationTree(BaseTreeRunner):      
    def __init__(self, node_name: str, **kwargs):
        """
        Initialize the MoveToPersonLocation.

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
        Create the MoveToPersonLocationTree tree that cancels navigation if the person moves rooms.
        """
        
        self.blackboard = py_trees.blackboard.Blackboard()
        root = py_trees.composites.Sequence("MoveToPersonSequence", memory=True)
        state = robot_interface.state

        num_attempts = self.kwargs.get("num_attempts", 3)

        # Build the behavior tree
        
        ## Get person location
        get_person_room = GetPersonLocation(state)
        
        ## get_person_room would give failure if person_location is not set or not valid
        move_to_home_tree = MoveToLocationTree(
            node_name="move_to_location_tree",
            location_key = "person_location"
        )
        move_to_room = move_to_home_tree.create_tree(robot_interface)
        
        robot_same_room = RobotPersonSameLocation(state)
    
        go_to_person_sequence = py_trees.composites.Sequence("GotoPersonRoutine", memory=True)
        go_to_person_sequence.add_children([get_person_room, move_to_room, robot_same_room])
        
        go_to_person_sequence_with_retry = py_trees.decorators.Retry(
            name="Go to Person Sequence with Retry",
            child=go_to_person_sequence,
            num_failures=num_attempts
        )
        
        root.add_child(go_to_person_sequence_with_retry)
        
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
        description="""Move to Person Location Behavior Tree 
        
        Handles automatic robot charging sequence:
        1. Gets person location
        2. moves to postion
        3. checks if robot and person are in same location
        4. Retries up to num_attempts times if needed
                """,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )

    parser.add_argument('--run_continuous', type=str2bool, default=False, help="Run tree continuously (default: False)")
    parser.add_argument("--num_attempts", type=int, default=3, help="Docking retry attempts (default: 3)")


    args, unknown = parser.parse_known_args()

    tree_runner = MoveToPersonLocationTree(
        node_name="move_to_person_location_tree",
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