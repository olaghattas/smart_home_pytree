#!/usr/bin/env python3

'''

 This tree should include all the topics that the robot should subscribe to and it should be 
 used at the highest tree level needed.
 IT SHOULD INCLUDe ALL THE TOPICS NEEDED BY THE TREE AND SUBTREES

'''

import py_trees
import py_trees_ros
from std_msgs.msg import Bool

def create_subscription_tree() -> py_trees.behaviour.Behaviour:
    root = py_trees.composites.Sequence(name="Topics2BB", memory=True)

    charging2bb = py_trees_ros.subscribers.ToBlackboard(
            name="Charging2BB",
            topic_name="/charging",
            topic_type=Bool,
            blackboard_variables={"charging": "data"},
            qos_profile=py_trees_ros.utilities.qos_profile_unlatched()
        )
    
    root.add_child(charging2bb)
    
    return root