#!/usr/bin/env python3

'''
Check if robot and person are in the same location. gets the location from robot_interface which reads it from the topics
'''
import py_trees
import operator

from datetime import datetime

class RobotPersonSameLocation(py_trees.behaviour.Behaviour):
    """
    Node that checks if robot and person are in the same location
    """
    def __init__(self, robot_interface, name="RobotPersonSameLocation"):
        super().__init__(name)
        self.robot_interface = robot_interface
        
    def update(self):
        person_location = self.robot_interface.state.get("person_location", None)
        robot_location = self.robot_interface.state.get("robot_location", None)
        
        print(f"[DEBUG] [{self.name}] person_location: {person_location}")
        print(f"[DEBUG] [{self.name}] robot_location: {robot_location}")

        if person_location is None or robot_location is None:
            return py_trees.common.Status.FAILURE
        
        if person_location == robot_location:
            print(f"[INFO] [{self.name}] Person and robot are in the SAME location: {person_location}")
            return py_trees.common.Status.SUCCESS
        else: 
            print(f"[INFO] [{self.name}] Person and robot are in DIFFERENT locations -> "
                  f"Person: {person_location}, Robot: {robot_location}")
            return py_trees.common.Status.FAILURE