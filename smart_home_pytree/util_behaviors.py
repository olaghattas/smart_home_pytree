#!/usr/bin/env python3

'''
Script that includes general Behaviors 
'''
import py_trees
import operator

from datetime import datetime

class CheckRobotStateKey(py_trees.behaviour.Behaviour):
    """
    Checks a key in RobotState and returns SUCCESS/FAILURE
    based on a comparison operator and expected value.
    """
    def __init__(self, name: str, state, key: str, expected_value, comparison=operator.eq):
        super().__init__(name)
        self.state = state
        self.key = key
        self.expected_value = expected_value
        self.comparison = comparison

    def update(self):
        # Retrieve safely
        value = self.state.get(self.key, None)
        if value is None:
            self.logger.warning(f"{self.name}: '{self.key}' not found in RobotState")
            return py_trees.common.Status.FAILURE
        
        # Compare and return
        if self.comparison(value, self.expected_value):
            self.logger.debug(f"{self.name}: {self.key} == {self.expected_value} → SUCCESS")
            return py_trees.common.Status.SUCCESS
        else:
            self.logger.debug(f"{self.name}: {self.key} != {self.expected_value} → FAILURE")
            return py_trees.common.Status.FAILURE

    def __help__(self):
        info = self.describe_requirements()
        print(f"\n[Help: {info['behavior']}]")
        print(f"  • Requires key: '{info['required_key']}'")
        print(f"  • Expected value: {info['expected_value']}")
        print(f"  • Comparison: {info['comparison']}")
        print(f"  • From state: {info['state_source']}\n")
        
class LoggingBehavior(py_trees.behaviour.Behaviour):
    def __init__(self, name: str, message: str, status: py_trees.common.Status =py_trees.common.Status.SUCCESS):
        """
        A simple logging behavior that prints the current time and a custom message
        each time it's ticked.
        """
        super().__init__(name)
        self.message = message
        self.status = status

    def setup(self, **kwargs):
        """
        No delayed setup needed for this simple logger.
        """
        self.logger.debug(f"{self.name} [LoggingBehavior::setup()]")

    def initialise(self):
        """
        Called the first time the behavior is ticked or when leaving a non-RUNNING state.
        """
        self.logger.debug(f"{self.name} [LoggingBehavior::initialise()]")

    def update(self):
        """
        Logs the current time and the provided message, then returns SUCCESS.
        """
        current_time = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        full_message = f"[{current_time}] {self.message}"
        self.feedback_message = full_message
        self.logger.info(full_message)

        # Alternatively, you can also print if you want console output:
        print(full_message)

        return self.status

    def terminate(self, new_status):
        """
        Called whenever the behavior switches to a non-running state.
        """
        self.logger.debug(f"{self.name} [LoggingBehavior::terminate()][{self.status}→{new_status}]")
         
class GetPersonLocation(py_trees.behaviour.Behaviour):
    """
    Node that retrieves the current person's location
    (e.g., from a topic or blackboard variable).
    """
    def __init__(self, state, name="GetPersonLocation"):
        super().__init__(name)
        self.state = state
        self.blackboard = py_trees.blackboard.Blackboard()
        self.locations = self.blackboard.get("locations")
        print("locations", self.locations)
        
    def update(self):
        value = self.state.get("person_location", None)
        print("value: ", value)
        print("type value: ", type(value))
        
        ## check if location is valid
        if value is None or value not in self.locations:
            print("location not in available locations")
            return py_trees.common.Status.FAILURE
        
        ## update blackboard
        print(" set in blackboard")
        self.blackboard.set("person_location", value)
        return py_trees.common.Status.SUCCESS
    
class RobotPersonSameLocation(py_trees.behaviour.Behaviour):
    """
    Node that checks if robot and person are in the same location
    """
    def __init__(self, state, name="RobotPersonSameLocation"):
        super().__init__(name)
        self.state = state
        
    def update(self):
        person_location = self.state.get("person_location", None)
        robot_location = self.state.get("robot_location", None)
        
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