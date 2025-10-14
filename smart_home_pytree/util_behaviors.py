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
    def __init__(self, name: str, message: str):
        """
        A simple logging behavior that prints the current time and a custom message
        each time it's ticked.
        """
        super().__init__(name)
        self.message = message

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

        return py_trees.common.Status.SUCCESS

    def terminate(self, new_status):
        """
        Called whenever the behavior switches to a non-running state.
        """
        self.logger.debug(f"{self.name} [LoggingBehavior::terminate()][{self.status}→{new_status}]")