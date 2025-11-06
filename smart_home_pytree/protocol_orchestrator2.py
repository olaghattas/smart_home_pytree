import os
import yaml
import time
import random
import heapq
import threading
from datetime import datetime
import rclpy
from smart_home_pytree.trees.two_reminder_protocol import TwoReminderProtocolTree
from smart_home_pytree.trees.charge_robot_tree import ChargeRobotTree
from smart_home_pytree.robot_interface import get_robot_interface

#  TRIGGER MONITOR 
class TriggerMonitor:
    def __init__(self, robot_interface, yaml_path=None):
        
        self.robot_interface = robot_interface
        
        if yaml_path is None:
            yaml_path = os.getenv("house_yaml_path")
        # yaml_path = "/home/olagh48652/smart_home_pytree_ws/src/smart_home_pytree/config/house_info copy.yaml"
        
        with open(yaml_path, "r") as f:
            self.protocols_yaml = yaml.safe_load(f)

        self.current_satisfied_protocols = []  # [(protocol_name, priority)]
        self.lock = threading.Lock()
        self.stop_flag = False
        self.completed_protocols = set()  # track completed ones
        
         # Dynamically collect event keys from YAML
        self.event_keys = self._extract_event_keys()
        print(f"[TriggerMonitor] Loaded event keys from YAML: {self.event_keys}")

    def _extract_event_keys(self):
        """Collect all event topic names used in the YAML protocols."""
        event_keys = set()
        for _, protocol_data in self.protocols_yaml.get("protocols", {}).items():
            for _, sub_data in protocol_data.items():
                high_level_subdata = sub_data["high_level"]
                reqs = high_level_subdata.get("requirements", {})
                event_reqs = reqs.get("event", [])
                for ev in event_reqs:
                    if "topic" in ev:
                        event_keys.add(ev["topic"])
        return sorted(event_keys)
    
    def _collect_current_events(self):
        """Build a dict of current event states from RobotState."""
        current_events = {}
        for key in self.event_keys:
            key = key.lstrip("/") 
            val = self.robot_interface.state.get(key)
            if val is None:
                print(f"[TriggerMonitor] Topic '{key}' not publishing or None → treating as False")
                current_events[key] = False
            else:
                current_events[key] = val
        return current_events

    def check_time_requirement(self, time_req, current_time=None):
        if not time_req:
            return True
        if current_time is None:
            current_time = datetime.now().strftime("%H:%M")
            
        fmt = "%H:%M"
        t_from = datetime.strptime(time_req['from'], fmt)
        t_to = datetime.strptime(time_req['to'], fmt)
        t_now = datetime.strptime(current_time, fmt)
        
        # print("***** t_from: ", t_from)
        # print("***** t_to: ", t_to)
        # print("***** t_now: ", t_now)

        return t_from <= t_now <= t_to

    def check_event_requirement(self, event_reqs, current_events):
        if not event_reqs:
            return True
        for ev in event_reqs:
            topic, val = ev['topic'].lstrip("/"), ev['value']
            if topic not in current_events or current_events[topic] != val:
                return False
        return True

    def satisfied_protocols(self, current_events, current_time=None):
        satisfied = []
        for protocol_name, protocol_data in self.protocols_yaml['protocols'].items():
            print("protocol_name: ", protocol_name)
            print("protocol_data: ", protocol_data)
            
            for sub_name, sub_data in protocol_data.items():
                # print("sub_name: ", sub_name)
                high_level_subdata = sub_data["high_level"]
                full_name = f"{protocol_name}.{sub_name}"
                if full_name in self.completed_protocols:  # skip completed ones
                    continue
                
                reqs = high_level_subdata.get('requirements', {})
                event_reqs = reqs.get('event', [])
                time_req = reqs.get('time', {})
                
                # print("$$$$$ time_req: ", time_req)
                
                if self.check_event_requirement(event_reqs, current_events) and \
                   self.check_time_requirement(time_req, current_time):
                    priority = high_level_subdata.get('priority', 1)
                    satisfied.append((full_name, priority))
        satisfied.sort(key=lambda x: x[1])
        return satisfied

    def start_monitor(self, current_time: str = None):
        while not self.stop_flag:
         
            current_events = self._collect_current_events()
            new_satisfied = self.satisfied_protocols(current_events, current_time=current_time)
            
            print("**************** new_satisfied: ", new_satisfied)
            
            with self.lock:
                self.current_satisfied_protocols = new_satisfied

            time.sleep(2)  # small delay to avoid busy loop

    def mark_completed(self, protocol_name):
        """Mark a protocol as successfully completed to avoid retriggering."""
        with self.lock:
            print(f"[TriggerMonitor] Marking {protocol_name} as completed.")
            self.completed_protocols.add(protocol_name)
            
    def get_satisfied(self):
        with self.lock:
            return list(self.current_satisfied_protocols)

    def stop_monitor(self):
        self.stop_flag = True

import time

class DummyProtocolTree:
    def __init__(self, node_name, protocol_name=None):
        self.node_name = node_name
        self.protocol_name = protocol_name
        self._running = False
        self.final_status = "failure"
        
    def setup(self):
        print(f"[{self.node_name}] Setup complete for protocol {self.protocol_name}")

    def run_until_done(self):
        self._running = True
        for i in range(30, 0, -5):
            if not self._running:
                print(f"[{self.node_name}] Interrupted at {i}s left.")
                return
            print(f"[{self.node_name}] Running... {i}s remaining")
            # time.sleep(5)
        
        if self._running:
            self.final_status = "success"
            
        print(f"[{self.node_name}] Finished successfully.")
        self._running = False

    def stop_tree(self):
        print(f"[{self.node_name}] Stop signal received.")
        self._running = False

    def cleanup(self):
        print(f"[{self.node_name}] Cleanup done.")
        
# PROTOCOL ORCHESTRATOR 
class ProtocolOrchestrator:
    def __init__(self, test_time=None):
        rclpy.init()
        
        print("initialize robot interface")
        self.robot_interface=get_robot_interface()
        
        self.trigger_monitor = TriggerMonitor(self.robot_interface)
        # self.monitor_thread = threading.Thread(target=self.trigger_monitor.start_monitor, daemon=True)
        
        self.monitor_thread = threading.Thread(
            target=self.trigger_monitor.start_monitor,
            kwargs={"current_time": test_time},  # can be None or e.g. "10:30"
            daemon=True
        )
        
        self.monitor_thread.start()
    
        self.running_tree = None
        self.running_thread = None
        self.lock = threading.Lock()
        self.stop_flag = False


    def orchestrator_loop(self):
        """Main loop that manages protocol execution."""
        while not self.stop_flag:
            # --- cleanup if running thread finished ---
            if self.running_thread and not self.running_thread.is_alive():
                with self.lock:
                    print(f"[Orchestrator] Protocol {self.running_tree['name']} completed.")
                    self.running_tree = None
                    self.running_thread = None

            satisfied = self.trigger_monitor.get_satisfied()
            next_protocol = min(satisfied, key=lambda x: x[1], default=None)

            if not next_protocol:
                time.sleep(2)
                continue

            if not self.running_tree:
                self.start_protocol(next_protocol)
            else:
                if next_protocol[1] < self.running_tree["priority"]:
                    print(f"[Orchestrator] Higher-priority protocol detected: {next_protocol}")
                    self.stop_protocol()
                    self.start_protocol(next_protocol)

            time.sleep(2)

    def _run_protocol(self, tree_runner, protocol_name, priority):
        """Run a protocol tree and clean up when done."""
        try:
            tree_runner.run_until_done()
        finally:
            print("tree_runner.final_status", tree_runner.final_status)
            if (tree_runner.final_status == "success"):
                self.trigger_monitor.mark_completed(protocol_name)
                
            # tree_runner.final_status == py_trees.common.Status.SUCCESS

            with self.lock:
                print(f"[Orchestrator] Finished: {protocol_name}")
                self.running_tree = None
                self.running_thread = None
                
    def start_protocol(self, protocol_tuple):
        protocol_name, priority = protocol_tuple
        sub_name = protocol_name.split(".")[-1]
        print(f"Starting dummy protocol: {protocol_name} (priority {priority})")

        # Use dummy tree instead of real one
        tree_runner = DummyProtocolTree(node_name="dummy_tree", protocol_name=sub_name)
        tree_runner.setup()

        thread = threading.Thread(
            target=self._run_protocol,
            args=(tree_runner, protocol_name, priority),
            daemon=True
        )
        thread.start()

        self.running_tree = {"name": protocol_name, "priority": priority, "tree": tree_runner}
        self.running_thread = thread


    # def start_protocol(self, protocol_tuple):
    #     """Start the protocol in its own thread."""
    #     protocol_name, priority = protocol_tuple
    #     sub_name = protocol_name.split(".")[-1]
    #     print(f"[Orchestrator] Starting: {protocol_name} (priority {priority})")

    #     # Choose the correct tree type
    #     if "TwoReminderProtocol" in protocol_name:
    #         tree_runner = TwoReminderProtocolTree(
    #             node_name="two_reminder_protocol_tree", protocol_name=sub_name
    #         )
    #     elif "ChargeRobotTree" in protocol_name:
    #         tree_runner = ChargeRobotTree(node_name="charge_robot_tree")
    #     else:
    #         print(f"[Orchestrator] No matching tree for {protocol_name}")
    #         return

    #     tree_runner.setup()
    #     thread = threading.Thread(
    #         target=self._run_protocol,
    #         args=(tree_runner, protocol_name, priority),
    #         daemon=True,
    #     )
    #     thread.start()

    #     with self.lock:
    #         self.running_tree = {"name": protocol_name, "priority": priority, "tree": tree_runner}
    #         self.running_thread = thread

    def stop_protocol(self):
        """Stop the currently running protocol."""
        with self.lock:
            if not self.running_tree:
                return
            name = self.running_tree["name"]
            print(f"[Orchestrator] Stopping: {name}")
            tree = self.running_tree["tree"]

        tree.stop_tree()
        self.running_thread.join(timeout=5)
        tree.cleanup()

        with self.lock:
            self.running_tree = None
            self.running_thread = None

    def shutdown(self):
        """Gracefully stop everything."""
        print("[Orchestrator] Shutting down...")
        self.stop_flag = True
        self.trigger_monitor.stop_monitor()
        self.monitor_thread.join(timeout=10)

        if self.running_tree:
            self.stop_protocol()

        rclpy.shutdown()
        print("[Orchestrator] Shutdown complete.")
        
        
if __name__ == "__main__":
    import time

    # For testing:
    orch = ProtocolOrchestrator(test_time="10:30")

    # For live use:
    # orch = ProtocolOrchestrator()

    

    # # Mock data: pretend these are current satisfied protocols
    # orch.trigger_monitor.get_satisfied = lambda: [
    #     ("TwoReminderProtocol.medicine_am", 1)
    # ]

    try:
        orch.orchestrator_loop()
    except KeyboardInterrupt:
        print("Shutting down orchestrator...")
        orch.shutdown()
