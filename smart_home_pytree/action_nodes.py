#!/usr/bin/env python3

"""
This script is repsponsible for creating the robot tree to move the robot to a location.

The tree receives an (x,y) position and a quaternion representing the target pose for the robot. Before initiating movement, it checks whether the robot is currently charging and, if so, commands it to undock first.
"""


import rclpy
from smart_home_pytree.robot_interface import RobotInterface
## to launch action server from scripts
import launch
import launch_ros.actions
import argparse
import subprocess

def launch_tb3_simulation_subprocess():
    cmd = [
        "ros2", "launch", "nav2_bringup", "tb3_simulation_launch.py",
        "headless:=False"
    ]
    
    # Open a file in /tmp to write logs
    log_file = open("/tmp/tb3_simulation.log", "w")
    
    # Launch the subprocess with stdout and stderr redirected
    process = subprocess.Popen(cmd, stdout=log_file, stderr=subprocess.STDOUT)
    print("tb3_simulation_launch.py started. Logs are written to /tmp/tb3_simulation.log")
    
    return process

def launch_required_action_nodes(action_server_dict: dict):
    """
    Launch action servers from a dict: {package_name: [node1, node2, ...]}
    Returns the LaunchService instance and thread.
    """
    nodes = []
    for package_name, node_list in action_server_dict.items():
        for node_name in node_list:
            nodes.append(
                launch_ros.actions.Node(
                    package=package_name,
                    executable=node_name,
                    name=node_name,
                    output='screen',
                    emulate_tty=True
                )
            )
    
    ld = launch.LaunchDescription(nodes)
    launch_service = launch.LaunchService()
    launch_service.include_launch_description(ld)

    print("Action servers launched:", nodes)
    return launch_service


## Runs until root gives success or failure
def launch_action_main(simulator: bool, action_server_dict: dict):
    """
    Entry point for the script.
    Runs both LaunchServices in the main thread, while spinning the executor.
    """

    # Launch services
    run_action_servers = launch_required_action_nodes(action_server_dict)
    
    tb3_process = None
    if simulator:
        print("Starting TB3 simulator...")
        tb3_process = launch_tb3_simulation_subprocess()
    else:
        print("Simulator not launched. Make sure Nav2 is running to provide /navigate_to_pose action.")


    # Run both launch services in the main thread
    def run_launch_services():
        # These calls block, but are run sequentially in the main thread
        print("Starting mock action servers...")
        run_action_servers.run()
        print("Mock action servers stopped.")
        
    try:
        # Start launch services in the main thread (this blocks)
        run_launch_services()
    except KeyboardInterrupt:
        print("Launch interrupted by user.")
    finally:        
        # Stop TB3 simulator if it was launched
        if tb3_process:
            tb3_process.terminate()
            tb3_process.wait()
            print("TB3 simulator terminated.")
            
        print("Shutdown complete.")

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description="Launch smart_home_pytree robot tree.")
    parser.add_argument(
        "--simulator",
        type=bool,
        default=True,
        help="Launch TB3 simulator with Nav2 (default: True)"
    )
    args = parser.parse_args()
    
    action_server_dict = {
    "smart_home_pytree": ["docking", "undocking"]
    }
    
    launch_action_main(args.simulator, action_server_dict)
    # launch_action_main()
    
# python3 action_nodes.py --simulator True    # Launch TB3
# python3 action_nodes.py --simulator False 