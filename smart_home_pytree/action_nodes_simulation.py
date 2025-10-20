#!/usr/bin/env python3

"""
Launch and cleanly stop the TurtleBot3 simulation and action servers.
"""

import os
import signal
import subprocess
import psutil


def launch_action_servers_subprocess(action_server_dict):
    """
    Launch ROS2 action nodes as independent process groups.
    Each entry in action_server_dict is {package_name: [node1, node2, ...]}.
    """
    processes = []

    for package_name, nodes in action_server_dict.items():
        for node_name in nodes:
            cmd = ["ros2", "run", package_name, node_name]

            log_path = f"/tmp/{node_name}_log.txt"
            log_file = open(log_path, "w")

            # Start process in its own group
            proc = subprocess.Popen(
                cmd,
                stdout=log_file,
                stderr=subprocess.STDOUT,
                preexec_fn=os.setsid,   # <-- ensures group control
            )

            processes.append((proc, log_file))
            print(f"Launched {node_name} (PID {proc.pid}). Logs: {log_path}")

    return processes


def stop_action_servers(processes, timeout=3):
    """Terminate all launched action servers and their children."""
    for proc, log_file in processes:
        try:
            if proc.poll() is None:
                pgid = os.getpgid(proc.pid)
                print(f"Terminating action server group (PGID {pgid})...")

                os.killpg(pgid, signal.SIGTERM)
                try:
                    proc.wait(timeout=timeout)
                except subprocess.TimeoutExpired:
                    print(f"Action server group {pgid} did not exit in {timeout}s; forcing kill.")
                    os.killpg(pgid, signal.SIGKILL)
                    proc.wait(timeout=1)

                print(f"Terminated {proc.pid}")
            else:
                print(f"Action server {proc.pid} already exited.")
        except ProcessLookupError:
            print(f"Process {proc.pid} already gone.")
        except Exception as e:
            print(f"Error stopping process {proc.pid}: {e}")
        finally:
            try:
                log_file.close()
            except Exception:
                pass


def launch_tb3_simulation_subprocess():
    """
    Launch the TurtleBot3 Gazebo simulation with its own process group.
    """
    cmd = [
        "ros2", "launch", "nav2_bringup", "tb3_simulation_launch.py",
        "headless:=False"
    ]

    log_path = "/tmp/tb3_simulation.log"
    log_file = open(log_path, "w")

    process = subprocess.Popen(
        cmd,
        stdout=log_file,
        stderr=subprocess.STDOUT,
        preexec_fn=os.setsid,  # separate session for clean shutdown
    )
    print(f"TurtleBot3 simulation started (PID {process.pid}). Logs: {log_path}")
    return process


def stop_tb3_simulation(process, timeout=5):
    """
    Cleanly terminate the TB3 simulator and all child processes (Gazebo, RViz, etc.).
    """
    try:
        if process.poll() is None:
            pgid = os.getpgid(process.pid)
            print(f"Terminating TB3 simulator group (PGID {pgid})...")

            os.killpg(pgid, signal.SIGTERM)
            try:
                process.wait(timeout=timeout)
            except subprocess.TimeoutExpired:
                print(f"Simulator group {pgid} did not exit in {timeout}s; forcing kill.")
                os.killpg(pgid, signal.SIGKILL)
                process.wait(timeout=1)

            print("TB3 simulator terminated.")
        else:
            print("TB3 simulator already exited.")
    except ProcessLookupError:
        print("Simulator process already gone.")
    except Exception as e:
        print(f"Error stopping TB3 simulator: {e}")
