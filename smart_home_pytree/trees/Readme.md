This folder contains all the trees that are avaibable

Every tree must have these two methods run_tree and run_tree_infinite function. The first one is responsible to run the tree until the root succeedes or fails then exit while the other would run indefinetly, keeps on looping until its stopped by the user. The tree needs to added to the registery in the __init__ file of the parent folder.


# Smart Home PyTree – Behavior Tree Framework

This directory contains the modular **behavior tree (BT)** framework used to control the robot in a smart home environment.  
Each tree defines a self-contained behavior (e.g., moving, charging, or interacting) and inherits from a shared base class that handles setup, execution, and ROS2 integration.

---

## Directory Structure

```
smart_home_pytree/
├── trees/
│   ├── base_tree_runner.py        # Base class for all behavior trees
│   ├── move_to_tree.py            # Example: Move robot to target location
│   ├── charge_robot_tree.py       # Example: Charge robot if battery low
│   └── Readme.md                  # (this file)
├── util_behaviors.py              # Common reusable behaviors (check states, log info)
├── robot_interface.py             # Wraps ROS2 topics, actions, and services
```

---

## Core Concept

All trees inherit from **`BaseTreeRunner`**, which provides:

| Component | Responsibility |
|------------|----------------|
| `setup()` | Initializes ROS2, executor, robot interface, and builds the tree |
| `run_until_done()` | Runs the tree until it reaches `SUCCESS` or `FAILURE` |
| `run_continuous()` | Keeps ticking the tree until manually stopped |
| `cleanup()` | Safely shuts down ROS2 and all nodes |
| `required_actions()` / `required_topics()` | List dependencies for the tree |
| `describe_requirements()` | Prints a summary of what is required to run the tree |

Each tree defines its structure by overriding `create_tree()`.

---

## Example: MoveToLocationTree

A sample subclass looks like this:

```python
from geometry_msgs.msg import Quaternion
from smart_home_pytree.trees.base_tree_runner import BaseTreeRunner
from smart_home_pytree.util_behaviors import CheckRobotStateKey, LoggingBehavior
import py_trees

class MoveToLocationTree(BaseTreeRunner):
    def __init__(self, node_name: str, run_actions: bool = False, run_simulator: bool = False, **kwargs):
        super().__init__(node_name=node_name, run_actions=run_actions, run_simulator=run_simulator, **kwargs)

    def create_tree(self, robot_interface):
        x = self.kwargs.get("x", -0.56)
        y = self.kwargs.get("y", 0.60)
        quat = self.kwargs.get("quat", Quaternion(x=0.0, y=0.0, z=0.0, w=1.0))

        root = py_trees.composites.Sequence(name="MoveToSequence")
        root.add_children([
            LoggingBehavior("Starting MoveTo Tree"),
            CheckRobotStateKey("CheckChargingState", key="charging")
        ])
        return root

    def required_actions(self):
        return {"smart_home_pytree": ["docking", "undocking"]}

    def required_topics(self):
        return ["/charging"]
```

---

## Running a Tree

You can run any tree directly as a standalone ROS2 node:

```bash
ros2 run smart_home_pytree move_to_tree
```

or run it as a Python script:

```bash
python3 move_to_tree.py
```

Example instantiation in code:
```python
tree = MoveToLocationTree(
    node_name="move_to_tree",
    run_actions=True,
    x=-0.56,
    y=0.60
)
tree.setup()
tree.run_until_done()
```

---

## Creating a New Tree

To add a new behavior tree:

1. **Create a new file** under `smart_home_pytree/trees/` (e.g., `follow_person_tree.py`).
2. **Inherit** from `BaseTreeRunner`.
3. **Implement** at least:
   - `create_tree(self, robot_interface)` → defines your BT structure  
   - Optionally, `required_actions()` and `required_topics()`  
4. **Register** it in `smart_home_pytree/__init__.py` if you want it callable via CLI.

---

## Required Actions and Topics

To make trees self-documenting, each tree should define what resources it needs:

```python
def required_actions(self):
    return {"my_robot_pkg": ["follow_person", "detect_person"]}

def required_topics(self):
    return ["/person_detected", "/robot_pose"]
```

If you run the tree with `run_actions=True` and these aren’t implemented, an error is raised to alert you that required action servers are missing.

---

## Simulation Mode

You can run any tree in simulation mode by setting:

```python
tree = MoveToLocationTree(node_name="move_to_tree", run_simulator=True)
```

This disables ROS2 action calls and prints simulation warnings, allowing testing without a physical robot.

---

## Tips for Developers

- **Keep logic in small reusable behaviors** — define them in `util_behaviors.py`.
- **Avoid hardcoding topics or actions** — use `required_actions()` and `required_topics()` to declare dependencies.
- **Use `self.kwargs`** for custom parameters like coordinates or thresholds.
- **Leverage `py_trees.display.unicode_tree()`** to debug structure during runtime.

---

## Example Output

When you run a tree, it prints its dependency summary:

```
[MoveToLocationTree] Required topics/actions:
 • /navigate_to_pose (action) — Nav2 move base action server
 • /undock (action) — Undocks the robot
 • /charging (topic) — Bool indicating if robot is charging
```

And shows the tree structure at every tick.

## For the scripts you can get help 
python3 move_to_tree.py --help

ree/trees$  python3 move_to_tree.py --help
usage: move_to_tree.py [-h] [--run_simulator] [--run_actions] [--run_until_done] [--x X]
                       [--y Y] [--qx QX] [--qy QY] [--qz QZ] [--qw QW]

Run move_to_location tree for robot navigation

options:
  -h, --help        show this help message and exit
  --run_simulator   Launch TB3 simulator with Nav2 (default: False)
  --run_actions     Launch action nodes (default: False)
  --run_until_done  Run tree until completion instead of continuously (default: True)
  --x X             X coordinate to move to (default: 0.0)
  --y Y             Y coordinate to move to (default: 0.0)
  --qx QX           X component of quaternion orientation (default: 0.0)
  --qy QY           Y component of quaternion orientation (default: 0.0)
  --qz QZ           Z component of quaternion orientation (default: 0.0)
  --qw QW           W component of quaternion orientation (default: 1.0)

Examples:
  # Run until done with custom coordinates
  python3 move_to_tree.py --run_actions --x -0.56 --y 0.60 --qx 0.0 --qy 0.0 --qz 0.0 --qw 1.0

  # Run continuously with default coordinates
  python3 move_to_tree.py --run_actions --run_until_done False

  # Run with simulator (continuous mode)
  python3 move_to_tree.py --run_simulator --run_actions --run_until_done False

  # Show this help message
  python3 move_to_tree.py --help
