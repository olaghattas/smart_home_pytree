# smart_home_pytree

## Setup

``` 
mkdir -p smart_home_pytree_ws/src
cd ~/smart_home_pytree_ws/src

git clone https://github.com/splintered-reality/py_trees
git clone https://github.com/splintered-reality/py_trees_ros/
git clone https://github.com/splintered-reality/py_trees_ros_viewer
```
## Available Trees
These are trees that can be used to build a larger tree

move_to_tree: The tree receives an (x,y) position and a quaternion representing the target pose for the robot. Before initiating movement, it checks whether the robot is currently charging and, if so, commands it to undock first.

charge_robot_tree: Repsponsible for creating the charge robot tree. It takes the number of retries (num_attempts) as input. The tree should check if the is charging and exit. else it moves the robot to home position then dock the robot and checks if it succcessfully charged. It will try for num_attempts then log if it fails

TODO:

move_to_person: 

reminder_protocol: This subtree moves the robot to the person’s current location. It takes the number of retries (num_attempts) .It retrieves the person’s location, passes it to the MoveTo subtree, and checks if the robot reached the person. If the attempt fails, it retries up to num_attempts times before logging a failure message.

shutdown: 

## Helpful commands

to get a visualization of the tree:
py-trees-render -b smart_home_pytree.charge_robot_tree.create_charge_robot_tree

to play the tree:
py-trees-render -b smart_home_pytree.charge_robot_tree.charge_robot_tree_main




## Relevant documentation

https://py-trees-ros.readthedocs.io/en/devel/