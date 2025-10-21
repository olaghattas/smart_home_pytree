# Imports
# from . import move_to_tree
# from . import subscription_tree

# from smart_home_pytree.trees import charge_robot_tree
from smart_home_pytree.trees import move_to_tree
from smart_home_pytree.trees import base_tree_runner

# TREE_REGISTRY = {
#     "charge_robot_tree": charge_robot_tree.run_tree,
#     "move_to_tree": move_to_tree.run_tree,
# }

# INFINITE_TREE_REGISTRY = {
#     "charge_robot_tree": charge_robot_tree.run_tree_infinite,
#     "move_to_tree": move_to_tree.run_tree_infinite,
# }

from . import util_behaviors
from . import robot_interface
from . import registry
