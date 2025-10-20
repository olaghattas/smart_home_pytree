#!/usr/bin/env python3
import yaml
import py_trees
import py_trees_ros

def load_locations_to_blackboard(yaml_path: str):
    """
    Load location data from a YAML file and register it to the py_trees blackboard.
    """
    
    # Get blackboard
    blackboard = py_trees.blackboard.Blackboard()

    ## singleton-style check. To prevent writing data multiple times since it will be used
    ## in the subtrees.
    # --- Singleton guard ---
    if getattr(blackboard, "initialized", False):
        # Already done → skip reloading
        return blackboard
    # ------------------------
    
    # Load YAML
    with open(yaml_path, 'r') as file:
        data = yaml.safe_load(file)

    if "locations" not in data:
        raise KeyError("YAML file must contain a 'locations' field.")

    locations = data["locations"]
    
    # Register to blackboard
    blackboard.set("locations", locations)

    print("Registered the following locations to the blackboard:")
    for name, loc in locations.items():
        print(f"  {name}: {loc}")

    # Set flag (singleton-style marker)
    blackboard.initialized = True
    print("[Blackboard] Registered 'locations' once only.")
    
    return blackboard


if __name__ == "__main__":
    yaml_file_path = "/home/olagh48652/smart_home_pytree_ws/src/smart_home_pytree/config/house_info.yaml"
    bb = load_locations_to_blackboard(yaml_file_path)
