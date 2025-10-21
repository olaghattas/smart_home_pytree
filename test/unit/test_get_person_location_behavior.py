import pytest
import py_trees
from smart_home_pytree.util_behaviors import GetPersonLocation
from smart_home_pytree.registry import load_locations_to_blackboard


@pytest.fixture(scope="module")
def setup_blackboard():
    """Fixture to load the house locations into the blackboard."""
    yaml_file_path = "/home/olagh48652/smart_home_pytree_ws/src/smart_home_pytree/config/house_info.yaml"
    load_locations_to_blackboard(yaml_file_path)
    blackboard = py_trees.blackboard.Blackboard()
    return blackboard


def test_person_location_valid(setup_blackboard):
    """Test when the person location is a valid room."""
    blackboard = setup_blackboard
    target_location = blackboard.get("locations")
    loc = "kitchen"

    # Simulate blackboard and state
    blackboard.set("person_location", loc)
    state = {"person_location": loc}

    # Run GetPersonLocation
    get_person_loc = GetPersonLocation(state)
    status = get_person_loc.update()

    # Expected behavior
    assert loc in target_location
    assert status == py_trees.common.Status.SUCCESS


def test_person_location_not_registered(setup_blackboard):
    """Test when the location is not in the registered house locations."""
    blackboard = setup_blackboard
    loc = "garage"  # Not in YAML
    state = {"person_location": loc}

    blackboard.set("person_location", loc)
    get_person_loc = GetPersonLocation(state)
    status = get_person_loc.update()

    assert status == py_trees.common.Status.FAILURE


def test_person_location_none(setup_blackboard):
    """Test when the person location is None."""
    blackboard = setup_blackboard
    state = {"person_location": None}

    get_person_loc = GetPersonLocation(state)
    status = get_person_loc.update()

    assert status == py_trees.common.Status.FAILURE


def test_person_location_missing_key(setup_blackboard):
    """Test when the state dictionary does not contain the key 'person_location'."""
    state = {}  # Missing key
    get_person_loc = GetPersonLocation(state)
    status = get_person_loc.update()

    assert status == py_trees.common.Status.FAILURE


# folder: /home/olagh48652/smart_home_pytree_ws/src/smart_home_pytree/test
# python3 -m pytest -v unit/test_get_person_location_behavior.py
# =========================================================================================== test session starts ============================================================================================
# platform linux -- Python 3.10.12, pytest-6.2.5, py-1.11.0, pluggy-0.13.0 -- /usr/bin/python3
# cachedir: .pytest_cache
# rootdir: /home/olagh48652/smart_home_pytree_ws/src/smart_home_pytree
# plugins: launch-testing-ros-0.19.7, launch-pytest-1.0.5, launch-testing-1.0.5, ament-xmllint-0.12.10, ament-pep257-0.12.10, ament-mypy-0.12.10, ament-flake8-0.12.10, ament-copyright-0.12.10, ament-lint-0.12.10, langsmith-0.4.27, colcon-core-0.20.1, anyio-4.11.0, cov-3.0.0, mock-3.6.1, timeout-2.1.0, repeat-0.9.1, rerunfailures-10.2, typeguard-4.3.0
# collected 4 items                                                                                                                                                                                          

# unit/test_get_person_location_behavior.py::test_person_location_valid PASSED                                                                                                                         [ 25%]
# unit/test_get_person_location_behavior.py::test_person_location_not_registered PASSED                                                                                                                [ 50%]
# unit/test_get_person_location_behavior.py::test_person_location_none PASSED                                                                                                                          [ 75%]
# unit/test_get_person_location_behavior.py::test_person_location_missing_key PASSED                                                                                                                   [100%]

# ============================================================================================ 4 passed in 0.49s =============================================================================================
# olagh48652@olagh-Legion-5-15IAH7H:~/smart_home_pytree_ws/src/smart_home_pytree/test$ 
