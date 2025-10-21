import rclpy
from rclpy.node import Node

from std_msgs.msg import String, Bool
from threading import Lock

class RobotState:
    """Thread-safe state container."""
    def __init__(self):
        self._data = {}
        self._lock = Lock()
        self._in_use = False
        self._owner = None
    
    # Data Management    
    def update(self, key, value):
        """Update or add a key-value pair."""
        with self._lock:
            self._data[key] = value

    def get(self, key, default=None):
        """Get a value safely."""
        with self._lock:
            return self._data.get(key, default)
    def keys(self):
        """Return a list of all available keys in the state."""
        with self._lock:
            return list(self._data.keys())
        
    def to_dict(self):
        with self._lock:
            return dict(self._data)
        
    
    # Access Management
    def acquire(self, owner=None):
        """
        Attempt to acquire control of the state.
        Returns True if acquired successfully, False if already in use.
        """
        with self._lock:
            if self._in_use:
                print("RobotState in Use")
                return False
            self._in_use = True
            self._owner = owner
            return True

    def release(self):
        """Release control of the state."""
        with self._lock:
            self._in_use = False
            self._owner = None

    def is_in_use(self):
        """Return True if the state is currently locked (in use)."""
        with self._lock:
            return self._in_use
        
    # Debug / Introspection
    def __str__(self):
        """Readable printout when you do `print(state)`."""
        with self._lock:
            keys = list(self._data.keys())
            owner = self._owner or "None"
            return (
                f"=== RobotState ===\n"
                f"Keys: {keys if keys else 'No data yet'}\n"
                f"In use: {self._in_use}\n"
                f"Owner: {owner}\n"
                f"=================="
            )

    __repr__ = __str__  # same behavior in interactive shells


class RobotInterface(Node):
    """ROS2 interface to manage robot state and topics."""
    def __init__(self):
        super().__init__('robot_interface')
        
        self.state = RobotState()
        
        self.robot_location = self.create_subscription(String, 'robot_location', self.robot_location_callback, 10)
        self.person_location = self.create_subscription(String, 'person_location', self.person_location_callback, 10)
        self.charging_sub = self.create_subscription(Bool, 'charging', self.charging_callback, 10)

    def charging_callback(self, msg):
        self.state.update('charging', msg.data)
        self.get_logger().debug(f"Charging: {msg.data}")
        
    def robot_location_callback(self, msg):
        self.state.update('robot_location', msg.data)
        self.get_logger().debug(f"Person Location: {msg.data}")
        
    def person_location_callback(self, msg):
        self.state.update('person_location', msg.data)
        self.get_logger().debug(f"Person Location: {msg.data}")


def main(args=None):
    rclpy.init(args=args)

    robot_interface = RobotInterface()

    rclpy.spin(robot_interface)

    # Destroy the node explicitly
    robot_interface.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()