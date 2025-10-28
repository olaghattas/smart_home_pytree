import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String, Bool
import threading

class RobotState:
    """Thread-safe singleton state container."""

    _instance = None
    _lock = threading.Lock()

    def __new__(cls):
        if cls._instance is None:
            with cls._lock:
                if cls._instance is None:
                    cls._instance = super(RobotState, cls).__new__(cls)
                    cls._instance._data = {}
                    cls._instance._owner = None
        return cls._instance

    # Data Management
    def update(self, key, value):
        """Update or add a key-value pair."""
        self._data[key] = value

    def get(self, key, default=None):
        """Get a value safely."""
        return self._data.get(key, default)

    def keys(self):
        """Return a list of all available keys in the state."""
        return list(self._data.keys())

    # Debug / Introspection
    def __str__(self):
        keys = list(self._data.keys())
        owner = self._owner or "None"
        return (
            f"=== RobotState ===\n"
            f"Keys: {keys if keys else 'No data yet'}\n"
            f"Owner: {owner}\n"
            f"=================="
        )

    __repr__ = __str__

class RobotInterface(Node):
    """Singleton ROS2 interface running in a background thread."""

    _instance = None
    _lock = threading.Lock()

    def __new__(cls):
        if cls._instance is None:
            with cls._lock:
                if cls._instance is None:
                    cls._instance = super().__new__(cls)
        return cls._instance

    def __init__(self):
        if getattr(self, "_initialized", False):
            return

        if not rclpy.ok():
            rclpy.init()

        super().__init__('robot_interface')
        self.state = RobotState()
        
        # Subscriptions
        self.create_subscription(String, 'robot_location', self.robot_location_callback, 10)
        self.create_subscription(String, 'person_location', self.person_location_callback, 10)
        self.create_subscription(Bool, 'charging', self.charging_callback, 10)
        self.create_subscription(Bool, 'protocol_1', self.protocol_1_callback, 10)
        self.create_subscription(Bool, 'protocol_2', self.protocol_2_callback, 10)


        # Background spinning thread
        self._stop_event = threading.Event()
        self.spin_thread = threading.Thread(target=self._spin_background, daemon=True)
        self.spin_thread.start()

        self._initialized = True
        self.get_logger().info("RobotInterface initialized and spinning in background thread.")

    # --- Spinning ---
    def _spin_background(self):
        while rclpy.ok() and not self._stop_event.is_set():
            rclpy.spin_once(self, timeout_sec=0.1)

    def shutdown(self):
        """Gracefully stop the background spinner."""
        self.get_logger().info("Shutting down RobotInterface...")
        self._stop_event.set()
        self.spin_thread.join(timeout=1.0)
        self.destroy_node()
        self.get_logger().info("RobotInterface shutdown complete.")

    # --- Callbacks ---
    def robot_location_callback(self, msg):
        self.get_logger().debug(f"Robot location: {msg.data}")
        self.state.update('robot_location', msg.data)

    def person_location_callback(self, msg):
        self.get_logger().debug(f"Person location: {msg.data}")
        self.state.update('person_location', msg.data)

    def charging_callback(self, msg):
        self.get_logger().debug(f"Charging: {msg.data}")
        self.state.update('charging', msg.data)
        
    def protocol_1_callback(self, msg):
        self.get_logger().debug(f"protocol_1 : {msg.data}")
        self.state.update('protocol_1', msg.data)
    
    def protocol_2_callback(self, msg):
        self.get_logger().debug(f"protocol_2: {msg.data}")
        self.state.update('protocol_2', msg.data)


def get_robot_interface():
    if not rclpy.ok():
        rclpy.init()
    return RobotInterface()


def main():
    rclpy.init()
    robot_interface = get_robot_interface()
    try:
        while rclpy.ok():
            pass
    except KeyboardInterrupt:
        pass
    finally:
        robot_interface.shutdown()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
