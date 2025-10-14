class MockRobotInterface:
    """
    Mock Robot Interface for testing.
    Allows setting charging status.
    """
    def __init__(self, charging=False):
        self.state = {'charging': charging}
