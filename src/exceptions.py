class NoMotorMovementException(Exception):
    """Motor not moving. Check battery power or """
    def __init__(self, motorName):
        self.motorName = motorName


class MotorRunningFastException(Exception):
    """Motor running too fast!"""
    def __init__(self, motorName, speed):
        self.speed = speed
        self.motorName = motorName


class PIDControlerRunningTooLongException(Exception):
    """PID controler unable to stabilize the values of the target, taking too much time."""
    def __init__(self, reason):
        self.reason = reason


class ControlerRunningTooLongException(Exception):
    """Controler unable to stabilize the values of the target, taking too much time."""
    def __init__(self, reason):
        self.reason = reason


class MotorUrgentStopException(Exception):
    """Needs to stop because urgent need"""
    def __init__(self, motorName):
        self.motorName = motorName


class LowVoltageException(Exception):
    """Low voltage detected"""


class OutOfBoundAngle(Exception):
    """Angle out of bounds for link"""
    def __init__(self, reason, targetAngle):
        self.reason = reason
        self.targetAngle = targetAngle
