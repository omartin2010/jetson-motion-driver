# flake8: ignore=E501
from ev3dev2.motor import SpeedPercent
import threading
from mymotor import MyMotor
from logger import RoboLogger
import numpy as np
from numpy import radians
from constant import JOINT_TESTING_DIRECTION_MAX, JOINT_TESTING_DIRECTION_MIN, \
    MOTOR_INIT_METHOD_BUTTON_PRESSED, MOTOR_INIT_METHOD_DONT_INIT, \
    MOTOR_INIT_METHOD_MECHANICAL
from constant import LOGGER_EE_INIT, LOGGER_EE_FINDPHYLIMIT, \
    LOGGER_EE_RESETMOTOR, LOGGER_EE_ENDEFFECTOR

log = RoboLogger.getLogger()


class EndEffector(object):
    """
    Implementation of an arm joint (mainly the motor and methods around it being an arm joint vs a wheel for exampe)
    """
    __slots__ = [
        'motor',
        'status',
        '_polarity',
        'initSpeedPercent',
        'initMethod',
        '_urgentStop',
        'close_position',
        'open_position',
        'openClose',
        'lock'
    ]

    OPENED_GRIP = 1
    CLOSED_GRIP = 0
    GRIPPING_TIME = 3
    #: Power will be removed from the motor and it will freely coast to a stop.
    STOP_ACTION_COAST = MyMotor.STOP_ACTION_COAST
    #: Power will be removed from the motor and a passive electrical load will
    #: be placed on the motor.
    STOP_ACTION_BRAKE = MyMotor.STOP_ACTION_BRAKE
    #: Does not remove power from the motor. Instead it actively try to hold the motor
    #: at the current position. If an external force tries to turn the motor, the motor
    #: will `push back` to maintain its position.
    STOP_ACTION_HOLD = MyMotor.STOP_ACTION_HOLD

    def __init__(self, motor_address, status, initSpeedPercent, initMethod, openClose=0):
        log.debug(LOGGER_EE_INIT, 'Initializing end effector class.')
        self.motor = MyMotor(address=motor_address, rampUp=0, rampDown=0)
        self.status = status
        self.polarity = MyMotor.POLARITY_NORMAL
        self.initSpeedPercent = initSpeedPercent
        self.initMethod = initMethod
        self.urgentStop = False
        self.openClose = openClose
        self.lock = threading.Lock()

    # region properties
    @property
    def polarity(self):
        return self._polarity

    @polarity.setter
    def polarity(self, value):
        self._polarity = value
        self.motor.polarity = self._polarity

    @property
    def urgentStop(self):
        return self._urgentStop

    @urgentStop.setter
    def urgentStop(self, value):
        if value is True:
            log.debug(LOGGER_EE_ENDEFFECTOR, 'Need to immediately stop the arm joint.')
            self.motor.stop()
        self._urgentStop = value

    # endregion

    def initialize(self, percentSpeedTest, method, **kwargs):
        """ Initializes a joint motor by trying to reach limits without breaking the legos.
        Then calculates the ratio of where it is in comparison to the theoretical angles of the models
        Positions the joints as 0 and then exits the process.
        speedPercentTest
            desc : Percent at which to run the motors to test the limits
            type : decimal value > 0  and < 100
        method :
            description : method utilized to initialize the motor. As of now, there are 2 options
                            1. mechanical blocking indicates the max or min is reached
                            2. button pressed indicates the max or min is reached
            type : string, one of the MOTOR_INIT_METHOD constant of the BaseRobot class
        **kwargs :
            description : contains the button used for the init method with button press
        """
        if self.motor:
            if method == MOTOR_INIT_METHOD_DONT_INIT:
                # in this case we don't change the motor configuration and simply return
                log.info(LOGGER_EE_INIT, "Motor {} : Assuming already in the right position as method is been called not to initialize.".format(self.motor.kwargs['address']))
                self.resetMotor()
                self.open_position = self.motor.position
                self.close_position = - self.openClose
                return
            elif method != MOTOR_INIT_METHOD_DONT_INIT:
                close_position = self.findPhysicalJointLimit(percentSpeedTest, method, JOINT_TESTING_DIRECTION_MIN, **kwargs)
                log.info(LOGGER_EE_INIT, "Motor {} : Close Position : {:.2f} degrees".format(self.motor.kwargs['address'], close_position))
                open_position = self.findPhysicalJointLimit(percentSpeedTest, method, JOINT_TESTING_DIRECTION_MAX, **kwargs)
                log.info(LOGGER_EE_INIT, "Motor {} : Open Position : {:.2f} degrees".format(self.motor.kwargs['address'], open_position))
                # reset current motor position to 0
                log.info(LOGGER_EE_INIT, "Motor {} : Resetting motor position.".format(self.motor.kwargs['address']))
                self.resetMotor()
                # Re-calculate the relative close position
                close_position = close_position - open_position
                self.close_position = close_position
                self.open_position = self.motor.position
                self.status = self.OPENED_GRIP

    def findPhysicalJointLimit(self, percentSpeedTest, method, direction, **kwargs):

        # Change the direction of rotation if testing in the minimum direction
        if direction == JOINT_TESTING_DIRECTION_MIN:
            percentSpeedTest = -percentSpeedTest
        elif direction == JOINT_TESTING_DIRECTION_MAX:
            percentSpeedTest = percentSpeedTest
        else:
            raise("Problem, should be called with min or max.")

        # Wait for button press before starting testing
        if method == MOTOR_INIT_METHOD_BUTTON_PRESSED:
            button = kwargs["button"]
            log.info(LOGGER_EE_FINDPHYLIMIT, 'Waiting for button press before starting the initialization process for the motor.')
            button.wait_for_bump()
            log.info(LOGGER_EE_FINDPHYLIMIT, 'Button pressed. Initialization process started.')
        # Launch the motor at percentspeedtest...
        if method != MOTOR_INIT_METHOD_DONT_INIT:
            self.motor.on(SpeedPercent(percentSpeedTest), block=False)
            if method == MOTOR_INIT_METHOD_MECHANICAL:
                self.motor.wait_until(MyMotor.STATE_OVERLOADED)
            if method == MOTOR_INIT_METHOD_BUTTON_PRESSED:
                log.info(LOGGER_EE_FINDPHYLIMIT, 'Please press button when reaching joint limit...')
                button.wait_for_bump()
                log.info(LOGGER_EE_FINDPHYLIMIT, 'Button pressed. Moving to next step.')
            self.motor.off()
        else:
            if direction == JOINT_TESTING_DIRECTION_MIN:
                position = int(np.degrees(self.angleLimit[0] / self.gearRatio))
            elif direction == JOINT_TESTING_DIRECTION_MAX:
                position = int(np.degrees(self.angleLimit[1] / self.gearRatio))
            return position
        return self.motor.position

    def resetMotor(self):
        log.debug(LOGGER_EE_RESETMOTOR, 'Resetting motors and restoring polarity')
        self.motor.reset()
        # Looks useless... but the setter resets the motor's polarity
        self.polarity = self.polarity

    def closeEndEffector(self, blocking=False, stop_action=STOP_ACTION_BRAKE):
        if self.urgentStop:
            self.motor.off()
            return
        if self.lock.locked():
            log.warning(LOGGER_EE_ENDEFFECTOR, f'End effector thread locked. Waiting before closing EE...')
        with self.lock:
            log.info(LOGGER_EE_ENDEFFECTOR, f'Acquired lock for end effector.')
            log.debug(LOGGER_EE_ENDEFFECTOR, f'Building thread for closing end effector - at position {self.motor.position}, moving to position {self.close_position}.')
            thread = self.motor.on_to_position(currentphi_func=lambda: radians(self.motor.position),
                                               targetphi=self.close_position,
                                               pid=False,
                                               speedpercent=self.initSpeedPercent,
                                               stop_action=stop_action)
            log.debug(LOGGER_EE_ENDEFFECTOR, f'Launching thread for closing end effector.')
            thread.start()
            log.debug(LOGGER_EE_ENDEFFECTOR, f'Joining end effector close thread...')
            thread.join()
            log.info(LOGGER_EE_ENDEFFECTOR, f'Released control after starting motor to close end effector.')
            self.status = self.CLOSED_GRIP
        log.info(LOGGER_EE_ENDEFFECTOR, f'Released lock after end effector closed')

    def openEndEffector(self, blocking=False, stop_action=STOP_ACTION_BRAKE):
        if self.urgentStop:
            self.motor.off()
            return
        if self.lock.locked():
            log.warning(LOGGER_EE_ENDEFFECTOR, f'End effector thread locked. Waiting before opening EE...')
        with self.lock:
            log.info(LOGGER_EE_ENDEFFECTOR, f'Acquired lock for end effector.')
            log.debug(LOGGER_EE_ENDEFFECTOR, f'Building thread for opening end effector - at position {self.motor.position}, moving to position {self.close_position}.')
            thread = self.motor.on_to_position(currentphi_func=lambda: radians(self.motor.position),
                                               targetphi=self.open_position,
                                               pid=False,
                                               speedpercent=self.initSpeedPercent,
                                               stop_action=stop_action)
            log.debug(LOGGER_EE_ENDEFFECTOR, f'Launching thread for opening end effector.')
            thread.start()
            log.debug(LOGGER_EE_ENDEFFECTOR, f'Joining end effector open thread...')
            thread.join()
            log.info(LOGGER_EE_ENDEFFECTOR, f'Released control after starting motor to open end effector.')
            self.status = self.OPENED_GRIP
        log.info(LOGGER_EE_ENDEFFECTOR, f'Released lock after end effector opened')
