# flake8: ignore=E501
from logger import RoboLogger
import threading
from ev3dev2.motor import SpeedDPS    # , SpeedPercent
from mymotor import MyMotor
from gyrosensor import GyroSensor
import traceback
# import asyncio
import time
from exceptions import ControlerRunningTooLongException, MotorRunningFastException, MotorUrgentStopException, OutOfBoundAngle
from constant import JOINT_MAX_SPEED_PERCENT, MOVEMENT_TIME_SCALING_METHOD_DEFAULT, STOP_ACTION_HOLD   # noqa : F401
from numpy import degrees, radians
from constant import LOGGER_LINK_MAIN, LOGGER_LINK_MTA, LOGGER_LINK_RESET

log = RoboLogger.getLogger()


class ArmLink(object):
    """
    Implementation of an arm joint (mainly the motor and methods around it being an arm joint vs a wheel for exampe)
    """

    __slots__ = [
        'linkID',
        'motor',
        'urgentstop',
        'gyroAngleVerticalOffsetX',
        '_odoOffset',
        '_urgentStop',
        'virtual',
        'initOrder',
        'jointScrewB',
        '_gyroSensor',
        'gearRatio',
        'angleLimit',
        'depLink',
        'stopAction',
        '_polarity',
        '_motorMinSpeedDPS',
        '_motorMaxSpeedDPS',
        'maxDelta',
        '_killThread',
        'lock',
        'wiggleDeg'
    ]

    def __init__(self,
                 jointScrewB,
                 angleLimit,
                 linkID,
                 motorMinSpeedDPS=None,
                 motorMaxSpeedDPS=None,
                 maxDelta=None,
                 gearRatio=None,
                 initOrder=None,
                 gyroAngleVerticalOffsetX=None,
                 gyroLinkDep=None,
                 gyroSensor=None,
                 motorAddress=None,
                 stopAction=STOP_ACTION_HOLD,
                 polarity=None,
                 virtualLink=False,
                 wiggleDeg=5.0
                 ):

        '''
        Description:constructor for the robot arm link
        param:jointScrewB:desc:part of the B list for the joint in question
        param:jointScrewB:type:[float, float, float, float, float, float]
        param:angleLimit:desc: limit of the thing in radians, min and max
        param:angleLimit:type: [float, float]
        param:linkID:int:id of the link
        param:linkID:type:int
        param:motorMaxSpeedDPS:desc:max speed in DPS for motor operation
        param:motorMaxSpeedDPS:type:int
        param:motorMinSpeedDPS:desc:min speed in DPS for motor operation
        param:motorMinSpeedDPS:type:int
        param:maxDelta:desc:when at this angle delta for motion, motor
                            will be start at max speed.
        param:gearRatio:desc:gear ratio - used to estimate ETA for motion
        param:gearRatio:type:float
        param:initOrder:desc:order in which the links were initialized
        param:initOrder:type:int
        param:gyroAngleVerticalOffsetX:desc:degrees offset from vertical line to the zero for the robot (or None if virtual joint)
        param:gyroAngleVerticalOffsetX:type:float
        param:gyroLinkDep:desc:value of the link that this depends on.
        param:gyroLinkDep:int
        param:gyroSensor:desc:associated gyrosensor class
        param:gyroSensor:type:class `GyroSensor`
        param:motorAddress:desc: RoboMotor class
        param:motorAddress:type:
        param:stopAction:desc:STOP_ACTION_HOLD, STOP_ACTION_COAST, STOP_ACTION_BRAKE, says what happens
                                  when we're stopping the motor.
        param:stopAction:str
        param:polarity:desc:RoboMotor.POLARITY_INVERSED or RoboMotor.POLARITY_NORMAL so that + is in
                            one direction and - is in the other direction
        param:polarity:type:str
        param:virtualLink:desc:if the link has a motor, it's not virtual, if not, it is (like simulated links for robot base)
        param:virtualLink:bool
        param:wiggleDeg:float:used to determine precision of motion, eg at what point is a set point ok. If set to
                              40 degrees and wiggleDeg=2, then it's ok between 38 and 42.
        '''
        LOGGER_LINK_MAIN_INSTANCE = LOGGER_LINK_MAIN.format(linkID)
        self.linkID = linkID
        self.angleLimit = angleLimit
        if not virtualLink:
            log.debug(LOGGER_LINK_MAIN_INSTANCE, 'Initializing physical link...')
            self.motor = MyMotor(address=motorAddress)
            if self.motor:   # don't run if motor is set to None (testing mode)
                self.polarity = MyMotor.POLARITY_INVERSED if polarity is False else MyMotor.POLARITY_NORMAL
                self.motorMaxSpeedDPS = motorMaxSpeedDPS
                self.motorMinSpeedDPS = motorMinSpeedDPS
                self.maxDelta = maxDelta
            self._gyroSensor = GyroSensor(sensorPort=gyroSensor['input'],
                                          linkID=self.linkID,
                                          i2cBus=int(gyroSensor['i2cbus']),
                                          i2cAddress=int(gyroSensor['address'], 16),
                                          gyroFilterValue=1,
                                          gyroSensitivity='2G')
            self.stopAction = stopAction
            self.motor.stop(stop_action=self.stopAction)
            self.urgentStop = False
            self.virtual = False
            self.gearRatio = gearRatio
            self.gyroAngleVerticalOffsetX = gyroAngleVerticalOffsetX
            self.depLink = gyroLinkDep
            self.wiggleDeg = wiggleDeg
            if self.depLink is None:
                self.odoOffset = self.gyroAngleVerticalOffsetX
        else:
            log.debug(LOGGER_LINK_MAIN_INSTANCE, 'Initializing virtual link...')
            self.virtual = True
        self.initOrder = initOrder
        self.jointScrewB = jointScrewB
        self.lock = threading.Lock()
        log.info(LOGGER_LINK_MAIN_INSTANCE, 'Done link constructor')

    # region properties, setters and getters
    @property
    def motorMinSpeedDPS(self):
        return self._motorMinSpeedDPS

    @motorMinSpeedDPS.setter
    def motorMinSpeedDPS(self, value):
        if value > 0:
            self._motorMinSpeedDPS = int(value)

    @property
    def motorMaxSpeedDPS(self):
        return self._motorMaxSpeedDPS

    @motorMaxSpeedDPS.setter
    def motorMaxSpeedDPS(self, value):
        if value > 0:
            self._motorMaxSpeedDPS = int(value)

    @property
    def killThread(self):
        return self._killThread

    @killThread.setter
    def killThread(self, value):
        self.gyroSensor.killThread = value

    @property
    def gyroSensor(self):
        return self._gyroSensor

    @gyroSensor.setter
    def gyroSensor(self, value):
        if isinstance(value, GyroSensor):
            self._gyroSensor = value
        else:
            raise("Wrong class for the angle sensor.")

    @property
    def armAngleDegX(self):
        if self.depLink:
            # add readings from dependnts
            self.odoOffset = self.depLink.odoOffset + self.depLink.armAngleDegX
        return (90 - self.odoOffset + self.gyroSensor.currentAngleXDeg)

    @property
    def armAngleRadX(self):
        return radians(self.armAngleDegX)

    @property
    def armAngleDegY(self):
        return self._gyroSensor.currentAngleYDeg

    @property
    def armAngleDegZ(self):
        return self._gyroSensor.currentAngleZDeg

    @property
    def odoOffset(self):
        return self._odoOffset

    @odoOffset.setter
    def odoOffset(self, value):
        if isinstance(float(value), float):
            self._odoOffset = float(value)
        else:
            raise("wrong type, has to be float")

    @property
    def polarity(self):
        return self._polarity

    @polarity.setter
    def polarity(self, value):
        self._polarity = value
        self.motor.polarity = self._polarity

    @property
    def position(self):
        return self._motor.position

    @property
    def urgentStop(self):
        return self._urgentStop

    @urgentStop.setter
    def urgentStop(self, value):
        if value:
            self.motor.off()
        self._urgentStop = value

    # endregion setters and getters

    def resetLink(self):
        '''
        Description:resets the motors.
        params:None
        '''
        LOGGER_LINK_RESET_INSTANCE = LOGGER_LINK_RESET.format(self.linkID)
        log.debug(LOGGER_LINK_RESET_INSTANCE, 'checking if link is virtual...')
        if not self.virtual:
            log.debug(LOGGER_LINK_RESET_INSTANCE, 'link non virtual, resetting.')
            self.motor.reset()
            log.debug(LOGGER_LINK_RESET_INSTANCE, 'Link motor reset.')
            # Looks useless... but the setter resets the motor's polarity
            self.motor.stop(stop_action=self.stopAction)
            self.polarity = self.polarity

    def moveToAngle(self,
                    targetAngleRadians,
                    dryrun=False):
        '''
        Description:meant to move link to position (in radians) at a certain fixed speed
        param:targetAngleRadians:desc:target angle in radians
        param:targetAngleRadians:type:float
        # param:stop_action:desc:what to do when finished ('coast', 'brake', 'hold'
        #     choices of ev3dev2.motor.LargeMotor.STOP_ACTION_COAST
        #                ev3dev2.motor.LargeMotor.STOP_ACTION_BRAKE
        #                ev3dev2.motor.LargeMotor.STOP_ACTION_HOLD
        param:stop_action:type:str
        param:dryrun:desc:if true, won't run, if false, will do.
        param:dryrun:type:bool
        '''
        LOGGER_LINK_MTA_INSTANCE = LOGGER_LINK_MTA.format(self.linkID)

        try:
            if targetAngleRadians < self.angleLimit[0] or targetAngleRadians > self.angleLimit[1]:
                raise OutOfBoundAngle('Setpoint does not fit within acceptable bounds', targetAngle=targetAngleRadians)
            with self.lock:
                # 0. Check for urgent stop
                if self.urgentStop:
                    raise MotorUrgentStopException(self.motor.description)

                # 1. Estimate time to target:
                currentAngleDegrees = self.armAngleDegX
                targetAngleDegrees = degrees(targetAngleRadians)
                eta = abs(targetAngleDegrees - currentAngleDegrees) / abs(self.motorMinSpeedDPS) * self.gearRatio

                # 2. Initialize variables
                idx = 0
                startTime = time.time()
                initialMotorPosition = self.motor.position
                maxLoopTimeFactor = 4
                logEvery = 1
                avgStepDuration = 0

                if dryrun:
                    log.warning(LOGGER_LINK_MTA_INSTANCE, "Step {0} - Not moving motor {1} : dry run".format(idx, self.motor.kwargs['address']))
                    return
                angleError = abs(targetAngleDegrees - self.armAngleDegX)
                thetaDotPrev = 0
                while angleError > self.wiggleDeg:
                    loopStartTime = time.time()

                    # Check for silly and urgent conditions
                    if self.urgentStop:
                        raise MotorUrgentStopException(self.motor.description)
                    if self.armAngleRadX < self.angleLimit[0]:
                        log.critical(LOGGER_LINK_MTA_INSTANCE, f'Current angle {self.armAngleDegX} lower than angleLimit[0] {degrees(self.angleLimit[0])}')
                        raise OutOfBoundAngle('While moving arm link, we reached the lower angle limit.', self.armAngleDegX)
                    if self.armAngleRadX > self.angleLimit[1]:
                        log.critical(LOGGER_LINK_MTA_INSTANCE, f'Current angle {self.armAngleDegX} higher than angleLimit[1] {degrees(self.angleLimit[0])}')
                        raise OutOfBoundAngle('While moving arm link, we reached the upper angle limit.', self.armAngleDegX)
                    if loopStartTime - startTime > maxLoopTimeFactor * eta:
                        raise ControlerRunningTooLongException(f'Took over {maxLoopTimeFactor} times original anticipated ETA (of {eta:.2f}s ==> {maxLoopTimeFactor} * {eta:.2f}s = {maxLoopTimeFactor*eta:.2f}s).')
                    if loopStartTime - startTime > 3 and self.motor.position == initialMotorPosition:
                        raise ControlerRunningTooLongException('After 3 seconds, the motor is still in the same initial position')
                    if abs(self.motor.speed - self.motor.max_dps) < 50:
                        raise MotorRunningFastException(self.motor.description)

                    # Calculate motor speed for step
                    delta = angleError
                    thetaDot = min(((self.motorMaxSpeedDPS - self.motorMinSpeedDPS) / self.maxDelta) * delta + self.motorMinSpeedDPS, self.motorMaxSpeedDPS)
                    # Calculate direction of motor rotation
                    thetaDot = int(-thetaDot) if self.armAngleDegX > targetAngleDegrees else int(thetaDot)
                    # Validate if motor speed needs to change
                    if thetaDotPrev != thetaDot:
                        log.debug(LOGGER_LINK_MTA_INSTANCE, f'Step {idx} - Updating motor {self.motor.address}\'s speed from {thetaDotPrev} to {thetaDot} dps.')
                        self.motor.on(SpeedDPS(thetaDot), brake=False, block=False)

                    # Note current angle error and last motor speed
                    thetaDotPrev = thetaDot
                    angleError = abs(targetAngleDegrees - self.armAngleDegX)

                    stepDuration = time.time() - loopStartTime
                    avgStepDuration += stepDuration
                    if idx % logEvery == 0:
                        avgStepDuration /= logEvery
                        infoLogString = f'Step {idx+1} - Avg. Loop : {avgStepDuration:.3f}s => Position/Target/Error = ' \
                                        f'{self.armAngleDegX:.2f}/{targetAngleDegrees:.2f}/{angleError:.2f}'
                        log.debug(LOGGER_LINK_MTA_INSTANCE, infoLogString)
                        avgStepDuration = 0
                    idx += 1
                # Stop the motor at this point - and hold position
                log.debug(LOGGER_LINK_MTA_INSTANCE, f'Final position = {self.armAngleDegX:.2f} deg - set point is {targetAngleDegrees:.2f} deg')
        except OutOfBoundAngle as error:
            log.error(LOGGER_LINK_MTA_INSTANCE, f'Asked for an angle for this link out of bounds - asked : {error.targetAngle}, limits = [{degrees(self.angleLimit[0]):.2f}, {degrees(self.angleLimit[1]):.2f}]. Reason = {error.reason}')
        except MotorRunningFastException as error:
            log.error(LOGGER_LINK_MTA_INSTANCE, "Motor {} running too fast ({}) - exiting and stopping motor".format(error.motorName, error.speed))
            raise
        except ControlerRunningTooLongException as error:
            log.error(LOGGER_LINK_MTA_INSTANCE, "MoveToAngle routine running for too long. Something is likely wrong.")
            log.error(LOGGER_LINK_MTA_INSTANCE, "Detailed Error Message = {}".format(error.reason))
        except MotorUrgentStopException as error:
            log.error(LOGGER_LINK_MTA_INSTANCE, "Urgent Motor Stop called. {}".format(error.motorName))
            self.resetLink()
        except:
            log.error(LOGGER_LINK_MTA_INSTANCE, 'Error : {}'.format(traceback.print_exc()))
            raise
        finally:
            self.motor.stop(stop_action=self.stopAction)
        return
