# flake8: ignore=E501
from logger import RoboLogger
import sys
import threading
import traceback
import time
import numpy as np
import modern_robotics as mr
from numpy import pi, arccos, arctan, degrees, radians, cos, sin
from utils import normalizeAngle
from constant import STOP_ACTION_BRAKE, STOP_ACTION_COAST, STOP_ACTION_HOLD   # noqa: F401
from constant import MOVEMENT_TIME_SCALING_METHOD_DEFAULT, MOVEMENT_TIME_SCALING_METHOD_CUBIC, MOVEMENT_TIME_SCALING_METHOD_QUINTIC, MOTOR_RAMP_UP_TIME  # noqa: F401
from constant import ODOMETRY_BASE_LOGGING_LOOP_AGGREGATED
from mymotor import MyMotor
from constant import LOGGER_BASE_MAIN, LOGGER_BASE_MOVEX, LOGGER_BASE_MOVEXYPHI, \
    LOGGER_BASE_ODOMETRY, LOGGER_BASE_PRIMWHEELMVT, LOGGER_BASE_ROTATEBASEPHI, LOGGER_BASE_RUN

log = RoboLogger.getLogger()


class DiffDriveBase(object):
    """
    Implementation of the robot's base
    """

    __slots__ = [
        'leftMotor',
        'rightMotor',
        '_urgentStop',
        'currentX',
        'currentY',
        'currentPhi',
        'current_omega',
        '_current_V',
        'oldLeftPhi',
        'oldRightPhi',
        'max_omega',
        'max_V',
        'wheelRadius',
        'maxBaseSpeed',
        'baseHeight',
        'interWheelDistance',
        # 'wheelCircumference',
        'threadOdometry',
        'odometryTimer',
        'Vb6',
        'lock',
        'ThreadImplOdometry'
    ]

    def __init__(self,
                 baseDefinition,
                 initRobotConfig,
                 odometryTimer):
        '''
        Description:constructor for the robot arm
        param:baseDefinition:dictionary defining the base of the robot'
        param:baseDefinition:type:dictionary (see robot-def-??.json)
        param:initRobotConfig:desc:config vector x,y, phi of base at initialization
        param:initRobotConfig:type:[float, float, float] (list of float)
        '''
        if baseDefinition["motors"]["leftMotor"]["output"]:
            log.info(LOGGER_BASE_MAIN, 'Initializing left motor')
            self.leftMotor = MyMotor(address=baseDefinition["motors"]["leftMotor"]["output"],
                                     description=baseDefinition["motors"]["leftMotor"]["description"],
                                     defaultStopAction=baseDefinition["motors"]["leftMotor"]["defaultStopAction"])
        else:
            log.info(LOGGER_BASE_MAIN, 'No left motor to initialize')
            self.leftMotor = None
        if baseDefinition["motors"]["rightMotor"]["output"]:
            log.info(LOGGER_BASE_MAIN, 'Initializing right motor')
            self.rightMotor = MyMotor(address=baseDefinition["motors"]["rightMotor"]["output"],
                                      description=baseDefinition["motors"]["rightMotor"]["description"],
                                      defaultStopAction=baseDefinition["motors"]["rightMotor"]["defaultStopAction"])
        else:
            log.info(LOGGER_BASE_MAIN, 'No right motor to initialize')
            self.rightMotor = None
        log.info(LOGGER_BASE_MAIN, 'Initializing other parameters for the base.')
        self.wheelRadius = baseDefinition["sizing"]["wheelRadius"]
        self.max_V = radians(self.leftMotor.max_speed) * self.wheelRadius
        self.baseHeight = baseDefinition["sizing"]["baseHeight"]
        self.interWheelDistance = baseDefinition["sizing"]["interWheelDistance"]
        # self.wheelCircumference = 2 * pi * self.wheelRadius
        self.max_omega = self.max_V / self.interWheelDistance / 2
        self.currentX, self.currentY, self.currentPhi = initRobotConfig
        self.odometryTimer = odometryTimer
        self.lock = threading.Lock()

    # region properties
    @property
    def urgentStop(self):
        return self._urgentStop

    @urgentStop.setter
    def urgentStop(self, value):
        self._urgentStop = value
        if self.leftMotor:
            self.leftMotor.urgentstop = value
        if self.rightMotor:
            self.rightMotor.urgentstop = value

    @property
    def current_V(self):
        return self._current_V

    @current_V.setter
    def current_V(self, value):
        assert(value <= self.max_V), "Too high speed, unsupported."
        self._current_V = value

    @property
    def baseConfigVector(self):
        return np.array[self.leftMotor.wheelphi, self.rightMotor.wheelphi]

    @property
    def F(self):
        C = 1 / self.interWheelDistance
        return self.wheelRadius * np.array([[-C, C],
                                            [0.5, 0.5],
                                            [0.0, 0.0]], dtype=np.float32)

    @property
    def F6(self):
        return np.array([np.zeros_like(self.F[0]),
                         np.zeros_like(self.F[0]),
                         self.F[0],
                         self.F[1],
                         self.F[2],
                         np.zeros_like(self.F[0])], dtype=np.float32)

    @property
    def currentConfigVector(self):
        return np.r_[self.base.currentX,
                     self.base.currentY,
                     self.base.currentPhi,
                     self.leftMotor.wheelphi,
                     self.rightMotor.wheelphi]

    @currentConfigVector.setter
    def currentConfigVector(self, value):
        self.currentX = value[0]
        self.currentY = value[1]
        self.currentPhi = value[2]
        # ignoring wheel phis, they're set with the position of the motor

    @property
    def Tsb(self):
        return np.array([[cos(self.currentPhi), -sin(self.currentPhi),  0, self.currentX],
                        [sin(self.currentPhi),  cos(self.currentPhi),  0, self.currentY],
                        [                   0,                     0,  1, self.baseHeight],
                        [                   0,                     0,  0, 1]])

    # endregion

    def run(self):
        '''
        Description : will start required functions for the base (odometry, etc.)
        '''
        log.warning(LOGGER_BASE_RUN, 'Launching Odometry Thread for robot base...')
        self.threadOdometry = threading.Thread(
            target=self.threadImplOdometry,
            args=([self.odometryTimer, ODOMETRY_BASE_LOGGING_LOOP_AGGREGATED]),
            name="threadImplOdometry")
        self.threadOdometry.live = True
        self.threadOdometry.start()
        log.warning(LOGGER_BASE_RUN, 'Odometry Thread for robot base')

    def __primitiveWheelMovement(self,
                                 wheelDesiredPosition,
                                 pid,
                                 velocity_factor,
                                 method=MOVEMENT_TIME_SCALING_METHOD_DEFAULT,
                                 dryrun=False):
        '''
        Description : moves the wheels to the desired position, handles locking etc.
        param:wheelDesiredPosition:desc:desired position (in radians) of wheel positions
        param:wheelDesiredPosition:type:tuple (leftwheelphi, rightwheelphi)
        velocity_factor:desc:performance factor - motor power percentage
        velocity_factor:type:float, 0<x<1
        method:desc:time scaling method = cubic if 3, quintic if 5 (quintinc is smoother)
        method:type:int (only valid with 3 or 5)
        dryrun:desc:indicates whether it's going to run or not
        dryrun:type:bool
        '''
        try:
            # Adjust movement speed
            self.current_omega = self.max_omega * velocity_factor
            self.current_V = self.max_V * velocity_factor
            target_left_phi, target_right_phi = wheelDesiredPosition
            assert(method == MOVEMENT_TIME_SCALING_METHOD_CUBIC or method == MOVEMENT_TIME_SCALING_METHOD_QUINTIC), "Invalid time scaling method."
            try:
                if self.lock.locked():
                    log.debug(LOGGER_BASE_PRIMWHEELMVT, 'Base is locked. Waiting before next move.')
                else:
                    log.debug(LOGGER_BASE_PRIMWHEELMVT, 'Base is unlocked, proceeding.')
                with self.lock:
                    log.debug(LOGGER_BASE_PRIMWHEELMVT, "Locked Base : Motion Start, moving to targetphi = {:.2f}, {:.2f}".format(degrees(target_left_phi), degrees(target_right_phi)))
                    leftThread = self.leftMotor.on_to_position(currentphi_func=lambda: self.leftMotor.wheelphi,
                                                               targetphi=int(degrees(target_left_phi)),
                                                               pid=pid,
                                                               timestep=0.1,
                                                               method=method,
                                                               speedpercent=velocity_factor * 100,
                                                               stop_action=STOP_ACTION_COAST,
                                                               # block=False,
                                                               dryrun=dryrun)
                    rightThread = self.rightMotor.on_to_position(currentphi_func=lambda: self.rightMotor.wheelphi,
                                                                 targetphi=int(degrees(target_right_phi)),
                                                                 pid=pid,
                                                                 timestep=0.1,
                                                                 method=method,
                                                                 speedpercent=velocity_factor * 100,
                                                                 stop_action=STOP_ACTION_COAST,
                                                                 # block=False,
                                                                 dryrun=dryrun)
                    # Launch both threads for move
                    leftThread.start()
                    rightThread.start()
                    leftThread.join(timeout=30)
                    rightThread.join(timeout=30)
                    log.debug(LOGGER_BASE_PRIMWHEELMVT, "Motion segment completed.")
                log.debug(LOGGER_BASE_PRIMWHEELMVT, 'Unlocked Base.')
            except:
                log.error(LOGGER_BASE_PRIMWHEELMVT, 'Error {0} - {1}'.format(sys.exc_info()[0], sys.exc_info()[1]))
                raise
        except:
            log.error(LOGGER_BASE_PRIMWHEELMVT, 'Error in base rotation : {}'.format(traceback.print_exc()))
            raise
        finally:
            self.leftMotor.off()
            self.rightMotor.off()

    def rotateBasePhi(self,
                      phi,
                      pid,
                      velocity_factor=0.1,
                      method=MOVEMENT_TIME_SCALING_METHOD_DEFAULT,
                      inter_wheel_distance=None,
                      dryrun=False):
        """ used to start rotating the body of the bot for theta degrees at angular velocity omega
            params
            theta:desc: angle wrt current body orientation
            theta:type: float in radians
            velocity_factor:desc: factor slowing down the rotation
            velocity_factor:type: float > 0 <=1
            method:desc:time scaling method = cubic if 3, quintic if 5 (quintinc is smoother)
            method:type:int (only valid with 3 or 5)
            inter_wheel_distance:float:inter wheel distance (for debugging - if set to none, will use configuration...)
            dryrun:desc:indicates whether it's going to run or not
            dryrun:type:bool
        """

        try:
            iwd = self.interWheelDistance if inter_wheel_distance is None else inter_wheel_distance
            # Adjust movement speed
            self.current_omega = self.max_omega * velocity_factor
            self.current_V = self.max_V * velocity_factor

            assert(method == MOVEMENT_TIME_SCALING_METHOD_CUBIC or method == MOVEMENT_TIME_SCALING_METHOD_QUINTIC), "Invalid time scaling method."

            if phi != 0:
                # ensure we're not asked to do 1.5 turn... when we can do 1/2.
                phi = normalizeAngle(phi, lower=-pi, upper=pi)
                log.info(LOGGER_BASE_ROTATEBASEPHI, 'Rotating by {:.2f} degrees (normalized angle)'.format(degrees(phi)))

                delta_target_left_phi = -iwd / 2 * phi / self.wheelRadius
                delta_target_right_phi = iwd / 2 * phi / self.wheelRadius
                target_left_phi = self.leftMotor.wheelphi + delta_target_left_phi
                target_right_phi = self.rightMotor.wheelphi + delta_target_right_phi
                # Call the actual motion
                self.__primitiveWheelMovement((target_left_phi, target_right_phi),
                                              pid=pid,
                                              velocity_factor=velocity_factor,
                                              method=method,
                                              dryrun=dryrun)
                log.info(LOGGER_BASE_ROTATEBASEPHI, f'Done rotation by {degrees(phi):.2f} degrees (normalized angle)')
                log.debug(LOGGER_BASE_ROTATEBASEPHI, f'Target Left/Right wheel phi = {degrees(target_left_phi):.2f}/{degrees(target_right_phi):.2f}')
                log.debug(LOGGER_BASE_ROTATEBASEPHI, f'Actual Left/Right wheel phi = {degrees(self.leftMotor.wheelphi):.2f}/{degrees(self.rightMotor.wheelphi):.2f}')
            else:
                log.info(LOGGER_BASE_ROTATEBASEPHI, f'Not moving, asked angle Phi = {degrees(phi)}')
        except:
            log.error(LOGGER_BASE_ROTATEBASEPHI, f'Error in base rotation : {traceback.print_exc()}')
            raise

    def moveBaseX(self,
                  distance,
                  pid,
                  velocity_factor=0.1,
                  method=MOVEMENT_TIME_SCALING_METHOD_QUINTIC,
                  wheel_radius=None,
                  dryrun=False):
        """
        used to start moving the body of the bot for x distance at v velocity
        params:
        distance:desc: angle wrt current body orientation
        distance:type: float in meters
        velocity_factor:desc:performance factor - motor power percentage
        velocity_factor:type:float, 0<x<1
        method:desc:time scaling method = cubic if 3, quintic if 5 (quintinc is smoother)
        method:type:int (only valid with 3 or 5)
        wheel_radius:float:wheel diameter - used for debugging, if none, will use configuration value
        dryrun:desc:indicates whether it's going to run or not
        dryrun:type:bool
        """

        try:
            wr = self.wheelRadius if wheel_radius is None else wheel_radius

            # Adjust movement speed
            self.current_omega = self.max_omega * velocity_factor
            self.current_V = self.max_V * velocity_factor

            assert(method == MOVEMENT_TIME_SCALING_METHOD_CUBIC or method == MOVEMENT_TIME_SCALING_METHOD_QUINTIC), "Invalid time scaling method."

            if distance != 0:
                log.info(LOGGER_BASE_MOVEX, f'Moving forward by {distance:.2f} meters')
                delta_target_left_phi = distance / self.wheelCircumference(debugWheelRadius=wr) * 2 * pi
                delta_target_right_phi = distance / self.wheelCircumference(debugWheelRadius=wr) * 2 * pi
                target_left_phi = self.leftMotor.wheelphi + delta_target_left_phi
                target_right_phi = self.rightMotor.wheelphi + delta_target_right_phi
                # Call the actual motion
                self.__primitiveWheelMovement((target_left_phi, target_right_phi),
                                              pid=pid,
                                              velocity_factor=velocity_factor,
                                              method=method,
                                              dryrun=dryrun)
                log.info(LOGGER_BASE_MOVEX, f'Done moving forward by {distance:.2f} meters')
                log.debug(LOGGER_BASE_MOVEX, f'Target Left/Right wheel phi = {degrees(target_left_phi):.2f}/{degrees(target_right_phi):.2f}')
                log.debug(LOGGER_BASE_MOVEX, f'Actual Left/Right wheel phi = {degrees(self.leftMotor.wheelphi):.2f}/{degrees(self.rightMotor.wheelphi):.2f}')
            else:
                log.info(LOGGER_BASE_MOVEX, f'Not moving, asked distance = {distance:.2f} meters')
        except:
            log.error(LOGGER_BASE_MOVEX, f'Error in base forward motion : {traceback.print_exc()}')
            raise

    def moveBaseXYPhi(self,
                      endBaseConfig,
                      pid,
                      velocity_factor=0.1,
                      time_scaling_method=MOVEMENT_TIME_SCALING_METHOD_QUINTIC,
                      dryrun=True):
        '''
        Given a target body config vector (endBaseConfig : X, Y, phi), move the bot from the current position
        to this new position
        endBaseConfig = x, y, phi
                        type : float
        velocity_factor = 1 = 100%, 0 = 0%
                                type : float
        dryrun = true : will not drive the robot. False will
        '''

        # Adjust movement speed
        self.current_omega = self.max_omega * velocity_factor
        self.current_V = self.max_V * velocity_factor

        # Calculate coordinates in space frame (fixed frame starting at robot init)
        target_x, target_y, target_phi = endBaseConfig

        R = np.array([[1, 0, 0],
                      [0, 1, 0],
                      [0, 0, 1]])
        p = np.array((target_x, target_y, 0))
        Tdest = mr.RpToTrans(R, p)

        # Convert rotation + translation from base frame in original space frame
        Ts_dest = self.Tsb@Tdest

        # use coordinates in space frame to perform move.
        R, p = mr.TransToRp(Ts_dest)
        target_x, target_y = p[0], p[1]

        # calculate spin angle (check for singularity (if theta = -180 or pi or -pi), or if
        # there's only a spin angle, no movement in X or Y)
        # arctan is defined between -pi/2 and pi/2... so target_x has to be larger
        if target_x == self.currentX and target_y == self.currentY:
            phi = 0
        elif target_x >= self.currentX:       # quadrants 1 and 4ar
            phi = arctan(np.float64(target_y - self.currentY) / np.float64(target_x - self.currentX))
            if np.isnan(phi):
                phi = 0
        elif target_y >= self.currentY:         # arccos is defined between 0 and pi (quadrant 2)
            phi = arccos(np.float64(target_x - self.currentX) / (np.sqrt(np.power(target_x - self.currentX, 2) + np.power(target_y - self.currentY, 2))))
        elif target_y < self.currentY and target_x < self.currentX:     # below the X axis, 3th quadrant
            phi = pi + arctan(np.float64(target_y - self.currentY) / np.float64(target_x - self.currentX))
        else:
            log.info(LOGGER_BASE_MOVEXYPHI, 'We have a problem... check for singularity configurations')
        # subtract current angle
        phi -= self.currentPhi

        # First Spin
        self.rotateBasePhi(phi=phi,
                           pid=pid,
                           velocity_factor=velocity_factor,
                           method=time_scaling_method,
                           dryrun=dryrun)
        # Translate
        distance = np.linalg.norm((np.array([target_x, target_y]) - np.array([self.currentX, self.currentY])))
        self.moveBaseX(distance=distance,
                       pid=pid,
                       velocity_factor=velocity_factor,
                       method=time_scaling_method,
                       dryrun=dryrun)

        # Last spin for final orientation
        self.rotateBasePhi(phi=target_phi,
                           pid=pid,
                           velocity_factor=velocity_factor,
                           method=time_scaling_method,
                           dryrun=dryrun)

    def wheelCircumference(self, debugWheelRadius=None):
        if not debugWheelRadius:
            return 2 * pi * self.wheelRadius
        else:
            return 2 * pi * debugWheelRadius

    def stopThreadOdometry(self):
        '''
        Description : stops the odometry thread.
        '''
        self.threadOdometry.live = False
        self.threadOdometry.join()

    def resetBodyConfiguration(self):
        '''
        will reset coordinates of the space frame to 0,0,0
        '''
        self.leftMotor.reset()
        self.rightMotor.reset()
        self.currentConfigVector = [0, 0, 0, 0, 0]
        self.oldLeftPhi, self.oldRightPhi = 0, 0

    def threadImplOdometry(self, loopDelay, logEvery):
        '''
        Odometry loop. Thread started by the __init__ function of the robot
        loopDelay is the delay for every evaluation of the loop
        '''
        try:
            self.oldLeftPhi = self.leftMotor.actualwheelphi
            self.leftMotor.wheelphi = self.oldLeftPhi
            self.oldRightPhi = self.rightMotor.actualwheelphi
            self.rightMotor.wheelphi = self.oldRightPhi
            loopCount = 0   # , sleepTimeCumul = 0, 0
            deltaPhiadd, deltaXadd, deltaYadd = 0, 0, 0
            deltaLeftPhi, deltaRightPhi = 0, 0
            # cumulLoopTime = 0

            t = threading.currentThread()
            while getattr(t, "live", True):
                loopCount += 1
                startTime = time.time()
                # Retrieve delta theta since last loop.
                deltaLeftPhi = self.leftMotor.actualwheelphi - self.oldLeftPhi
                self.oldLeftPhi += deltaLeftPhi
                self.leftMotor.wheelphi = self.oldLeftPhi
                deltaRightPhi = self.rightMotor.actualwheelphi - self.oldRightPhi
                self.oldRightPhi += deltaRightPhi
                self.rightMotor.wheelphi = self.oldRightPhi
                # Calculate the 6 dimensional twist of displacement
                self.Vb6 = self.F6@[deltaLeftPhi, deltaRightPhi]
                wbz, vbx, vby = [self.Vb6[2], self.Vb6[3], self.Vb6[4]]
                # Express the twist in a transformation matrix Tbb'
                if wbz == 0:
                    deltaPhib = 0
                    deltaXb = vbx
                    deltaYb = vby
                else:
                    deltaPhib = wbz
                    deltaXb = (vbx * sin(wbz) + vby * (cos(wbz) - 1)) / wbz
                    deltaYb = (vby * sin(wbz) + vbx * (1 - cos(wbz))) / wbz
                deltaQb = np.array([deltaPhib, deltaXb, deltaYb])
                deltaQ = np.array([[1,                    0,                     0],
                                   [0, cos(self.currentPhi), -sin(self.currentPhi)],
                                   [0, sin(self.currentPhi),  cos(self.currentPhi)]])@deltaQb
                self.currentPhi += deltaQ[0]
                deltaPhiadd += deltaQ[0]
                self.currentX += deltaQ[1]
                deltaXadd += deltaQ[1]
                self.currentY += deltaQ[2]
                deltaYadd += deltaQ[2]
                sleepTime = max(0, loopDelay - (time.time() - startTime))
                # sleepTimeCumul += sleepTime
                # stopTime = time.time()
                # cumulLoopTime += stopTime - startTime
                time.sleep(sleepTime)
        except:
            log.error(LOGGER_BASE_ODOMETRY, f'Error : {traceback.print_exc()}')
            raise
        finally:
            log.warning(LOGGER_BASE_ODOMETRY, 'Shutting down odometry thread...')
