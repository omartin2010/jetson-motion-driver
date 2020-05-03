# flake8: ignore=E501
from ev3dev2.motor import LargeMotor, OUTPUT_A, OUTPUT_B, OUTPUT_C, OUTPUT_D, OUTPUT_E, OUTPUT_F, OUTPUT_G, OUTPUT_H, SpeedDPS, SpeedPercent   # noqa : F401
from logger import RoboLogger
import numpy as np
from numpy import radians, degrees
import threading
import modern_robotics as mr
import time
from constant import MOTOR_RAMP_UP_TIME, MOVEMENT_TIME_SCALING_METHOD_CUBIC, MOVEMENT_TIME_SCALING_METHOD_QUINTIC, MOVEMENT_TIME_SCALING_METHOD_DEFAULT   # noqa : F401
from constant import LOGGER_MOTOR_MAIN, LOGGER_MOTOR_OTP, LOGGER_MOTOR_OTPPID

log = RoboLogger.getLogger()


class MyMotor(LargeMotor):
    """
    EV3/NXT large servo motor class override for RoboTeam - includes methods specific to the robot
    and more generic like the PID controler in the on_to_position_pid
    """

    __slots__ = [
        'ki',
        'kp',
        'description',
        'wheelphi',
        'urgentstop',
        'timeStep',
        'defaultStopAction'
    ]

    def __init__(self,
                 address=None,
                 description=None,
                 ki=0.75,
                 kp=0.75,
                 rampUp=MOTOR_RAMP_UP_TIME,
                 rampDown=MOTOR_RAMP_UP_TIME,
                 defaultStopAction=None,
                 timeStep=0.25,
                 **kwargs):
        LOGGER_MOTOR_MAIN_INSTANCE = LOGGER_MOTOR_MAIN.format(address)
        super(MyMotor, self).__init__(address, **kwargs)
        log.info(LOGGER_MOTOR_MAIN_INSTANCE, 'Initializing motor')
        self.description = description
        self.ki = ki
        self.kp = kp
        self.wheelphi = 0
        self.timeStep = timeStep
        self.urgentstop = False
        if defaultStopAction is None:
            defaultStopAction = self.STOP_ACTION_COAST
        self.defaultStopAction = defaultStopAction
        # Init
        self.stop(stop_action=self.defaultStopAction)
        log.info(LOGGER_MOTOR_MAIN_INSTANCE, 'Exit init motor class')

# region properties
    @property
    def actualwheelphi(self):
        return radians(self.position / self.count_per_rot * 360)

    @property
    def urgentStop(self):
        return self._urgentStop

    @urgentStop.setter
    def urgentStop(self, value):
        if value:
            self.motor.stop()
# endregion

    def on_to_position(self,
                       currentphi_func,
                       targetphi,
                       pid=False,
                       timestep=None,
                       method=MOVEMENT_TIME_SCALING_METHOD_QUINTIC,
                       speedpercent=5,
                       stop_action=None,
                       dryrun=False):
        '''
        Thread launcher for PID controler for motor.
        input:
            currentphi : float :                current angle in degrees (can't obtain it from motor as it's unsafe to read motor position)
            targetphi :   float :               angle in degrees to move
            pid : bool                          use PID controlled motion if true, on_to_position if not.
            speedpercent : float :              value between ]0, 100]. 100=100% of max speed, 0=0%
            timestep : float :                  timestep to be used to create the list of positions. Defaults to self.timeStep - required for PID motion
            stop_action : bool :                      if True, motor stop action=holds. won't move after
            # block : bool :                      if True, function won't return until motor reaches target position
            dryrun : bool :                     if True, won't move
        returns:
            returns thread so it can be joined from caller
        '''
        if stop_action is None:
            stop_action = self.STOP_ACTION_COAST

        if timestep is None:
            timestep = self.timeStep

        targetThreadFunc = self.__on_to_position_pid if pid else self.__on_to_position
        # build description including motor function if required to display thread name with human understandable info
        desc = '' if not self.description else '-' + self.description
        threadThreadName = f'PID-Motor-{self.address}{desc}' if pid else f'Motor-{self.address}{desc}'
        if pid:
            thread_move_position = threading.Thread(target=targetThreadFunc,
                                                    args=([
                                                        currentphi_func,
                                                        targetphi,
                                                        timestep,
                                                        method,
                                                        speedpercent,
                                                        stop_action,
                                                        dryrun
                                                    ]),
                                                    name=threadThreadName)
        else:
            thread_move_position = threading.Thread(target=targetThreadFunc,
                                                    args=([
                                                        currentphi_func,
                                                        targetphi,
                                                        speedpercent,
                                                        stop_action,
                                                        dryrun
                                                    ]),
                                                    name=threadThreadName)
        return thread_move_position

    def __on_to_position(self,
                         currentphi_func,
                         targetphi,
                         speedpercent=5,
                         stop_action=None,
                         dryrun=False):
        '''
        Moves the motor to the specified target. Nothing specific is made in the code
        to ensure the target position is indeed reached.
        input:
            currentphi_func : function that connects to the current angle of the motor in RADIANS!!!! other param in degrees
            targetphi :   float :                     angle in degrees to move
            speedpercent : float :              value between [-100, 0[ or ]0, 100]. 100 = 100% of max speed for motor, 0 = 0% (doesn't
                                                support 0.
            stop_action : str :                 different stop action after reached target (Motor.STOP_ACTION_COAST, ..)
            dryrun : bool :                     if True, won't move
        returns:
            no value is returned
        '''
        LOGGER_MOTOR_OTP_INSTANCE = LOGGER_MOTOR_OTP.format(self.address)
        if self.urgentstop:
            log.critical(LOGGER_MOTOR_OTP_INSTANCE, f'Initiating urgent stop of motor {self.address}.')
            self.shutOffBaseMotors(gripper=True)
            log.critical(LOGGER_MOTOR_OTP_INSTANCE, f'Urgent stop of motor {self.address} completed.')
            return None

        if stop_action is None:
            stop_action = self.STOP_ACTION_COAST

        log.debug(LOGGER_MOTOR_OTP_INSTANCE, f'Moving motor {self.address} to position {targetphi} from position {int(degrees(currentphi_func()))}')
        # Setting up ramp up and down time based on constants

        # Ensure speedpercent is -100, 100
        speedpercent = np.clip(speedpercent, -100, 100)
        if speedpercent != 0 and not dryrun:
            log.debug(LOGGER_MOTOR_OTP_INSTANCE, f'Launching motor {self.address} at speedpercent = {speedpercent}.')
            direction = 1 if targetphi > int(degrees(currentphi_func())) else -1
            super(MyMotor, self).on_to_position(SpeedPercent(speedpercent), targetphi, brake=True, block=False)
            self.stop_action = stop_action
        elif dryrun:
            log.info(LOGGER_MOTOR_OTP_INSTANCE, f'Not moving motor {self.address} : dry run')
            return
        elif speedpercent == 0:
            log.info(LOGGER_MOTOR_OTP_INSTANCE, f'Not running motor {self.address} at speed {speedpercent:.2f}. Invalid speed.')
            return
        # Wait until not moving...
        moving = True
        nonMoveCount = 0
        log.debug(LOGGER_MOTOR_OTP_INSTANCE, f'Monitoring for motor {self.address} motion...')
        while moving:
            # Check if we change position between now and 1 second...
            curPos = int(degrees(currentphi_func()))
            sleepTime = 0.1
            log.debug(LOGGER_MOTOR_OTP_INSTANCE, f'Waiting {sleepTime:.2f}s before evaluating motor {self.address}\'s position. curpos = {curPos}, target = {targetphi}')
            time.sleep(sleepTime)
            nextPos = int(degrees(currentphi_func()))
            log.debug(LOGGER_MOTOR_OTP_INSTANCE, f'Motor {self.address} nextPos = {nextPos}')
            if (nonMoveCount > 3) or (nextPos >= targetphi and direction == 1) or \
                                     (nextPos <= targetphi and direction == -1):   # curPos:
                log.debug(LOGGER_MOTOR_OTP_INSTANCE, f'Motor reached target; nextpos = curpos = {nextPos} ==> nonMoveCount = {nonMoveCount}.')
                moving = False
                # Validate if we reached the target or if motor is blocked...
                if curPos == targetphi:
                    log.debug(LOGGER_MOTOR_OTP_INSTANCE, f'Motor {self.address} has stopped moving at target (curPos = nextPos = targetphi = {targetphi}). About to exit the function (and thread)')
                else:
                    log.debug(LOGGER_MOTOR_OTP_INSTANCE, f'Motor {self.address} is stopped before reaching target. Stopping motor now.')
                    super(MyMotor, self).off()
                    log.debug(LOGGER_MOTOR_OTP_INSTANCE, f'Motor {self.address} has been powered off. CurPos = nextPos = {curPos} ; target = {targetphi} ; gap = {targetphi-curPos}).')
                    log.debug(LOGGER_MOTOR_OTP_INSTANCE, f'About to exit the function (and thread)')
            if nextPos == curPos:
                nonMoveCount += 1

    def __on_to_position_pid(self,
                             currentphi_func,
                             targetphi,
                             timestep,
                             method=MOVEMENT_TIME_SCALING_METHOD_QUINTIC,
                             speedpercent=5,
                             stop_action=None,
                             dryrun=False):
        '''
        Threaded implementation of PID controler for motor.
            param:currentphi_func:desc:current angle in degrees (can't obtain it from motor as it's unsafe to read motor position)
            param:currentphi_func:type:float
            param:targetphi:desc:angle in degrees to move
            param:targetphi:type:float
            param:speedpercent:desc:value between ]0, 100]. 100 = 100% of max speed, 0 = 0%
            param:speedpercent:float
            param:stop_action:desc:see values in LargeMotor(Motor)
            param:stop_action:type:str
            param:dryrun:desc:if True, won't move
            param:dryrun:type:bool
        returns:
            no value is returned
        '''
        LOGGER_MOTOR_OTPPID_INSTANCE = LOGGER_MOTOR_OTPPID.format(self.address)
        wiggle = 1   # tolerated degrees for motor final position
        # 0. Check for urgent stop
        if self.urgentstop:
            log.critical(LOGGER_MOTOR_OTPPID_INSTANCE, "Initiating urgent stop of base motors.")
            self.off()
            log.critical(LOGGER_MOTOR_OTPPID_INSTANCE, "Urgent stop of base motors completed.")
            return

        # 1. Estimate time to target:
        theta = currentphi_func
        eta = abs((targetphi - degrees(theta())) / self.max_dps / (speedpercent / 100) * 2)  # /2 accounts for ramp up and down - it's approximate

        # 2. find trajectory in joint/wheel angle space:
        log.info(LOGGER_MOTOR_OTPPID_INSTANCE, 'Figuring out trajectory in joint space')
        trajectory = mr.JointTrajectory(thetastart=[degrees(theta())],
                                        thetaend=[targetphi],
                                        Tf=eta,
                                        N=max(2, int(eta / timestep)),
                                        method=method)

        # 3. estimate speed for next segment
        idx = 0
        adj_step_count = 0
        while not (targetphi - wiggle <= degrees(theta()) and degrees(theta()) <= targetphi + wiggle):
            tic = time.time()
            theta_desired = trajectory[min(idx, len(trajectory) - 1)]
            theta_desired_next = trajectory[min(idx + 1, len(trajectory) - 1)]
            if self.urgentstop:
                log.critical(LOGGER_MOTOR_OTPPID_INSTANCE, 'Stopping motion. Urgent stop called.')
                break
            # Get current error
            theta_err = theta_desired - degrees(theta())
            # Desired Speed (based on ideal trajectory)
            theta_dot_desired = (theta_desired_next - theta_desired) / timestep  # will be 0 after initial trajectory
            # Actual PID controler on speed
            theta_dot = int(theta_dot_desired + self.kp * theta_err + self.ki * theta_err * timestep)
            # Clip the speed if outside of boundaries
            theta_dot = np.clip(theta_dot, -self.max_speed, self.max_speed)
            # Do not run unless it's not a dryrun and speed > 0
            if not dryrun:
                if theta_dot != 0:
                    log.debug(LOGGER_MOTOR_OTPPID_INSTANCE, f'Setting speed to {theta_dot} for motor')
                    self.on(SpeedDPS(theta_dot), brake=True, block=False)
                else:
                    log.debug(LOGGER_MOTOR_OTPPID_INSTANCE, "Step {0} - Not moving motor {1} : set point = {2}".format(idx, self.kwargs['address'], theta_dot))
            else:
                log.debug(LOGGER_MOTOR_OTPPID_INSTANCE, "Step {0} - Not moving motor {1} : dry run".format(idx, self.kwargs['address']))
            stepDuration = time.time() - tic
            if idx % 5 == 0:
                if adj_step_count > 0:
                    log.debug(LOGGER_MOTOR_OTPPID_INSTANCE, "Step {0} - Duration of loop = {1:.2f}. Sleeping {2:.2f}s \
                           until next step. TimeStep = {3:.2f}. Position/Target = {4:.2f}/{5:.2f}".format(idx + 1,
                                                                                                          stepDuration,
                                                                                                          timestep - stepDuration,
                                                                                                          timestep,
                                                                                                          degrees(theta()),
                                                                                                          targetphi))
                else:
                    log.debug(LOGGER_MOTOR_OTPPID_INSTANCE, "Fine tuning step {0} - Duration of loop = {1:.2f}. Sleeping {2:.2f}s \
                          until next step. TimeStep = {3:.2f}. Position/Target = {4:.2f}/{5:.2f}".format(adj_step_count,
                                                                                                         stepDuration,
                                                                                                         timestep - stepDuration,
                                                                                                         timestep,
                                                                                                         degrees(theta()),
                                                                                                         targetphi))

            time.sleep(max(0, timestep - stepDuration))
            idx += 1
            if idx > len(trajectory):
                adj_step_count += 1
        self.stop(stop_action=stop_action)
        return
