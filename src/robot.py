# flake8: ignore=E501
import asyncio
import concurrent.futures
import traceback
import modern_robotics as mr
from ev3dev2.motor import OUTPUT_A, OUTPUT_B, OUTPUT_C, OUTPUT_D, OUTPUT_E, OUTPUT_F, OUTPUT_G, OUTPUT_H, SpeedDPS, SpeedPercent   # noqa : F401
from ev3dev2.sensor.lego import TouchSensor, UltrasonicSensor, Sensor # noqa : F401
from ev3dev2.port import LegoPort
from ev3dev2.sensor import INPUT_1, INPUT_2, INPUT_3, INPUT_4, INPUT_5, INPUT_6, INPUT_7, INPUT_8  # noqa : F401
from ev3dev2.power import PowerSupply
from logger import RoboLogger
import numpy as np
import queue
# import logging
import functools
import time
import os
import json
import paho.mqtt.client as mqtt
from arm import Arm
from diffdrivebase import DiffDriveBase
from numpy import pi, radians, degrees, sin, cos
import threading
from constant import LEGO_ROBOT_DEFINITIONS, LOW_VOLTAGE_THRESHOLD, \
    MOVEMENT_TIME_SCALING_METHOD_DEFAULT, MAX_WORKER_THREADS, DOUBLECLICK
from exceptions import LowVoltageException
from constant import \
    STOP_ACTION_BRAKE, \
    STOP_ACTION_COAST, \
    STOP_ACTION_HOLD            # noqa F401
from constant import LOGGER_DDR_MAIN, LOGGER_DDR_RUN, \
    LOGGER_DDR_THREADIMPLEVENTLOOP, LOGGER_DDR_IMPLMOVELOOP, \
    LOGGER_DDR_VOLTAGECHECK, LOGGER_DDR_ODOMETRY, \
    LOGGER_DDR_EXECUTORKILLSWITCH, LOGGER_DDR_KILLSWITCH, \
    LOGGER_DDR_THREADIMPLMQTT

log = RoboLogger.getLogger()


class DiffDriveManipulatorRobot(object):
    """
    DiffDrive main robot class. Launching a few threads to handle mechanics
    and odometry.
    """

    __slots__ = [
        '_urgentStop',
        'timeStep',
        'logger',
        '_current_V',
        'baseMovementLock',
        'armMovementLock',
        'powerSupply',
        'eventLoop',
        'eventLoopExecutors',
        'eventLoopArmExecutors',
        'eventLoopMainExecutors',
        'eventLoopBaseExecutors',
        'sensors',
        'gyroSensorConfiguration',
        'robotConfiguration',
        'arm',
        'base',
        'mqtt_topics',
        'mqtt_connect_mid',
        'threadOdometry',
        'eventLoopThread',
        'threadMQTT',
        'mqttMoveQueue',
        'mqttClient',
        'exceptionQueue',
        'asyncVoltageCheckLoopTask',
        'asyncOdometryReportingTask',
        'M0e',
        'Toe_grip',
        'Tbc',
        'Tcb',
        'asyncImplMoveLoopTask'
    ]

    def __init__(self,
                 robot_config_file,
                 initRobotConfig,
                 timeStep,
                 odometryTimer=0.1,
                 defaultLogging=30):
        """
        Creates the main bot class.
        """
        if not os.path.exists(robot_config_file):
            raise ValueError(f'Cannot find configuration file '
                             f'"{robot_config_file}"')

        with open(robot_config_file, 'r') as f:
            self.robotConfiguration = json.load(f)

        # Adjust JSON just read
        self.__adjustConfigDict(self.robotConfiguration)

        # Exception Queue for inter threads exception communications
        self.exceptionQueue = queue.Queue()
        self.mqttMoveQueue = queue.Queue()
        self.baseMovementLock = asyncio.locks.Lock()
        self.timeStep = timeStep
        self.powerSupply = PowerSupply()

        # region sensor Init
        self.sensors = {}
        for sensor in self.robotConfiguration["sensors"]:
            sensorDict = self.robotConfiguration["sensors"][sensor]
            legoPort = LegoPort(sensorDict["input"])
            # Applies to both I2C and SDK controled sensors
            if legoPort.mode != sensorDict["mode"]:
                legoPort.mode = sensorDict["mode"]
            # Only applies to sensors controled by the python SDK
            if "set_device" in sensorDict:
                legoPort.set_device = sensorDict["set_device"]
                time.sleep(1)
            address = sensorDict["input"]
            self.sensors[sensor] = sensorDict["sensorType"](address=address)
        # endregion

        # base Init
        self.base = DiffDriveBase(self.robotConfiguration["base"],
                                  initRobotConfig, odometryTimer)

        # arm/endeffector Init
        self.arm = Arm(armLinkDefinitions=self.robotConfiguration["arm"])

        # Other matrix init
        self.M0e = mr.RpToTrans(
            R=np.array(
                self.robotConfiguration["arm"]["arm1"]["manipulatorConfiguration"]["M0e"]["rotationMatrix"]).T,   # noqa F501
            p=self.robotConfiguration["arm"]["arm1"]["manipulatorConfiguration"]["M0e"]["p"])   # noqa F501
        self.Toe_grip = mr.RpToTrans(
            R=np.array(
                self.robotConfiguration["arm"]["arm1"]["manipulatorConfiguration"]["Toe_grip"]["rotationMatrix"]).T,   # noqa F501
            p=self.robotConfiguration["arm"]["arm1"]["manipulatorConfiguration"]["Toe_grip"]["p"])    # noqa F501
        self.Tbc = mr.RpToTrans(
            R=np.array(
                self.robotConfiguration["cameraConfiguration"]["Tbc"]["rotationMatrix"]).T,   # noqa F501
            p=self.robotConfiguration["cameraConfiguration"]["Tbc"]["p"])
        self.Tcb = mr.TransInv(self.Tbc)

        # Reset all motors at init
        self.urgentStop = False
        log.info(LOGGER_DDR_MAIN, 'Done main robot init.')
        # log.setLevel(LOGGER_DDR_MAIN)

    def __del__(self):
        """ Destructor """
        loggerName = 'ddrmain'
        log.info(loggerName,
                 msg=f'Deleting robot ; opening gripper '
                     f'and shutting down motors.')
        # Open the gripper
        self.arm.openEndEffector()
        # Shut down the motors
        self.urgentStop = True
        # Clear the event loop and shut it down
        self.asyncVoltageCheckLoopTask.cancel()
        self.asyncImplMoveLoopTask.cancel()
        # Cancel running tasks in the event loop
        if self.eventLoop.is_running():
            self.eventLoop.stop()
        # Stop threads
        self.base.stopThreadOdometry()
        # Will release motors that are on hold before shutting down
        self.base.resetBodyConfiguration()
        # Stop the mqtt client loop
        self.mqttClient.loop_stop()
        self.arm.killThread = True
        self.arm.openEndEffector()
        # Unload the drivers for sensors
        for sensor in self.robotConfiguration["sensors"]:
            LegoPort(
                self.robotConfiguration["sensors"][sensor]['input']).mode = \
                'none'
        log.debug(loggerName, 'Done the __del__ function')
        # Unload the sensor drivers

    # region properties
    @property
    def currentVoltage(self):
        return self.powerSupply.measured_volts

    @property
    def urgentStop(self):
        return self._urgentStop

    @urgentStop.setter
    def urgentStop(self, value):
        if value is True:
            log.info(LOGGER_DDR_MAIN, "Need to immediately stop the bot.")
            self.mqttMoveQueue = queue.Queue()
        self._urgentStop = value
        if self.base:
            self.base.urgentStop = value
        if self.arm:
            self.arm.urgentStop = value

    @property           # should stay on main robot
    def currentConfigVector(self):
        return np.r_[self.base.currentConfigVector,
                     self.arm.currentConfigVector,
                     self.arm.endEffectorStatus]
    # endregion

    def run(self):
        """
        run the main event loop for the robot and the required threads
        """
        try:
            # region Start Base Odometry Thread Init
            self.base.run()
            # endregion

            # region MQTTConnect Thread Init
            self.threadMQTT = threading.Thread(
                target=self.threadImplMQTT,
                name="threadImplMQTT")
            self.threadMQTT.start()
            # endregion

            # Launch main event loop in a separate thread
            self.eventLoopThread = threading.Thread(
                target=self.threadImplEventLoop,
                name="threadImplEventLoop")
            self.eventLoopThread.start()
            # Error handling for threads
            while True:
                try:
                    exc = self.exceptionQueue.get(block=True)
                except queue.Empty:
                    pass
                else:
                    log.error(LOGGER_DDR_RUN,
                              msg=f'Exception handled in one of the '
                                  f'spawned process : {exc}')
                    raise exc
                self.eventLoopThread.join(0.2)
                if self.eventLoopThread.isAlive():
                    continue
                else:
                    break
        except SystemExit:
            # raise the exception up the stack
            raise
        except:
            log.error(LOGGER_DDR_RUN, f'Error : {traceback.print_exc()}')

    def threadImplEventLoop(self):
        """
        Main event asyncio eventloop launched in a thread
        """
        try:
            log.warning(LOGGER_DDR_THREADIMPLEVENTLOOP,
                        msg=f'Launching main event loop.')
            self.eventLoop = asyncio.new_event_loop()
            self.eventLoopArmExecutors = \
                concurrent.futures.ThreadPoolExecutor(
                    max_workers=MAX_WORKER_THREADS,
                    thread_name_prefix='ArmThreadPool')
            self.eventLoopMainExecutors = \
                concurrent.futures.ThreadPoolExecutor(
                    max_workers=MAX_WORKER_THREADS,
                    thread_name_prefix='MainThreadPool')
            self.eventLoopBaseExecutors = \
                concurrent.futures.ThreadPoolExecutor(
                    max_workers=MAX_WORKER_THREADS,
                    thread_name_prefix='BaseThreadPool')
            self.asyncImplMoveLoopTask = \
                self.eventLoop.create_task(
                    self.asyncProcessMQTTMessages(0.25))
            self.asyncVoltageCheckLoopTask = \
                self.eventLoop.create_task(self.asyncVoltageCheckLoop(10))
            self.asyncOdometryReportingTask = \
                self.eventLoop.create_task(self.asyncOdometryReporting(1))
            self.eventLoop.run_in_executor(
                self.eventLoopMainExecutors,
                self.asyncExecutorImplKillSwitch)
            self.eventLoop.run_forever()
        except:
            log.error(LOGGER_DDR_THREADIMPLEVENTLOOP,
                      msg=f'Error : {traceback.print_exc()}')
        finally:
            self.eventLoop.stop()
            self.eventLoop.close()

    async def asyncProcessMQTTMessages(self, loopDelay):
        """
        This function receives the messages from MQTT to move the robot.
        It is implemented in another thread so that the killswitch can kill
        the thread without knowing which specific
        """
        while True:
            try:
                await asyncio.sleep(loopDelay)
                if self.mqttMoveQueue.empty() is False:
                    try:
                        if self.currentVoltage < LOW_VOLTAGE_THRESHOLD:
                            raise LowVoltageException
                    except LowVoltageException:
                        log.warning(LOGGER_DDR_THREADIMPLMQTT,
                                    msg=f'Low voltage threshold '
                                        f'{float(LOW_VOLTAGE_THRESHOLD):.2} '
                                        f'reached. Current voltage = '
                                        f'{self.currentVoltage:.2f} .'
                                        f'Robot will not move, but MQTT '
                                        f'message will attempt to process. '
                                        f'Consider charging the battery, '
                                        f'or find some kind of power supply')
                    # Remove the first in the list, will pause until there
                    # is something
                    currentMQTTMoveMessage = self.mqttMoveQueue.get()
                    # Decode message received
                    msgdict = json.loads(
                        currentMQTTMoveMessage.payload.decode('utf-8'))
                    # Verify if need to urgently overried current movement
                    # and queue
                    if 'override' in msgdict:
                        if msgdict['override']:
                            log.debug(LOGGER_DDR_IMPLMOVELOOP,
                                      msg=f'Overriding all motion - calling '
                                          f'killswitch to empty queue')
                            # Urgently stop the current movement (and empty
                            # movement queue - done in killSwitch)
                            self.killSwitch()

                    # Default motion configuration
                    pid = False if 'pid' not in msgdict else msgdict["pid"]
                    velocity_factor = 0.1 if 'velocity_factor' not \
                        in msgdict else msgdict["velocity_factor"]
                    dryrun = False if 'dryrun' not in msgdict else \
                        msgdict["dryrun"]
                    inter_wheel_distance = None if 'inter_wheel_distance' not\
                        in msgdict else msgdict["inter_wheel_distance"]
                    wheel_radius = None if 'wheel_radius' not in msgdict \
                        else msgdict["wheel_radius"]
                    time_scaling_method = MOVEMENT_TIME_SCALING_METHOD_DEFAULT

                    if currentMQTTMoveMessage.topic == 'bot/killSwitch':
                        self.killSwitch()

                    elif currentMQTTMoveMessage.topic == \
                            'bot/base/reset/position':
                        log.warning(
                            LOGGER_DDR_IMPLMOVELOOP,
                            msg=f'Invoking reset body configuration')
                        self.base.resetBodyConfiguration()

                    elif currentMQTTMoveMessage.topic == \
                            'bot/base/move/xyphi':
                        self.eventLoop.run_in_executor(
                            self.eventLoopBaseExecutors,
                            functools.partial(self.base.moveBaseXYPhi,
                                              endBaseConfig=(msgdict["x"],
                                                             msgdict["y"],
                                                             msgdict["phi"]),
                                              pid=pid,
                                              velocity_factor=velocity_factor,
                                              dryrun=dryrun))

                    elif currentMQTTMoveMessage.topic == \
                            'bot/base/move/x':
                        self.eventLoop.run_in_executor(
                            self.eventLoopBaseExecutors,
                            functools.partial(self.base.moveBaseX,
                                              distance=msgdict["distance"],
                                              pid=pid,
                                              velocity_factor=velocity_factor,
                                              wheel_radius=wheel_radius,
                                              dryrun=dryrun))

                    elif currentMQTTMoveMessage.topic == \
                            'bot/base/turn/phi':
                        self.eventLoop.run_in_executor(
                            self.eventLoopBaseExecutors,
                            functools.partial(
                                self.base.rotateBasePhi,
                                phi=msgdict["phi"],
                                pid=pid,
                                velocity_factor=velocity_factor,
                                inter_wheel_distance=inter_wheel_distance,
                                dryrun=dryrun))

                    elif currentMQTTMoveMessage.topic == \
                            'bot/base/move/home':
                        # Calculate where to go from there (not avoiding
                        # obstacles...)
                        _, (x, y, _) = mr.TransToRp(
                            mr.TransInv(self.base.Tsb))
                        # pi - self.base.currentPhi
                        phi = - 1 * self.base.currentPhi
                        self.base.moveBaseXYPhi(
                            endBaseConfig=(x, y, phi),
                            pid=pid,
                            velocity_factor=velocity_factor,
                            time_scaling_method=time_scaling_method,
                            dryrun=dryrun)

                    elif currentMQTTMoveMessage.topic == \
                            'bot/arm/move/rest':
                        self.eventLoop.run_in_executor(
                            self.eventLoopArmExecutors,
                            functools.partial(
                                self.arm.moveToAngle,
                                armConfigVector=self.arm.armRestPosition,
                                dryrun=dryrun))

                    elif currentMQTTMoveMessage.topic == \
                            'bot/arm/move/zero':
                        self.eventLoop.run_in_executor(
                            self.eventLoopArmExecutors,
                            functools.partial(
                                self.arm.moveToAngle,
                                armConfigVector=self.arm.armZeroPosition,
                                dryrun=dryrun))

                    elif currentMQTTMoveMessage.topic == \
                            'bot/arm/move/theta':
                        self.eventLoop.run_in_executor(
                            self.eventLoopArmExecutors,
                            functools.partial(
                                self.arm.moveToAngle,
                                armConfigVector=[msgdict["theta1"],
                                                 msgdict["theta2"]],
                                dryrun=dryrun))

                    elif currentMQTTMoveMessage.topic == \
                            'bot/gripper/open':
                        # Already an async call (non blocking)
                        self.arm.openEndEffector()

                    elif currentMQTTMoveMessage.topic == \
                            'bot/gripper/close':
                        # Already an async call (non blocking)
                        self.arm.closeEndEffector()

                    elif currentMQTTMoveMessage.topic == \
                            'bot/gripper/position':
                        # Call function to move this thing
                        Pco = np.array(msgdict["P"])
                        angle = msgdict["grip_angle"]
                        Toe_grip = mr.RpToTrans(
                            R=np.array([[cos(angle), 0, sin(angle)],
                                        [0, 1, 0],
                                        [-sin(angle), 0, cos(angle)]]),
                            p=[0, 0, 0])
                        Rco, _ = mr.TransToRp(Toe_grip)  # Rco=camera-object
                        # Rco, _ = mr.TransToRp(self.Toe_grip)
                        Tco_desired = mr.RpToTrans(R=Rco, p=Pco)
                        Tbo_desired = self.Tbc @ Tco_desired
                        # Initial guess - based on the angle received
                        thetalist0 = (0, 0, pi / 8, pi / 4)   # pi / 4, pi / 2)

                        log.debug(LOGGER_DDR_IMPLMOVELOOP,
                                  msg=f'Running inverse kinematics to get '
                                      f'required joint angles.')
                        thetalist, success = mr.IKinBody(
                            Blist=self.arm.bList,
                            M=self.M0e,
                            T=Tbo_desired,
                            thetalist0=thetalist0,
                            eomg=0.02,   # 0.08 rad = 5 deg uncerainty
                            ev=0.025)     # 4cm error?
                        if success:
                            thetalist = thetalist.round(6)
                            log.debug(LOGGER_DDR_IMPLMOVELOOP,
                                      msg=f'IK Solved with joint thetas '
                                          f'found {thetalist}')
                            # 0. Open gripper
                            log.debug(LOGGER_DDR_IMPLMOVELOOP,
                                      msg=f'Opening or ensuring end effector '
                                          f'is opened.')
                            self.arm.openEndEffector()
                            # 1. Move arm to 0 position - launch async.
                            log.debug(LOGGER_DDR_IMPLMOVELOOP,
                                      msg=f'Moving arm to driving'
                                          f'position')
                            self.arm.moveToAngle(
                                armConfigVector=self.arm.armDrivePosition,
                                dryrun=dryrun)
                            # 2. Move base by X first
                            log.debug(LOGGER_DDR_IMPLMOVELOOP,
                                      msg=f'Moving body by X '
                                          f'{thetalist[0]:.2f}')
                            self.base.moveBaseX(
                                distance=thetalist[0],
                                pid=pid,
                                velocity_factor=velocity_factor,
                                dryrun=dryrun)
                            # 3. Rotate base by Phi
                            log.debug(LOGGER_DDR_IMPLMOVELOOP,
                                      msg=f'Rotating base by phi '
                                          f'{thetalist[1]:.2f}')
                            self.base.rotateBasePhi(
                                phi=thetalist[1],
                                pid=pid,
                                velocity_factor=velocity_factor,
                                dryrun=dryrun)
                            # 4a. Move to standoff position
                            log.debug(LOGGER_DDR_IMPLMOVELOOP,
                                      msg=f'Moving arm joints to standoff '
                                          f'position')
                            # 4. Move other joint in position - need to block
                            log.debug(LOGGER_DDR_IMPLMOVELOOP,
                                      msg=f'Moving arm joints into position '
                                          f'for picking up object j3 = '
                                          f'{degrees(thetalist[2]):.2f}, '
                                          f'j4 = {degrees(thetalist[3]):.2f}')
                            self.arm.moveToAngle(armConfigVector=thetalist[2:],
                                                 dryrun=dryrun)
                            # 5. Grab object
                            log.debug(LOGGER_DDR_IMPLMOVELOOP,
                                      msg=f'Closing end effector')
                            self.arm.closeEndEffector()
                            # Rest b/c the previous call is non blocking,
                            # and blocking causes issues on the wait
                            log.debug(LOGGER_DDR_IMPLMOVELOOP,
                                      msg=f'Async Sleep for 1 second...')
                            await asyncio.sleep(1)
                            # 6. Set to driving position
                            log.debug(LOGGER_DDR_IMPLMOVELOOP,
                                      msg=f'Moving arm to driving position.')
                            self.arm.moveToAngle(
                                armConfigVector=self.arm.armDrivePosition,
                                dryrun=dryrun)
                            log.debug(LOGGER_DDR_IMPLMOVELOOP, f'Done!')
                        else:
                            log.warning(LOGGER_DDR_IMPLMOVELOOP,
                                        msg=f'Unable to calculate joint and '
                                            f'wheel angles to achieve the '
                                            f'required position.')

                    elif currentMQTTMoveMessage.topic == \
                            'bot/logger':
                        # Changing the logging level on the fly...
                        log.setLevel(msgdict['logger'], lvl=msgdict['level'])
                    elif currentMQTTMoveMessage.topic == \
                            'bot/logger/multiple':
                        # Changing the logging level on the fly for multiple loggers at a time
                        for logger, level in msgdict.items():
                            log.setLevel(logger, level)
                    else:
                        raise NotImplementedError
            except NotImplementedError:
                log.warning(LOGGER_DDR_IMPLMOVELOOP,
                            msg=f'MQTT message not implemented.')
            except asyncio.futures.CancelledError:
                log.warning(LOGGER_DDR_IMPLMOVELOOP,
                            msg=f'Cancelled the MQTT dequeing task.')
            except:
                log.error(LOGGER_DDR_IMPLMOVELOOP,
                          msg=f'Error : {traceback.print_exc()}')

    async def asyncVoltageCheckLoop(self, loopDelay):
        """
        run the voltage loop check
        """
        while True:
            try:
                await asyncio.sleep(loopDelay)
                if self.currentVoltage < LOW_VOLTAGE_THRESHOLD:
                    raise LowVoltageException
            except asyncio.futures.CancelledError:
                log.warning(LOGGER_DDR_VOLTAGECHECK,
                            msg=f'Cancelled the Voltage Check task.')
            except LowVoltageException:
                log.critical(LOGGER_DDR_VOLTAGECHECK,
                             msg=f'Voltage low on device '
                                 f'({self.currentVoltage:.2f}V - '
                                 f'threshold  = {LOW_VOLTAGE_THRESHOLD:.2f}V)'
                                 f' - consider charging battery or shutting '
                                 f'down.')

    async def asyncOdometryReporting(self, loopDelay):
        """
        run the odometry loop and printout
        """
        try:
            while True:
                await asyncio.sleep(loopDelay)
                log.info(LOGGER_DDR_ODOMETRY,
                         msg=f'Joint theta (degrees) = {self.arm}')
                log.info(LOGGER_DDR_ODOMETRY,
                         msg=f'Latest Vb6 = {self.base.Vb6} ; '
                             f'Phi(degrees) = '
                             f'{degrees(self.base.currentPhi):.2f}')
                log.info(LOGGER_DDR_ODOMETRY,
                         msg=f'Latest Base SE3 (self.base.Tsb) = \n'
                             f'{self.base.Tsb}')
                # Don't read actual position, there's a race condition
                left = self.base.oldLeftPhi
                right = self.base.oldRightPhi
                log.debug(LOGGER_DDR_ODOMETRY,
                          msg=f'Left/Right Position:{left:.2f}/{right:.2f}')
                theoretical_dist = radians((left + right) / 2) * \
                    self.base.wheelRadius
                log.debug(LOGGER_DDR_ODOMETRY,
                          msg=f'Theoretical distance:{theoretical_dist:.4f}')

        except asyncio.futures.CancelledError:
            log.warning(LOGGER_DDR_ODOMETRY,
                        msg=f'Cancelled the Voltage Check task.')

    def asyncExecutorImplKillSwitch(self):
        """ Detects click and double clicks"""
        try:
            button = self.sensors["killSwitch"]
            log.info(LOGGER_DDR_EXECUTORKILLSWITCH,
                     msg=f'Initialized async process for kill switch.')
            while True:
                firstClick = time.time()
                button.wait_for_bump()
                secondClick = time.time()
                if secondClick - firstClick < DOUBLECLICK:
                    log.warning(LOGGER_DDR_EXECUTORKILLSWITCH,
                                msg=f'DoubleClick caught.')
                    self.exceptionQueue.put(SystemExit)
                    # Return is there to end this thread and return it
                    # to the pool (run_in_executor).
                    return
                else:
                    self.killSwitch()
                log.warning(LOGGER_DDR_EXECUTORKILLSWITCH,
                            msg=f'Button pressed. Urgent Stop switch '
                                f'flipped. Threads using motors should '
                                f'stop.')
        except:
            log.warning(LOGGER_DDR_EXECUTORKILLSWITCH,
                        msg=f'Error in the kill switch routing : '
                            f'{traceback.print_exc()}')

    def __adjustConfigDict(self, confDict):
        '''
        adjustConfigDict :
        param:confDict:parameters read in the config file for the robot
        param:confDict:type:parameter dictionary
        returns : modified confDict
        '''
        for key, value in confDict.items():
            if not isinstance(value, dict):
                if not isinstance(value, list):
                    if value in LEGO_ROBOT_DEFINITIONS:
                        confDict[key] = LEGO_ROBOT_DEFINITIONS[value]
            else:
                confDict[key] = self.__adjustConfigDict(confDict[key])
        return confDict

    def killSwitch(self):
        # Empty movement queue
        try:
            log.warning(LOGGER_DDR_KILLSWITCH, 'Killing all motor motion.')
            self.urgentStop = True
            time.sleep(1)
            self.urgentStop = False
        except:
            pass

    def threadImplMQTT(self):
        """
        MQTT Thread launching the loop and subscripbing to the right topics
        """
        mqtt_default_qos = 2
        self.mqtt_topics = [
            (topic, mqtt_default_qos) for topic in
            self.robotConfiguration["mqtt"]["subscribedTopics"]]

        def on_connect(client, userdata, flags, rc):
            log.warning(LOGGER_DDR_THREADIMPLMQTT,
                        msg=f'Connected to MQTT broker. '
                            f'Result code {rc}')
            mqtt_connect_result, self.mqtt_connect_mid = \
                client.subscribe(self.mqtt_topics)
            if mqtt_connect_result == mqtt.MQTT_ERR_SUCCESS:
                log.warning(LOGGER_DDR_THREADIMPLMQTT,
                            msg=f'Successfully subscribed '
                                f'to {self.mqtt_topics}')
            else:
                log.error(LOGGER_DDR_THREADIMPLMQTT,
                          msg=f'MQTT Broker subscription problem.')

        def on_message(client, userdata, message):
            """ callback function used for the mqtt client (called when
            a new message is publisehd to one of the queues we subscribe to)
            """
            log.info(LOGGER_DDR_THREADIMPLMQTT,
                     msg=f'Received MID {message.mid} : '
                         f'{str(message.payload)} on topic '
                         f'{message.topic} with QoS {message.qos}')
            self.mqttMoveQueue.put_nowait(message)

        def on_disconnect(client, userdata, rc=0):
            """callback for handling disconnects
            """
            log.info(LOGGER_DDR_THREADIMPLMQTT,
                     msg=f'Disconnected MQTT result code = {rc}.'
                         f'Should automatically re-connect to broker')

        def on_subscribe(client, userdata, mid, granted_qos):
            if mid == self.mqtt_connect_mid:
                log.warning(LOGGER_DDR_THREADIMPLMQTT,
                            msg=f'Subscribed to topics. Granted QOS '
                                f'= {granted_qos}')
            else:
                log.error(LOGGER_DDR_THREADIMPLMQTT,
                          msg=f'Strange... MID doesn\'t match '
                              f'self.mqtt_connect_mid')

        self.mqttClient = mqtt.Client(
            client_id="mybot",
            clean_session=True,
            transport=self.robotConfiguration["mqtt"]["brokerProto"])
        self.mqttClient.enable_logger(
            logger=RoboLogger.getSpecificLogger(LOGGER_DDR_THREADIMPLMQTT))
        self.mqttClient.on_subscribe = on_subscribe
        self.mqttClient.on_connect = on_connect
        self.mqttClient.on_disconnect = on_disconnect
        self.mqttClient.on_message = on_message
        self.mqttClient.connect(
            host=self.robotConfiguration["mqtt"]["brokerIP"],
            port=self.robotConfiguration["mqtt"]["brokerPort"])
        self.mqttClient.loop_start()
