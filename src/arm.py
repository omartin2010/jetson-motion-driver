# flake8: ignore=E501
from logger import RoboLogger
import threading
import traceback
from armlink import ArmLink
from endeffector import EndEffector
from numpy import radians
import time
import numpy as np
from constant import STOP_ACTION_BRAKE, STOP_ACTION_COAST, STOP_ACTION_HOLD   # noqa: F401
from constant import LOGGER_ARM_EE, LOGGER_ARM_MAIN, LOGGER_ARM_MTA, LOGGER_ARM_RESET

log = RoboLogger.getLogger()


class Arm(object):
    """
    Implementation of an arm (that contains a collection of links)
    """

    __slots__ = [
        '__links',
        'stopAction',
        'lock',
        '_urgentStop',
        '_killThread',
        '__endEffector',
        '__motionThreads',
        'armZeroPosition',
        'armRestPosition',
        'armDrivePosition'
    ]

    def __init__(self,
                 armLinkDefinitions,
                 armName="arm1"):
        '''
        Description:constructor for the robot arm
        param:armLinkDefinitions:dictionary defining the links'
        param:armLinkDefinitions:type:dictionary (see robot-def-??.json)
        '''
        self.lock = threading.Lock()
        self.__links = []
        self.__motionThreads = []
        self.killThread = False
        for _, link in sorted(armLinkDefinitions[armName]["links"].items()):
            if "virtual" not in link:
                # Retrieve the dependent link if there is one
                if "gyroLinkDep" in link:
                    depLink = [curLink for curLink in self.__links if curLink.linkID == link["gyroLinkDep"]][0]
                else:
                    depLink = None
                self.__links.append(ArmLink(motorAddress=link["output"],
                                            jointScrewB=link["jointScrewB"],
                                            angleLimit=radians(link["angleLimit"]),
                                            linkID=link["linkID"],
                                            motorMinSpeedDPS=link["motorMinSpeedDPS"],
                                            motorMaxSpeedDPS=link["motorMaxSpeedDPS"],
                                            maxDelta=link["maxDelta"],
                                            gearRatio=link["gearRatio"],
                                            initOrder=link["initOrder"],
                                            gyroSensor=link["gyroSensor"],
                                            gyroAngleVerticalOffsetX=link["gyroAngleVerticalOffsetX"] if "gyroAngleVerticalOffsetX" in link else None,
                                            gyroLinkDep=depLink,
                                            polarity=link["polarity"],
                                            wiggleDeg=link["wiggleDeg"]))
            else:
                # Modeled links (to compensate the lack of IK for the mobile manipulator)
                self.__links.append(ArmLink(jointScrewB=link["jointScrewB"],
                                            angleLimit=radians(link["angleLimit"]),
                                            linkID=link["linkID"],
                                            initOrder=link["initOrder"],
                                            virtualLink=link["virtual"]))
        # region EndEffector Init
        if armLinkDefinitions[armName]["endEffector"]:
            endEffectorDefinition = armLinkDefinitions[armName]["endEffector"]
            self.__endEffector = EndEffector(motor_address=endEffectorDefinition["output"],
                                             status=EndEffector.OPENED_GRIP,
                                             initSpeedPercent=endEffectorDefinition["initSpeedPercent"],
                                             initMethod=endEffectorDefinition["initMethod"],
                                             openClose=endEffectorDefinition["openClose"])
            self.__endEffector.initialize(percentSpeedTest=self.__endEffector.initSpeedPercent,
                                          method=self.__endEffector.initMethod)
        else:
            self.__endEffector = None
        self.armZeroPosition = radians(armLinkDefinitions[armName]["armZeroPosition"])
        self.armRestPosition = radians(armLinkDefinitions[armName]["armRestPosition"])
        self.armDrivePosition = radians(armLinkDefinitions[armName]["armDrivePosition"])
        self.__links.sort(key=lambda x: x.linkID)
        log.info(LOGGER_ARM_MAIN, 'Done arm initialization.')

    # region properties
    @property
    def bList(self):
        return np.array([link.jointScrewB for link in self.__links]).T

    @property
    def odometryPosition(self):
        curThetaString = ''
        for link in [link for link in self.__links if link.virtual is False]:
            curThetaString += 'j{0}:{1:.1f} - '.format(link.linkID, link.armAngleDegX)
        return curThetaString[:-2]

    @property
    def urgentStop(self):
        return self._urgentStop

    @urgentStop.setter
    def urgentStop(self, value):
        assert isinstance(value, bool), "UrgentStop should be a bool, it's of type {}".format(type(value))
        for link in [link for link in self.__links if link.initOrder > 0]:
            link.urgentStop = value
        if self.__endEffector:
            self.__endEffector.openEndEffector()
            time.sleep(1)
            self.__endEffector.urgentStop = value

    @property
    def killThread(self):
        return self._killThread

    @killThread.setter
    def killThread(self, value):
        """ Mark each link so their respective threads are killed (gyroscope + motion) """
        for link in [link for link in self.__links if link.initOrder > 0]:
            link.killThread = True
        self._killThread = value

    @property
    def currentConfigVector(self):
        config_list = []
        for link in [link for link in self.__links if link.virtual is False]:
            config_list.append(link.armAngleDegX)
        # add the end effector
        config_list.append(self.__endEffector.status)
        return np.r_[config_list]

    @property
    def endEffectorStatus(self):
        return self.__endEffector.status
    # endregion

    def __str__(self):
        '''
        Definition: returns a string containing the angles for easy printout (in debugging)
        '''
        curThetaString = ''
        for link in [link for link in self.__links if link.virtual is False]:
            curX = link.armAngleDegX
            curThetaString += f'j{link.linkID}:{curX:.1f} - '
        # add real angle (wrt vertical) - accessing real value - semi illegal... not elegant
        for link in [link for link in self.__links if link.virtual is False]:
            curX = link.gyroSensor.currentAngleXDeg
            curThetaString += f'r{link.linkID}:{curX:.1f} - '
        return curThetaString[:-2]

    def resetArm(self):
        '''
        Description:resets the motors.
        params:None
        '''
        log.debug(LOGGER_ARM_RESET, 'Resetting arm...')
        for link in self.__links:
            link.resetLink()
        log.debug(LOGGER_ARM_RESET, 'Done.')

    def moveToAngle(self,
                    armConfigVector,
                    dryrun=False):
        '''
        Description:meant to move links to position (in radians) at a certain fixed speed
        param:targetAngleRadians:desc:target angle in radians
        param:targetAngleRadians:type:[float]
        param:dryrun:desc:if true, won't run, if false, will do.
        param:dryrun:type:bool
        '''

        try:
            self.__links.sort(key=lambda x: x.linkID)
            physicalLinks = [link for link in self.__links if link.initOrder > 0]
            self.__motionThreads = []
            log.debug(LOGGER_ARM_MTA, 'Preparing the threads to move each arm link.')
            for idx, link in enumerate(physicalLinks):
                thread = threading.Thread(target=link.moveToAngle, args=([armConfigVector[idx]]), name="link.moveToAngle-{}".format(link.linkID))
                thread.live = True
                self.__motionThreads.append(thread)
            if self.lock.locked():
                log.info(LOGGER_ARM_MTA, 'Arm is locked. Waiting for unlock.')
            else:
                log.info(LOGGER_ARM_MTA, 'Arm is unlocked, proceeding.')
            with self.lock:
                log.info(LOGGER_ARM_MTA, 'Locked the arm : launching the thread for arm motion')
                for thread in self.__motionThreads:
                    thread.start()
                log.debug(LOGGER_ARM_MTA, 'Joining threads for arm motion')
                for thread in self.__motionThreads:
                    thread.join()
                log.debug(LOGGER_ARM_MTA, 'Threads joined...')
            log.info(LOGGER_ARM_MTA, 'Unlocked Arm')
        except:
            log.error(LOGGER_ARM_MTA, 'Error : {}'.format(traceback.print_exc()))
            raise
        return

    def openEndEffector(self, blocking=False, stop_action=STOP_ACTION_HOLD):
        log.info(LOGGER_ARM_EE, 'Passing the call to the end effector to open')
        if self.__endEffector:
            self.__endEffector.openEndEffector(blocking=blocking, stop_action=stop_action)

    def closeEndEffector(self, blocking=False, stop_action=STOP_ACTION_HOLD):
        log.info(LOGGER_ARM_EE, 'Passing the call to the end effector to close')
        if self.__endEffector:
            self.__endEffector.closeEndEffector(blocking=blocking, stop_action=stop_action)
