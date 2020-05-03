# flake8: ignore=E501
from logger import RoboLogger
import time
from ev3dev2.port import LegoPort
from smbus import SMBus
from numpy import arctan2, pi, radians
import numpy as np
import threading
import traceback
from constant import GYRO_GYRO_READING, GYRO_ACCEL_READING, \
    GYRO_FILTER_SENSITIVITY_ADDRESS, GYRO_FILTER_SENSITIVITY_VALUE, GYRO_ACCEL_SCALE, \
    GYRO_I2C_MODE, GYRO_COMPLEMENTARY_FILTER_TIME_CONSTANT, \
    GYRO_LOOP_TIMER, GYRO_DPS_SCALE_2G, GYRO_DPS_SCALE_4G, GYRO_DPS_SCALE_8G, GYRO_DPS_SCALE_16G, \
    GYRO_IMU_COMMAND_REGISTER, ODOMETRY_GYRO_LOGGING_LOOP_AGGREGATED, \
    GYRO_ACCEL_SENSITIVITY_2G_COMMAND, GYRO_ACCEL_SENSITIVITY_4G_COMMAND, \
    GYRO_ACCEL_SENSITIVITY_8G_COMMAND, GYRO_ACCEL_SENSITIVITY_16G_COMMAND
from constant import LOGGER_GS_GETSENSORDATA, LOGGER_GS_MAIN, LOGGER_GS_THREADIMPLLOOP

log = RoboLogger.getLogger()


class GyroSensor(object):
    """
    Implementation of an angle sensor (MindSensors)
    """

    __slots__ = [
        '__bus',
        '__i2cBus',
        '__i2cAddress',
        'threadGyroLoop',
        'killThread',
        '__GYRO_ACCEL_SENSITIVITY',
        '__GYRO_DPS_SCALE',
        '__angles',
        'odometryLoopTimeStep',
        '__lastRetAccData',
        '__lastRetGyrData',
        '__linkID'
    ]

    def __init__(self,
                 sensorPort: str,
                 linkID: int,
                 i2cBus: str,
                 i2cAddress=0x11,
                 gyroFilterValue=GYRO_FILTER_SENSITIVITY_VALUE,
                 gyroSensitivity='2G',
                 odometryLoopTimeStep=GYRO_LOOP_TIMER):
        '''
        Description:constructor for an angle sensor
        param:sensorPort:desc:input used on the lego robot (INPUT_1, INPUT_2, etc.)
        param:sensorPort:type:string
        param:linkID:linkid of the robot section containing the gyroscope
        param:linkID:int
        param:i2cBus:desc:device no (/dev/i2c-x)... for brickpi3, it's INPUT_N, it's N+2
        param:i2cBus:type:int
        param:i2cAddress:desc:channel on the device - should be 0x11
        param:i2cAddress:type:int or hex
        param:gyroFitlerValue:value between 0 and 7 for n-order filtering. 7 = slows down reading by 10ms 0 = 0ms
        param:gyroFitlerValue:int
        param:gyroSensitivity:desc:sensitivity, 2g, 4g, 8g or 16g - see documentation for the sensor
        param:gyroSensitivity:type:str
        '''
        LOGGER_GS_MAIN_INSTANCE = LOGGER_GS_MAIN.format(linkID)
        # Place the port in I2C mode
        LegoPort(sensorPort).mode = GYRO_I2C_MODE
        self.__i2cBus = i2cBus
        self.__bus = SMBus(i2cBus)
        self.__i2cAddress = i2cAddress
        self.odometryLoopTimeStep = GYRO_LOOP_TIMER
        self.__linkID = linkID

        # Se gyro filter sensitivity
        log.debug(LOGGER_GS_MAIN_INSTANCE, 'Setting gyro filter sensitivity')
        # Set acceleration sensitivity to XG
        if gyroSensitivity == '2G':
            self.__GYRO_ACCEL_SENSITIVITY = GYRO_ACCEL_SENSITIVITY_2G_COMMAND
            self.__GYRO_DPS_SCALE = GYRO_DPS_SCALE_2G
        elif gyroSensitivity == '4G':
            self.__GYRO_ACCEL_SENSITIVITY == GYRO_ACCEL_SENSITIVITY_4G_COMMAND
            self.__GYRO_DPS_SCALE = GYRO_DPS_SCALE_4G
        elif gyroSensitivity == '8G':
            self.__GYRO_ACCEL_SENSITIVITY == GYRO_ACCEL_SENSITIVITY_8G_COMMAND
            self.__GYRO_DPS_SCALE = GYRO_DPS_SCALE_8G
        elif gyroSensitivity == '16G':
            self.__GYRO_ACCEL_SENSITIVITY == GYRO_ACCEL_SENSITIVITY_16G_COMMAND
            self.__GYRO_DPS_SCALE = GYRO_DPS_SCALE_16G

        # Configure device sensitivity
        self.__bus.write_byte_data(self.__i2cAddress, GYRO_FILTER_SENSITIVITY_ADDRESS, gyroFilterValue)
        self.__bus.write_byte_data(self.__i2cAddress, GYRO_IMU_COMMAND_REGISTER, self.__GYRO_ACCEL_SENSITIVITY)
        time.sleep(0.05)  # sleep after changing sensor sensitivity

        # Calculate components of acceleration (ignore gyr data at initialization)
        accData, _ = self.__getSensorData()
        pitch = np.float(arctan2(accData[0], accData[2]) * 180 / pi)
        roll = np.float(arctan2(accData[1], accData[2]) * 180 / pi)
        yaw = 0
        self.__angles = np.array([pitch, roll, yaw])

        # Launch odometry thread # TODO  : put this in a run function?
        self.killThread = False
        log.info(LOGGER_GS_MAIN_INSTANCE, 'Launching gyro thread')
        self.threadGyroLoop = threading.Thread(target=self.threadImplGyroLoop,
                                               args=([self.odometryLoopTimeStep,
                                                      ODOMETRY_GYRO_LOGGING_LOOP_AGGREGATED,
                                                      GYRO_COMPLEMENTARY_FILTER_TIME_CONSTANT]),
                                               name="threadImplGyroLoop_{}".format(self.__linkID))
        self.threadGyroLoop.start()
        log.info(LOGGER_GS_MAIN_INSTANCE, 'Started gyro thread.')

    # region

    @property
    def currentAngleXRad(self):
        return radians(self.__angles[0])

    @property
    def currentAngleYRad(self):
        return radians(self.__angles[1])

    @property
    def currentAngleZRad(self):
        return radians(self.__angles[2])

    @property
    def currentAngleXDeg(self):
        return self.__angles[0]

    @property
    def currentAngleYDeg(self):
        return self.__angles[1]

    @property
    def currentAngleZDeg(self):
        return self.__angles[2]

    # endregion

    def __getSensorData(self):
        '''
        Returns the processed sensor data (gyr + acceleration)
        '''
        LOGGER_GS_GETSENSORDATA_INSTANCE = LOGGER_GS_GETSENSORDATA.format(self.__linkID)
        try:
            sensorRawData = self.__bus.read_i2c_block_data(self.__i2cAddress, GYRO_ACCEL_READING, 6)
            self.__lastRetAccData = np.array([float(twos_comp(sensorRawData[0] + sensorRawData[1] * 256, 16)),
                                              float(twos_comp(sensorRawData[2] + sensorRawData[3] * 256, 16)),
                                              float(twos_comp(sensorRawData[4] + sensorRawData[5] * 256, 16))]) * GYRO_ACCEL_SCALE
            sensorRawData = self.__bus.read_i2c_block_data(self.__i2cAddress, GYRO_GYRO_READING, 6)
            self.__lastRetGyrData = np.array([float(twos_comp(sensorRawData[0] + sensorRawData[1] * 256, 16)),
                                              float(twos_comp(sensorRawData[2] + sensorRawData[3] * 256, 16)),
                                              float(twos_comp(sensorRawData[4] + sensorRawData[5] * 256, 16))]) * self.__GYRO_DPS_SCALE
        except OSError:
            log.error(LOGGER_GS_GETSENSORDATA_INSTANCE, 'Error reading sensor data - recovering... - Is the gyro sensor connected in the right input port? - or is the ic2bus set properly? supposed to be input+2.')
        except Exception:
            log.error(LOGGER_GS_GETSENSORDATA_INSTANCE, 'Other error in __getSensorData : {}'.format(traceback.print_exc()))
        return self.__lastRetAccData, self.__lastRetGyrData

    def threadImplGyroLoop(self, timeStep, logEvery, tau=0.98):
        '''
        Description : loop for keeping track and doing the matl for the IMU. Implements a complementary filter.
        Based on the work in this paper : https://docs.google.com/viewer?a=v&pid=sites&srcid=ZGVmYXVsdGRvbWFpbnxteWltdWVzdGltYXRpb25leHBlcmllbmNlfGd4OjY1Yzk3YzhiZmE1N2M4Y2U
        and also on this application note : https://www.analog.com/media/en/technical-documentation/application-notes/AN-1057.pdf
        and this blog post : http://www.geekmomprojects.com/gyroscopes-and-accelerometers-on-a-chip/
        timeStep:desc:delay for the loop to keep track of sensor position
        timeStep:type:float
        logEvery:desc:param specifying how often to log to screen
        logEvery:type:float
        tau:desc:time constant. alpha = tau/(tau + timeStep) ; tau = 0.5sec => alpha = (0.5 / (0.5 + 0.01)) = 0.98
                           tau = time constant. Beyond tau seconds (0.5), we start trusting more the
                           accelerometer. < 0.5s, we trust the vs gyroscope.
                           see the paper referenced above.
        tau:type:float
        '''
        LOGGER_GS_THREADIMPLLOOP_INSTANCE = LOGGER_GS_THREADIMPLLOOP.format(self.__i2cBus)
        loopCount, sleepTimeCumul = 0, 0
        # Calculate the time constant
        alpha = tau / (tau + timeStep)
        log.info(LOGGER_GS_THREADIMPLLOOP_INSTANCE, 'Initialized Thread : Loop time = {:.3f} ; time constant = {:.2f} (alpha = {:.2f})'.format(timeStep, tau, alpha))

        # Main odometry loop
        while not self.killThread:
            loopCount += 1
            startTime = time.time()
            # Get scaled data for acc + gyr
            accData, gyrData = self.__getSensorData()

            # Calculate components of acceleration per axis
            pitch = np.float(arctan2(accData[0], accData[2]) * 180 / pi)
            roll = np.float(arctan2(accData[1], accData[2]) * 180 / pi)
            yaw = 0

            # pitch = np.float(arctan2(accData[0], (sqrt(accData[1] * accData[1] + accData[2] * accData[2])))) * 180 / pi
            # roll = np.float(arctan2(accData[1], (sqrt(accData[0] * accData[0] + accData[2] * accData[2])))) * 180 / pi
            # yaw = np.float(arctan2((sqrt(accData[1] * accData[1] + accData[2] * accData[2])), accData[2])) * 180 / pi

            # Calculate the complimentary filter (alpha = time constant)
            self.__angles = alpha * (self.__angles + gyrData * timeStep) + (1 - alpha) * (np.array([pitch, roll, yaw]))

            # if none, don't display here, it'll be displayed in the main loop.
            if logEvery:
                if loopCount % logEvery == 0:
                    log.debug(LOGGER_GS_THREADIMPLLOOP_INSTANCE, f'angles = {self.__angles[0]:.2f}/{self.__angles[1]:.2f}/{self.__angles[2]:.2f}')
                    avgLoopTime = sleepTimeCumul / logEvery
                    sleepTimeCumul = 0
                    log.debug(LOGGER_GS_THREADIMPLLOOP_INSTANCE, f'average sleep time/loop = {avgLoopTime:.4f}s vs loop duration = {timeStep:.4f} ==> average loop duration = {timeStep - avgLoopTime:.4f}')

            # find if there's time to sleep until the next iteration and if so, sleep.
            endTime = time.time()
            sleepTime = max(0, timeStep - (endTime - startTime))
            sleepTimeCumul += sleepTime
            time.sleep(sleepTime)
        log.info(LOGGER_GS_THREADIMPLLOOP_INSTANCE, 'Exiting gyroloop sensor thread.')


def twos_comp(val, bits):
    """compute the 2's complement of int value val"""
    # if sign bit is set e.g., 8bit: 128-255
    if (val & (1 << (bits - 1))) != 0:
        # compute negative value
        val = val - (1 << bits)
    return val
