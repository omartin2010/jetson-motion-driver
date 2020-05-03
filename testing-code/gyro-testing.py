# from ev3dev2.motor import LargeMotor, OUTPUT_A, OUTPUT_B, OUTPUT_C, OUTPUT_D, OUTPUT_E, OUTPUT_F, OUTPUT_G, OUTPUT_H, SpeedDPS, SpeedPercent
from ev3dev2.sensor.lego import Sensor     # GyroSensor, TouchSensor, UltrasonicSensor
from ev3dev2.port import LegoPort
from ev3dev2.sensor import INPUT_7, INPUT_8     # INPUT_2, INPUT_3, INPUT_5, INPUT_6, INPUT_7, INPUT_8
# import numpy as np
import logging
import time
# import sys
# import copy
# import os
# import json

logging.basicConfig(level=logging.INFO, format='%(asctime)s:%(levelname)s:%(threadName)s:%(message)s')
log = logging.getLogger(__name__)

port1 = INPUT_7
port4 = INPUT_8

legoPort1 = LegoPort(port1)
legoPort1.mode = 'nxt-i2c'
legoPort1.set_device = 'ms-absolute-imu 0x11'
# legoPort4 = LegoPort(port4)
# legoPort4.mode = 'nxt-i2c'
# legoPort4.set_device = 'ms-absolute-imu 0x11'

print('Pausing 2 seconds...')
time.sleep(2)
print('Done, creating sensor class object for {}, {}'.format(port1, port4))
sensor1 = Sensor(address='{}:i2c17'.format(port1))
# sensor4 = Sensor(address='{}:i2c17'.format(INPUT_4))
target_mode = 'ALL'
sensor1.mode = target_mode
# sensor4.mode = target_mode
while True:
    # log.info('Joint 3 ({}) X/Y/Z angle = {}/{}/{}'.format(INPUT_4, sensor4.value(0), sensor4.value(1), sensor4.value(2)))
    log.info('Joint 4 ({}) X/Y/Z angle = {}/{}/{}'.format(port1, sensor1.value(0), sensor1.value(1), sensor1.value(2)))

    time.sleep(0.125)

log.info('Done')
