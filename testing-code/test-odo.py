from __future__ import print_function
from ev3dev2.motor import LargeMotor, OUTPUT_B, OUTPUT_C, SpeedPercent   # OUTPUT_A, OUTPUT_D, eTank
import time
print("Ready to go.")

rightMotor = LargeMotor(OUTPUT_B)
leftMotor = LargeMotor(OUTPUT_C)
leftMotor.reset()
rightMotor.reset()

# setting target speed (deg per sec)
leftMotor.speed_sp = 360
rightMotor.speed_sp = 360

rampup = 5
spValue = 20
timeStep = 0.5
simDuration = 15
totalSteps = int(simDuration / timeStep)
initial_angle = 0
final_angle = 720
ramp_up_down_left = int(rampup * 1000 * leftMotor.max_speed / leftMotor.speed_sp)
ramp_up_down_right = int(rampup * 1000 * rightMotor.max_speed / rightMotor.speed_sp)

leftMotor.ramp_up_sp = ramp_up_down_left
leftMotor.ramp_down_sp = ramp_up_down_left
rightMotor.ramp_up_sp = ramp_up_down_right
rightMotor.ramp_down_sp = ramp_up_down_right

startTime = time.time()
leftMotor.on_for_seconds(SpeedPercent(spValue), simDuration, block=False)
rightMotor.on_for_seconds(SpeedPercent(spValue), simDuration, block=True)
stopTime = time.time()

duration = stopTime - startTime
print('Traveled for {0:0.2f} seconds'.format(duration))

leftMotor.reset()
rightMotor.reset()


# pos = range(initial_angle, final_angle, int(final_angle/totalSteps))
# #leftMotor.max_speed

# time.sleep(20)
# print("Slept")
