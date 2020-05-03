from ev3dev2.motor import LargeMotor, OUTPUT_A, OUTPUT_B, OUTPUT_C, OUTPUT_D, OUTPUT_E, OUTPUT_F, OUTPUT_G, OUTPUT_H
from ev3dev2.sensor.lego import TouchSensor, UltrasonicSensor, Sensor
from ev3dev2.sensor import INPUT_1, INPUT_2, INPUT_3, INPUT_4, INPUT_5, INPUT_6, INPUT_7, INPUT_8
STOP_ACTION_COAST = LargeMotor.STOP_ACTION_COAST
STOP_ACTION_BRAKE = LargeMotor.STOP_ACTION_BRAKE
STOP_ACTION_HOLD = LargeMotor.STOP_ACTION_HOLD

# region gyro constants
GYRO_GYRO_READING = 0x53
GYRO_ACCEL_READING = 0x45
GYRO_IMU_COMMAND_REGISTER = 0x41
GYRO_ACCEL_SENSITIVITY_2G_COMMAND = 0x31
GYRO_ACCEL_SENSITIVITY_4G_COMMAND = 0x32
GYRO_ACCEL_SENSITIVITY_8G_COMMAND = 0x33
GYRO_ACCEL_SENSITIVITY_16G_COMMAND = 0x34
GYRO_FILTER_SENSITIVITY_ADDRESS = 0x5A
GYRO_FILTER_SENSITIVITY_VALUE = 3
GYRO_ACCEL_SCALE = 0.00981
GYRO_DPS_SCALE_2G = 1 / (32767 / 250)     # 0.00875     # Degrees/Sec (at G)
GYRO_DPS_SCALE_4G = 1 / (32767 / 500)     # 0.0175      # Degrees/Sec (at 4G)
GYRO_DPS_SCALE_8G = 1 / (32767 / 1000)    # 0.035       # Degrees/Sec (at 8G)
GYRO_DPS_SCALE_16G = 1 / (32767 / 2000)   # 0.070      # Degrees/Sec (at 16G)
GYRO_I2C_MODE = 'nxt-i2c'
GYRO_LOOP_TIMER = 0.1  # run loop every 100ms
GYRO_COMPLEMENTARY_FILTER_TIME_CONSTANT = 1.0
# endregion

MOTOR_INIT_METHOD_BUTTON_PRESSED = 'button-pressed'     # Press a button to indicate the min and max limits
MOTOR_INIT_METHOD_MECHANICAL = 'mechanical'             # Mechanical blocking determine joint limits - WARNING - CAN BREAK GEARS
MOTOR_INIT_METHOD_MANUAL = 'manual'                     # Manually set the joint position to 0 and press the button
MOTOR_INIT_METHOD_DONT_INIT = 'no_init'                 # Do not change the current settings
JOINT_TESTING_DIRECTION_MIN = 'min'
JOINT_TESTING_DIRECTION_MAX = 'max'
JOINT_MAX_SPEED_PERCENT = 50
JACOBIAN_FULL_BODY_ARM = 'full'
JACOBIAN_ARM = 'arm'
JACOBIAN_BASE = 'base'
MOVEMENT_METHOD_POSITION = 'position'
MOVEMENT_METHOD_VELOCITY = 'velocity'
MOVEMENT_TIME_SCALING_METHOD_QUINTIC = 5
MOVEMENT_TIME_SCALING_METHOD_CUBIC = 3
MOVEMENT_TIME_SCALING_METHOD_DEFAULT = MOVEMENT_TIME_SCALING_METHOD_CUBIC
ODOMETRY_BASE_LOGGING_LOOP_AGGREGATED = 250   # 500
ODOMETRY_GYRO_LOGGING_LOOP_AGGREGATED = 10
MOTOR_RAMP_UP_TIME = 30000
JOINT_MOTOR_RAMP_TIME = 50000   # means going from 0 to 10% of maxpseed will happen in ((10%*1500)/1500) * 25000msec = 2.5s
JOINT_MOTOR_RAMP_UP_TIME = JOINT_MOTOR_RAMP_TIME
JOINT_MOTOR_RAMP_DOWN_TIME = JOINT_MOTOR_RAMP_TIME
MAX_WORKER_THREADS = 5
LOW_VOLTAGE_THRESHOLD = 5               # CHANGE to 7 when batteries are charged.
DOUBLECLICK = 1     # seconds for doubleclick on sensors

# region LOGGERS
LOGGER_STARTUP_MAIN = 'logger_main'
LOGGER_DDR_MAIN = 'ddr_main'
LOGGER_DDR_RUN = 'ddr_run'
LOGGER_DDR_THREADIMPLEVENTLOOP = 'ddr_eventloop'
LOGGER_DDR_IMPLMOVELOOP = 'ddr_moveloop'
LOGGER_DDR_VOLTAGECHECK = 'ddr_voltagecheck'
LOGGER_DDR_ODOMETRY = 'ddr_odometry'
LOGGER_DDR_EXECUTORKILLSWITCH = 'ddr_killswitch'
LOGGER_DDR_KILLSWITCH = 'ddr_killswitch'
LOGGER_DDR_THREADIMPLMQTT = 'ddr_mqtt'
LOGGER_EE_INIT = 'ee_init'
LOGGER_EE_FINDPHYLIMIT = 'ee_limit'
LOGGER_EE_RESETMOTOR = 'ee_reset'
LOGGER_EE_ENDEFFECTOR = 'ee_motion'
LOGGER_GS_MAIN = 'gs_main_{}'
LOGGER_GS_GETSENSORDATA = 'gs_getdata_{}'
LOGGER_GS_THREADIMPLLOOP = 'gs_threadsensorloop_{}'
LOGGER_MOTOR_MAIN = 'motor_main_{}'
LOGGER_MOTOR_OTP = 'motor_ontopos_{}'
LOGGER_MOTOR_OTPPID = 'motor_ontopospid_{}'
LOGGER_ARM_MAIN = 'arm_main'
LOGGER_ARM_RESET = 'arm_reset'
LOGGER_ARM_MTA = 'arm_movetoangle'
LOGGER_ARM_EE = 'arm_moveeendeffector'
LOGGER_LINK_MAIN = 'link_main_{}'
LOGGER_LINK_RESET = 'link_reset_{}'
LOGGER_LINK_MTA = 'link_mta_{}'
LOGGER_BASE_MAIN = 'base_main'
LOGGER_BASE_RUN = 'base_run'
LOGGER_BASE_PRIMWHEELMVT = 'base_prim_wheel'
LOGGER_BASE_ROTATEBASEPHI = 'base_rotate_phi'
LOGGER_BASE_MOVEX = 'base_movex'
LOGGER_BASE_MOVEXYPHI = 'base_movexyphi'
LOGGER_BASE_ODOMETRY = 'base_odometry'
# endregion

LEGO_ROBOT_DEFINITIONS = {
    "LargeMotor": LargeMotor,
    "OUTPUT_A": OUTPUT_A,
    "OUTPUT_B": OUTPUT_B,
    "OUTPUT_C": OUTPUT_C,
    "OUTPUT_D": OUTPUT_D,
    "OUTPUT_E": OUTPUT_E,
    "OUTPUT_F": OUTPUT_F,
    "OUTPUT_G": OUTPUT_G,
    "OUTPUT_H": OUTPUT_H,
    "INPUT_1": INPUT_1,
    "INPUT_2": INPUT_2,
    "INPUT_3": INPUT_3,
    "INPUT_4": INPUT_4,
    "INPUT_5": INPUT_5,
    "INPUT_6": INPUT_6,
    "INPUT_7": INPUT_7,
    "INPUT_8": INPUT_8,
    "MOTOR_INIT_METHOD_DONT_INIT": MOTOR_INIT_METHOD_DONT_INIT,
    "MOTOR_INIT_METHOD_MECHANICAL": MOTOR_INIT_METHOD_MECHANICAL,
    "MOTOR_INIT_METHOD_BUTTON_PRESSED": MOTOR_INIT_METHOD_BUTTON_PRESSED,
    "MOTOR_INIT_METHOD_MANUAL": MOTOR_INIT_METHOD_MANUAL,
    "TouchSensor": TouchSensor,
    "UltrasonicSensor": UltrasonicSensor,
    "Sensor": Sensor,
    "True": True,
    "False": False,
    "STOP_ACTION_BRAKE": STOP_ACTION_BRAKE,
    "STOP_ACTION_COAST": STOP_ACTION_COAST,
    "STOP_ACTION_HOLD": STOP_ACTION_HOLD
}
# endregion
