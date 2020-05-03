# flake8: ignore=E501
from robot import DiffDriveManipulatorRobot
import time
import numpy as np
from logger import RoboLogger
import logging
import argparse
# import getopt
import signal
import sys
import traceback
from constant import LOGGER_STARTUP_MAIN

log = RoboLogger.getLogger()
bot = None


def main():
    """
    Full Robot Program
    """

    parser = argparse.ArgumentParser(description='Robot!')
    parser.add_argument('-f', '--file', type=str, dest='robot_config_file', required=True,
                        help='configuration file for the robot')
    parser.add_argument('-l', '--logging', type=int, dest='logging', default=logging.WARNING,
                        help='Logging level for main robot = 10 = debug, 20=info, 30=warn, 40=error, 50=critical.')
    parser.add_argument('-t', '--timestep', type=float, dest='timeStep', default=0.25,
                        help='timestamp used in some of the integration steps for the robot')
    parser.add_argument('-o', '--odometry_timer', type=float, dest='odometryTimer', default=0.01,
                        help='odometry timer for various odometry (IMU, robot positions, etc.)')
    args = parser.parse_args()

    try:
        global bot
        log.warning(LOGGER_STARTUP_MAIN, msg='Launching main BOT now.')
        initRobotConfig = np.array((0, 0, 0), dtype=np.float32)
        # X, Y, phi is the initial configuration. PHI could take the reading from the gyroscope depending on research on its X orientation.
        signal.signal(signal.SIGTERM, sigterm_handler)
        bot = DiffDriveManipulatorRobot(robot_config_file=args.robot_config_file,
                                        initRobotConfig=initRobotConfig,
                                        timeStep=args.timeStep,  # 0.25,
                                        odometryTimer=args.odometryTimer,      # 0.01
                                        defaultLogging=args.logging)
        bot.run()
    except SystemExit:
        log.info(LOGGER_STARTUP_MAIN, 'Caught SystemExit...')
    except:
        log.critical(LOGGER_STARTUP_MAIN, 'Crash in startup : {}'.format(traceback.print_exc()))
    finally:
        graceful_shutdown()
        logging.shutdown()


def sigterm_handler(sig, frame):
    log.info(LOGGER_STARTUP_MAIN, 'SIGTERM caught. Docker Container being terminated.')
    graceful_shutdown()
    log.info(LOGGER_STARTUP_MAIN, 'SIGTERM signal processing done.')


def graceful_shutdown():
    try:
        log.info(LOGGER_STARTUP_MAIN, 'Initiating Graceful Shutdown')
        global bot
        if 'bot' in globals():
            log.info(LOGGER_STARTUP_MAIN, 'Deleting robot... now.')
            bot.__del__()
            del bot
        sys.exit(0)
    except SystemExit:
        log.info(LOGGER_STARTUP_MAIN, 'Exiting process.')
    except:
        log.critical(LOGGER_STARTUP_MAIN, 'trace: {}'.format(traceback.print_exc()))


if __name__ == "__main__":
    main()  # robot_config_file=robot_config_file)
    time.sleep(10)
    log.info(LOGGER_STARTUP_MAIN, "Done")
    try:
        sys.exit(0)
    except SystemExit:
        log.info(LOGGER_STARTUP_MAIN, 'Exiting startup.')
