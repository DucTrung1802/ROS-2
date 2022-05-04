#!/usr/bin/python3

import subprocess
import time
from click import prompt
from openpyxl import Workbook
import serial
import re
import json
import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
from motor.MotorDriver import MotorDriver
from motor.DataRecoder import DataRecoder


# =========== Configurable parameters =============
# Serial parameters
SKIP_SERIAL_LINES = 12
LIDAR_USB_NAME = "FTDI USB Serial Device"
MCU_USB_NAME = "cp210x"
BAUD_RATE = 115200
RECEIVING_FREQUENCY = 2000

# Node parameters
PUBLISH_FREQUENCY = 1000
NODE_NAME = "motor_driver"

# Motor parameters
LEFT_MOTOR_MAX_VELOCITY = 0.6
RIGHT_MOTOR_MAX_VELOCITY = 0.6

LEFT_MOTOR_DIAMETER = 0.09  # m
LEFT_MOTOR_PULSE_PER_ROUND_OF_ENCODER = 480
LEFT_MOTOR_PWM_FREQUENCY = 1000
LEFT_MOTOR_SAMPLE_TIME = 0.05
left_wheel_RPM_for_1_rad_per_sec_of_robot = 100

RIGHT_MOTOR_DIAMETER = 0.09  # m
RIGHT_MOTOR_PULSE_PER_ROUND_OF_ENCODER = 480
RIGHT_MOTOR_PWM_FREQUENCY = 1000
RIGHT_MOTOR_SAMPLE_TIME = 0.05
right_wheel_RPM_for_1_rad_per_sec_of_robot = 100

# Kalman Filter parameters
LEFT_MOTOR_X = 0
LEFT_MOTOR_P = 10000
LEFT_MOTOR_Q = 0
LEFT_MOTOR_R = 273

RIGHT_MOTOR_X = 0
RIGHT_MOTOR_P = 10000
RIGHT_MOTOR_Q = 0
RIGHT_MOTOR_R = 273

# Test data
DATA_RECORDING = False
DIRECTION_LEFT = 1
DIRECTION_RIGHT = 1
TEST_PWM_FREQUENCY = 1000
TEST_PWM = 510

# DataRecorder parameters
DATA_AMOUNT = 120

# =================================================

# Non-configure parameters
# Motor instances
LEFT_MOTOR = MotorDriver(
    diameter=LEFT_MOTOR_DIAMETER,
    pulse_per_round_of_encoder=LEFT_MOTOR_PULSE_PER_ROUND_OF_ENCODER,
    pwm_frequency=LEFT_MOTOR_PWM_FREQUENCY,
    sample_time=LEFT_MOTOR_SAMPLE_TIME,
)

RIGHT_MOTOR = MotorDriver(
    diameter=RIGHT_MOTOR_DIAMETER,
    pulse_per_round_of_encoder=RIGHT_MOTOR_PULSE_PER_ROUND_OF_ENCODER,
    pwm_frequency=RIGHT_MOTOR_PWM_FREQUENCY,
    sample_time=RIGHT_MOTOR_SAMPLE_TIME,
)

# Motor parameters
LEFT_MOTOR_MAX_RPM = LEFT_MOTOR_MAX_VELOCITY / (LEFT_MOTOR_DIAMETER * math.pi) * 60.0
RIGHT_MOTOR_MAX_RPM = RIGHT_MOTOR_MAX_VELOCITY / (RIGHT_MOTOR_DIAMETER * math.pi) * 60.0

# Kalman Filter instances
LEFT_MOTOR.setupValuesKF(X=LEFT_MOTOR_X, P=LEFT_MOTOR_P, Q=LEFT_MOTOR_Q, R=LEFT_MOTOR_R)
RIGHT_MOTOR.setupValuesKF(
    X=RIGHT_MOTOR_X, P=RIGHT_MOTOR_P, Q=RIGHT_MOTOR_Q, R=RIGHT_MOTOR_R
)


# Node parameters
receiving_timer = time.time()
publish_timer = time.time()
previous_linear_velocity = 0
current_state_is_straight = True
previous_current_state_is_straight = True
RECEIVING_PERIOD = 1
""" The timer will be started and every ``PUBLISH_PERIOD`` number of seconds the provided\
    callback function will be called. For no delay, set it equal ZERO. """
PUBLISH_PERIOD = 0

# Serial parameters
MCUSerialObject = None
foundMCU = False
foundLidar = False
serialData = ""
dictionaryData = {}

# JSON parameters
KEY = "pwm_pulse"
STORE_TICK_1 = 0
STORE_TICK_2 = 0
TICK_1 = 0
TICK_2 = 0

# Data recorder
WORKBOOK = DataRecoder(TEST_PWM, TEST_PWM_FREQUENCY, LEFT_MOTOR.getSampleTime())


def checkConditions():
    global RECEIVING_PERIOD, PUBLISH_PERIOD
    if RECEIVING_FREQUENCY <= 0:
        raise Exception("RECEIVING_FREQUENCY must be an positive integer!")
    RECEIVING_PERIOD = 1 / RECEIVING_FREQUENCY

    if PUBLISH_FREQUENCY <= 0:
        raise Exception("PUBLISH_FREQUENCY must be an positive integer!")
    PUBLISH_PERIOD = 1 / PUBLISH_FREQUENCY


class MotorDriverNode(Node):
    def __init__(self, node_name):
        super().__init__(node_name)
        self.__need_publish = True
        self.left_ticks_pub = self.create_publisher(Int32, "left_ticks", 1)
        self.right_ticks_pub = self.create_publisher(Int32, "right_ticks", 1)
        self.left_RPM_pub = self.create_publisher(Float32, "left_RPM", 1)
        self.right_RPM_pub = self.create_publisher(Float32, "right_RPM", 1)
        self.timer = self.create_timer(0, self.publisherCallback)

        self.controller_sub = self.create_subscription(
            Twist, "cmd_vel", self.subscriberCallback, 1
        )
        self.controller_sub  # prevent unused variable warning

    def setNeedPublish(self):
        self.__need_publish = True

    def resetNeedPublish(self):
        self.__need_publish = False

    def publisherCallback(self):
        if self.__need_publish:
            left_ticks = Int32()
            right_ticks = Int32()
            left_ticks.data = TICK_1
            right_ticks.data = TICK_2
            self.left_ticks_pub.publish(left_ticks)
            self.right_ticks_pub.publish(right_ticks)

            # left_RPM = Float32()
            # right_RPM = Float32()
            # left_RPM.data = LEFT_MOTOR.getKalmanFilterRPM()
            # right_RPM.data = RIGHT_MOTOR.getKalmanFilterRPM()
            # self.left_RPM_pub.publish(left_RPM)
            # self.right_RPM_pub.publish(right_RPM)

            # self.get_logger().info('Publishing: "%s"' % msg.data)

    def subscriberCallback(self, msg):
        driveMotors(msg)


def MPStoRPM(mps):
    return mps / (LEFT_MOTOR_DIAMETER * math.pi) * 60


def RPMtoMPS(rpm):
    return rpm * (LEFT_MOTOR_DIAMETER * math.pi) / 60


def saturate(index, min, max):
    if index <= min:
        return min
    elif index >= max:
        return max
    else:
        return index


def getDirection(velocity):
    if velocity >= 0:
        return 1
    else:
        return -1


def resetKF():
    LEFT_MOTOR.setupValuesKF(
        X=LEFT_MOTOR_X, P=LEFT_MOTOR_P, Q=LEFT_MOTOR_Q, R=LEFT_MOTOR_R
    )
    RIGHT_MOTOR.setupValuesKF(
        X=RIGHT_MOTOR_X, P=RIGHT_MOTOR_P, Q=RIGHT_MOTOR_Q, R=RIGHT_MOTOR_R
    )


def differientialDriveLeft(angular_velocity):
    return left_wheel_RPM_for_1_rad_per_sec_of_robot * angular_velocity


def differientialDriveRight(angular_velocity):
    return right_wheel_RPM_for_1_rad_per_sec_of_robot * angular_velocity


def driveMotors(msg):
    global previous_linear_velocity, current_state_is_straight, previous_current_state_is_straight

    # Evaluate to have RPM value

    linear_velocity = MPStoRPM(msg.linear.x)  # RPM
    angular_velocity = msg.angular.z  # rad/s

    if linear_velocity != previous_linear_velocity:
        resetKF()
        previous_linear_velocity = linear_velocity

    if angular_velocity:
        current_state_is_straight = False
    elif not angular_velocity:
        current_state_is_straight = True

    if current_state_is_straight != previous_current_state_is_straight:
        resetKF()
        previous_current_state_is_straight = current_state_is_straight

    # Differential drive
    linear_velocity_left = abs(linear_velocity)  # RPM
    linear_velocity_right = abs(linear_velocity)  # RPM

    if angular_velocity >= 0:
        linear_velocity_left -= differientialDriveLeft(abs(msg.angular.z))
        linear_velocity_right += differientialDriveRight(abs(msg.angular.z))
    elif angular_velocity < 0:
        linear_velocity_left += differientialDriveLeft(abs(msg.angular.z))
        linear_velocity_right -= differientialDriveRight(abs(msg.angular.z))

    linear_velocity_left = saturate(linear_velocity_left, 0, LEFT_MOTOR_MAX_RPM)
    linear_velocity_right = saturate(linear_velocity_right, 0, RIGHT_MOTOR_MAX_RPM)

    print("Left RPM: " + str(linear_velocity_left))
    print("Right RPM: " + str(linear_velocity_right))
    print("---")

    # Control
    direction = getDirection(msg.linear.x)

    pwm_freq_1 = LEFT_MOTOR.getPWMFrequency()
    pwm_freq_2 = RIGHT_MOTOR.getPWMFrequency()

    data = {
        "motor_data": [
            direction,
            pwm_freq_1,
            linear_velocity_left * 1023 / MPStoRPM(LEFT_MOTOR_MAX_VELOCITY),
            direction,
            pwm_freq_2,
            linear_velocity_right * 1023 / MPStoRPM(RIGHT_MOTOR_MAX_VELOCITY),
        ]
    }

    data = json.dumps(data)
    MCUSerialObject.write(formSerialData(data))


def getMCUSerial():
    global foundMCU, foundLidar
    MCUSerial = None
    output = subprocess.getstatusoutput("dmesg | grep ttyUSB")
    stringDevices = list(output[1].split("\n"))
    stringDevices.reverse()

    for device in stringDevices:
        if device.find(MCU_USB_NAME) > 0 and not foundMCU:
            if device.find("disconnected") > 0:
                raise Exception("MCU is disconnected!")
            else:
                # print("MCU in serial: " + device.split()[-1])
                foundMCU = True
                MCUSerial = device.split()[-1]
                break

        # elif (device.find(LIDAR_USB_NAME) > 0 and not foundLidar):
        #     if (device.find("disconnected") > 0):
        #         raise Exception("Lidar is disconnected!")
        #     else:
        #         # print("Lidar in serial: " + device.split()[-1])
        #         foundLidar = True
        #         continue

    return MCUSerial


def initializeSerial():
    global MCUSerialObject
    MCUSerial = getMCUSerial()
    if MCUSerial:
        MCUSerialObject = serial.Serial(
            port="/dev/" + MCUSerial,
            baudrate=BAUD_RATE,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            bytesize=serial.EIGHTBITS,
            timeout=1,
            write_timeout=1,
        )

    skipLines = SKIP_SERIAL_LINES
    MCUSerialObject.setDTR(False)
    time.sleep(0.01)
    MCUSerialObject.reset_input_buffer()
    MCUSerialObject.setDTR(True)

    while skipLines:
        # Skip some lines of serial when MCU is reseted
        MCUSerialObject.readline()
        skipLines = skipLines - 1

    time.sleep(0.01)


def readSerialData():
    global serialData, dictionaryData
    rawData = MCUSerialObject.readline()
    # print(rawData)
    serialData = rawData.decode("windows-1252")  # decode s
    serialData = serialData.rstrip()  # cut "\r\n" at last of string
    filteredSerialData = re.sub(
        "[^A-Za-z0-9\s[]{}]", "", serialData
    )  # filter regular characters
    # print(filteredSerialData)
    try:
        dictionaryData = json.loads(filteredSerialData)
    except:
        return


def updateStorePosFromSerial():
    global STORE_TICK_1, STORE_TICK_2
    # MCUSerialObject.write(formSerialData("{pwm_pulse:[1023,1023]}"))
    try:
        readSerialData()
        STORE_TICK_1 = dictionaryData["left_tick"]
        STORE_TICK_2 = dictionaryData["right_tick"]
    except:
        return


#
def updatePosFromStorePos():
    global TICK_1, TICK_2
    TICK_1 = STORE_TICK_1
    TICK_2 = STORE_TICK_2


def manuallyWrite():
    # A command is appended with "#" to mark as finish
    command = prompt("Command") + "#"
    command = bytes(command, "utf-8")
    MCUSerialObject.write(command)


def formSerialData(stringData):
    stringData += "#"
    data = bytes(stringData, "utf-8")
    return data


def varyPWM(PWM):
    test_dict = {
        "motor_data": [
            DIRECTION_LEFT,
            TEST_PWM_FREQUENCY,
            PWM,
            DIRECTION_RIGHT,
            TEST_PWM_FREQUENCY,
            PWM,
        ]
    }
    MCUSerialObject.write(formSerialData(json.dumps(test_dict)))


def setup():
    checkConditions()
    initializeSerial()


def loop():
    global receiving_timer, publish_timer, TICK_1, TICK_2
    rclpy.init()

    motor_driver_node = MotorDriverNode(NODE_NAME)

    LEFT_MOTOR.resetDataCount()
    RIGHT_MOTOR.resetDataCount()

    try:
        if DATA_RECORDING:
            index = 0
            old_pwm_value = 0
            pwm_value = 714

            while index <= DATA_AMOUNT:

                # varyPWM(1023)

                if time.time() - receiving_timer >= RECEIVING_PERIOD:
                    updateStorePosFromSerial()
                    receiving_timer = time.time()

                if time.time() - publish_timer >= PUBLISH_PERIOD:
                    updatePosFromStorePos()
                    motor_driver_node.setNeedPublish()
                    rclpy.spin_once(motor_driver_node)
                    motor_driver_node.resetNeedPublish()
                    publish_timer = time.time()

                LEFT_MOTOR.calculateRPM(TICK_1)
                RIGHT_MOTOR.calculateRPM(TICK_2)
                rclpy.spin_once(motor_driver_node)

                if index != LEFT_MOTOR.getDataCount():
                    print(str(index) + "/" + str(DATA_AMOUNT))
                    index += 1

                    # Vary PWM
                    # if 0 < index <= DATA_AMOUNT / 3:
                    #     pwm_value = 1023
                    # elif DATA_AMOUNT / 3 < index <= DATA_AMOUNT * 2 / 3:
                    #     pwm_value = 714
                    # else:
                    #     pwm_value = 510

                    if pwm_value != old_pwm_value:
                        LEFT_MOTOR.setupValuesKF(X=0, P=10000, Q=0, R=273)
                        RIGHT_MOTOR.setupValuesKF(X=0, P=10000, Q=0, R=273)
                        old_pwm_value = pwm_value
                        print("change")

                    varyPWM(TEST_PWM)

                    WORKBOOK.writeData(index + 1, 1, LEFT_MOTOR.getLowPassRPM())
                    WORKBOOK.writeData(index + 1, 2, LEFT_MOTOR.getKalmanFilterRPM())
                    WORKBOOK.writeData(index + 1, 4, RIGHT_MOTOR.getLowPassRPM())
                    WORKBOOK.writeData(index + 1, 5, RIGHT_MOTOR.getKalmanFilterRPM())

        else:
            while True:
                # manuallyWrite()
                if time.time() - receiving_timer >= RECEIVING_PERIOD:
                    updateStorePosFromSerial()
                    receiving_timer = time.time()

                if time.time() - publish_timer >= PUBLISH_PERIOD:
                    updatePosFromStorePos()
                    motor_driver_node.setNeedPublish()
                    rclpy.spin_once(motor_driver_node)
                    motor_driver_node.resetNeedPublish()
                    publish_timer = time.time()

                LEFT_MOTOR.calculateRPM(TICK_1)
                RIGHT_MOTOR.calculateRPM(TICK_2)
                rclpy.spin_once(motor_driver_node)

    except KeyboardInterrupt:
        # JSON
        MCUSerialObject.write(formSerialData("{motor_data:[0,1000,0,0,1000,0]}"))
        MCUSerialObject.close()
        WORKBOOK.saveWorkBook()

    finally:
        MCUSerialObject.write(formSerialData("{motor_data:[0,1000,0,0,1000,0]}"))
        MCUSerialObject.close()
        WORKBOOK.saveWorkBook()


def main():
    setup()
    loop()


if __name__ == "__main__":
    main()
