#!/usr/bin/python3

import subprocess
import time
from click import prompt
from openpyxl import Workbook
import serial
import re
import json

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
MOTOR_LEFT_DIAMETER = 0.09  # m
MOTOR_LEFT_PULSE_PER_ROUND_OF_ENCODER = 480
MOTOR_LEFT_PWM_FREQUENCY = 1000
MOTOR_LEFT_SAMPLE_TIME = 0.05

MOTOR_RIGHT_DIAMETER = 0.09  # m
MOTOR_RIGHT_PULSE_PER_ROUND_OF_ENCODER = 480
MOTOR_RIGHT_PWM_FREQUENCY = 1000
MOTOR_RIGHT_SAMPLE_TIME = 0.05

# Kalman Filter parameters
MOTOR_LEFT_X = 0
MOTOR_LEFT_P = 10000
MOTOR_LEFT_Q = 0
MOTOR_LEFT_R = 273

MOTOR_RIGHT_X = 0
MOTOR_RIGHT_P = 10000
MOTOR_RIGHT_Q = 0
MOTOR_RIGHT_R = 273

# Test data
DATA_RECORDING = False
DIRECTION_LEFT = 1
DIRECTION_RIGHT = 1
TEST_PWM_FREQUENCY = 1000
TEST_PWM = 510

# DataRecorder parameters
WORKBOOK = DataRecoder(TEST_PWM, TEST_PWM_FREQUENCY, MOTOR_LEFT.getSampleTime())
DATA_AMOUNT = 120

# =================================================

# Non-configure parameters
# Motor instances
MOTOR_LEFT = MotorDriver(
    diameter=MOTOR_LEFT_DIAMETER,
    pulse_per_round_of_encoder=MOTOR_LEFT_PULSE_PER_ROUND_OF_ENCODER,
    pwm_frequency=MOTOR_LEFT_PWM_FREQUENCY,
    sample_time=MOTOR_LEFT_SAMPLE_TIME,
)

MOTOR_RIGHT = MotorDriver(
    diameter=MOTOR_RIGHT_DIAMETER,
    pulse_per_round_of_encoder=MOTOR_RIGHT_PULSE_PER_ROUND_OF_ENCODER,
    pwm_frequency=MOTOR_RIGHT_PWM_FREQUENCY,
    sample_time=MOTOR_RIGHT_SAMPLE_TIME,
)

# Kalman Filter instances
MOTOR_LEFT.setupValuesKF(X=MOTOR_LEFT_X, P=MOTOR_LEFT_P, Q=MOTOR_LEFT_Q, R=MOTOR_LEFT_R)
MOTOR_RIGHT.setupValuesKF(
    X=MOTOR_RIGHT_X, P=MOTOR_RIGHT_P, Q=MOTOR_RIGHT_Q, R=MOTOR_RIGHT_R
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
            # left_RPM.data = MOTOR_LEFT.getKalmanFilterRPM()
            # right_RPM.data = MOTOR_RIGHT.getKalmanFilterRPM()
            # self.left_RPM_pub.publish(left_RPM)
            # self.right_RPM_pub.publish(right_RPM)

            # self.get_logger().info('Publishing: "%s"' % msg.data)

    def subscriberCallback(self, msg):
        driveMotors(msg)


def resetKF():
    MOTOR_LEFT.setupValuesKF(
        X=MOTOR_LEFT_X, P=MOTOR_LEFT_P, Q=MOTOR_LEFT_Q, R=MOTOR_LEFT_R
    )
    MOTOR_RIGHT.setupValuesKF(
        X=MOTOR_RIGHT_X, P=MOTOR_RIGHT_P, Q=MOTOR_RIGHT_Q, R=MOTOR_RIGHT_R
    )


def driveMotors(msg):
    global previous_linear_velocity, current_state_is_straight, previous_current_state_is_straight

    # evaluate()
    linear_velocity = msg.linear.x
    angular_velocity = msg.angular.z

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

    linear_velocity_left = msg.linear.x
    linear_velocity_right = msg.linear.x

    # Turn left
    if angular_velocity >= 0:
        

    # control()

    # PID
    # controlMotors()
    # MCUSerialObject.write(formSerialData("{motor_data:[0,1000,1023,0,1000,1023]}"))
    direction = 0
    if msg.linear.x > 0:
        direction = 1
    elif msg.linear.x < 0:
        direction = -1
    else:
        direction = 0

    # Reset KF
    if msg.linear.x != previous_linear_velocity:
        MOTOR_LEFT.setupValuesKF(X=0, P=10000, Q=0, R=273)
        MOTOR_RIGHT.setupValuesKF(X=0, P=10000, Q=0, R=273)
        previous_linear_velocity = msg.linear.x

    pwm_freq_1 = MOTOR_LEFT.getPWMFrequency()
    pwm_freq_2 = MOTOR_RIGHT.getPWMFrequency()

    msg.linear.x = abs(msg.linear.x)

    data = {
        "motor_data": [
            direction,
            pwm_freq_1,
            msg.linear.x * 1023 / 0.6,
            direction,
            pwm_freq_2,
            msg.linear.x * 1023 / 0.6,
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
    time.sleep(0.1)
    MCUSerialObject.reset_input_buffer()
    MCUSerialObject.setDTR(True)

    while skipLines:
        # Skip some lines of serial when MCU is reseted
        MCUSerialObject.readline()
        skipLines = skipLines - 1

    time.sleep(0.1)


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
    readSerialData()
    # print("left tick: " + str(dictionaryData["left_tick"]))
    # print("right tick: " + str(dictionaryData["right_tick"]))
    STORE_TICK_1 = dictionaryData["left_tick"]
    STORE_TICK_2 = dictionaryData["right_tick"]


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

    MOTOR_LEFT.resetDataCount()
    MOTOR_RIGHT.resetDataCount()

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

                MOTOR_LEFT.calculateRPM(TICK_1)
                MOTOR_RIGHT.calculateRPM(TICK_2)
                rclpy.spin_once(motor_driver_node)

                if index != MOTOR_LEFT.getDataCount():
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
                        MOTOR_LEFT.setupValuesKF(X=0, P=10000, Q=0, R=273)
                        MOTOR_RIGHT.setupValuesKF(X=0, P=10000, Q=0, R=273)
                        old_pwm_value = pwm_value
                        print("change")

                    varyPWM(TEST_PWM)

                    WORKBOOK.writeData(index + 1, 1, MOTOR_LEFT.getLowPassRPM())
                    WORKBOOK.writeData(index + 1, 2, MOTOR_LEFT.getKalmanFilterRPM())
                    WORKBOOK.writeData(index + 1, 4, MOTOR_RIGHT.getLowPassRPM())
                    WORKBOOK.writeData(index + 1, 5, MOTOR_RIGHT.getKalmanFilterRPM())

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

                MOTOR_LEFT.calculateRPM(TICK_1)
                MOTOR_RIGHT.calculateRPM(TICK_2)
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
