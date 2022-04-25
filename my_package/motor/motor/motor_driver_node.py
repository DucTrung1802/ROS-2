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
PUBLISH_FREQUENCY = 100
NODE_NAME = "motor_driver"

# Motor parameters
MOTOR_1 = MotorDriver(
    diameter=0.09, pulse_per_round_of_encoder=480, pwm_frequency=1000, sample_time=0.05
)
MOTOR_1.setupValuesKF(X=0, P=10000, Q=0, R=273)

MOTOR_2 = MotorDriver(
    diameter=0.09, pulse_per_round_of_encoder=480, pwm_frequency=1000, sample_time=0.05
)
MOTOR_2.setupValuesKF(X=0, P=10000, Q=0, R=273)


# Test data
TEST_PWM_FREQUENCY = 1000
TEST_PWM = 500

# DataRecorder parameters
WORKBOOK = DataRecoder(TEST_PWM, TEST_PWM_FREQUENCY, MOTOR_1.getSampleTime())
DATA_AMOUNT = 500
# =================================================

# Non-configure parameters
# Node parameters
receiving_timer = time.time()
publish_timer = time.time()
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
            # left_ticks = Int32()
            # right_ticks = Int32()
            # left_ticks.data = TICK_1
            # right_ticks.data = TICK_2
            # self.left_ticks_pub.publish(left_ticks)
            # self.right_ticks_pub.publish(right_ticks)

            left_RPM = Float32()
            right_RPM = Float32()
            left_RPM.data = MOTOR_1.getKalmanFilterRPM()
            right_RPM.data = MOTOR_2.getKalmanFilterRPM()
            self.left_RPM_pub.publish(left_RPM)
            self.right_RPM_pub.publish(right_RPM)

            # self.get_logger().info('Publishing: "%s"' % msg.data)

    def subscriberCallback(self, msg):
        driveMotors(msg)


def driveMotors(msg):
    # Kalman Filter
    # PID
    # controlMotors()
    # MCUSerialObject.write(formSerialData("{motor_data:[1000,1023,1000,1023]}"))
    pwm_freq_1 = MOTOR_1.getPWMFrequency()
    pwm_freq_2 = MOTOR_2.getPWMFrequency()
    data = {
        "motor_data": [
            pwm_freq_1,
            msg.linear.x * 1023 / 0.6,
            pwm_freq_2,
            msg.linear.x * 1023 / 0.6,
        ]
    }
    data = json.dumps(data)
    MCUSerialObject.write(formSerialData(data))
    pass


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
    test_dict = {"motor_data": [TEST_PWM_FREQUENCY, PWM, TEST_PWM_FREQUENCY, PWM]}
    MCUSerialObject.write(formSerialData(json.dumps(test_dict)))


def setup():
    checkConditions()
    initializeSerial()


def loop():
    global receiving_timer, publish_timer, TICK_1, TICK_2
    rclpy.init()

    motor_driver_node = MotorDriverNode(NODE_NAME)

    MOTOR_1.resetDataCount()
    MOTOR_2.resetDataCount()

    # Record data
    index = 0

    # Test
    test_dict = {
        "motor_data": [TEST_PWM_FREQUENCY, TEST_PWM, TEST_PWM_FREQUENCY, TEST_PWM]
    }
    MCUSerialObject.write(formSerialData(json.dumps(test_dict)))

    try:
        while index <= DATA_AMOUNT:
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

            # Vary PWM
            if 0 < index <= DATA_AMOUNT / 3:
                varyPWM(1023)
            elif DATA_AMOUNT / 3 < index <= DATA_AMOUNT * 2 / 3:
                varyPWM(714)
            else:
                varyPWM(510)

            MOTOR_1.calculateRPM(TICK_1)
            MOTOR_2.calculateRPM(TICK_2)
            rclpy.spin_once(motor_driver_node)

            if index != MOTOR_1.getDataCount():
                print(str(index) + "/" + str(DATA_AMOUNT))
                index += 1
                WORKBOOK.writeData(index + 1, 1, MOTOR_1.getLowPassRPM())
                WORKBOOK.writeData(index + 1, 2, MOTOR_1.getKalmanFilterRPM())
                WORKBOOK.writeData(index + 1, 4, MOTOR_2.getLowPassRPM())
                WORKBOOK.writeData(index + 1, 5, MOTOR_2.getKalmanFilterRPM())

    except KeyboardInterrupt:
        # JSON
        MCUSerialObject.write(formSerialData("{motor_data:[1000,0,1000,0]}"))
        MCUSerialObject.close()
        WORKBOOK.saveWorkBook()

    finally:
        MCUSerialObject.write(formSerialData("{motor_data:[1000,0,1000,0]}"))
        MCUSerialObject.close()
        WORKBOOK.saveWorkBook()


def main():
    setup()
    loop()


if __name__ == "__main__":
    main()
