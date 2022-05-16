#!/usr/bin/python3

# Python libraries
import subprocess
import time
from click import prompt
from openpyxl import Workbook
import serial
import re
import json
import math
import copy
import hashlib


# ROS 2 libraries
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
from motor.MotorDriver import MotorDriver
from motor.DataRecoder import DataRecoder
from motor.PIDController import PIDController


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
LEFT_MOTOR_MAX_VELOCITY = 0.6  # m/s
RIGHT_MOTOR_MAX_VELOCITY = 0.6  # m/s

LEFT_MOTOR_DIAMETER = 0.09  # m
LEFT_MOTOR_PULSE_PER_ROUND_OF_ENCODER = 480  # ticks
LEFT_MOTOR_PWM_FREQUENCY = 1000  # Hz
LEFT_MOTOR_SAMPLE_TIME = 0.005  # s
left_wheel_RPM_for_1_rad_per_sec_of_robot = 100

RIGHT_MOTOR_DIAMETER = 0.09  # m
RIGHT_MOTOR_PULSE_PER_ROUND_OF_ENCODER = 480  # ticks
RIGHT_MOTOR_PWM_FREQUENCY = 1000  # Hz
RIGHT_MOTOR_SAMPLE_TIME = 0.005  # s
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

# PID Controller parameters
LEFT_MOTOR_Kp = 0.074
LEFT_MOTOR_Ki = 0.43
LEFT_MOTOR_Kd = 0
LEFT_MOTOR_MIN = 0
LEFT_MOTOR_MAX = 12

RIGHT_MOTOR_Kp = 0.06
RIGHT_MOTOR_Ki = 0.46
RIGHT_MOTOR_Kd = 0
RIGHT_MOTOR_MIN = 0
RIGHT_MOTOR_MAX = 12


# Test data
DATA_RECORDING = True
DIRECTION_LEFT = 1
DIRECTION_RIGHT = 1
TEST_PWM_FREQUENCY = 1000
TEST_PWM = 1023

# DataRecorder parameters
DATA_AMOUNT = 2000

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

# PID instances
LEFT_MOTOR_PID_CONTROLLER = PIDController(
    LEFT_MOTOR_Kp,
    LEFT_MOTOR_Ki,
    LEFT_MOTOR_Kd,
    LEFT_MOTOR.getSampleTime(),
    LEFT_MOTOR_MIN,
    LEFT_MOTOR_MAX,
)

RIGHT_MOTOR_PID_CONTROLLER = PIDController(
    RIGHT_MOTOR_Kp,
    RIGHT_MOTOR_Ki,
    RIGHT_MOTOR_Kd,
    RIGHT_MOTOR.getSampleTime(),
    RIGHT_MOTOR_MIN,
    RIGHT_MOTOR_MAX,
)


# Node parameters
receiving_timer = time.time()
publish_timer = time.time()
drive_timer = time.time()
linear_velocity = 0
previous_linear_velocity = 0
current_state_is_straight = True
previous_current_state_is_straight = True
linear_velocity_left = 0
linear_velocity_right = 0
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
successful_receive = 0
error_receive = 0

# JSON parameters
KEY = "pwm_pulse"
STORE_RPM_LEFT = 0
STORE_RPM_RIGHT = 0
STORE_CHECKSUM = ""
RPM_LEFT = 0
RPM_RIGHT = 0
CHECKSUM = ""

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
            RPM_left = Float32()
            RPM_right = Float32()
            RPM_left.data = float(RPM_LEFT)
            RPM_right.data = float(RPM_RIGHT)
            self.left_RPM_pub.publish(RPM_left)
            self.right_RPM_pub.publish(RPM_right)

            # self.get_logger().info('Publishing: "%s"' % msg.data)

    def subscriberCallback(self, msg):
        setupSetpoint(msg)


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


def setupSetpoint(msg):
    global previous_linear_velocity, current_state_is_straight, previous_current_state_is_straight
    global linear_velocity, linear_velocity_left, linear_velocity_right
    # Evaluate to have RPM value

    linear_velocity = MPStoRPM(msg.linear.x)  # RPM
    angular_velocity = msg.angular.z  # rad/s

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

    # Testing
    # linear_velocity_left = linear_velocity_left * 1023 / LEFT_MOTOR_MAX_RPM
    # linear_velocity_right = linear_velocity_right * 1023 / RIGHT_MOTOR_MAX_RPM


def driveMotors():
    global linear_velocity, linear_velocity_left, linear_velocity_right
    global drive_timer, pwm_left, pwm_right

    if time.time() - drive_timer >= LEFT_MOTOR.getSampleTime():
        direction = getDirection(linear_velocity)

        pwm_freq_1 = LEFT_MOTOR.getPWMFrequency()
        pwm_freq_2 = RIGHT_MOTOR.getPWMFrequency()

        LEFT_MOTOR_PID_CONTROLLER.evaluate(linear_velocity_left, RPM_LEFT)
        RIGHT_MOTOR_PID_CONTROLLER.evaluate(linear_velocity_right, RPM_RIGHT)

        if linear_velocity_left == 0:
            pwm_left = 0.0
        elif linear_velocity_left > 0:
            pwm_left = LEFT_MOTOR_PID_CONTROLLER.getOutputValue() * 1023.0 / 12.0

        if linear_velocity_right == 0:
            pwm_right = 0.0
        elif linear_velocity_right > 0:
            pwm_right = RIGHT_MOTOR_PID_CONTROLLER.getOutputValue() * 1023.0 / 12.0

        print("---")
        print("Left PWM: " + str(pwm_left) + "; Left RPM: " + str(RPM_LEFT))
        print("Right PWM: " + str(pwm_right) + "; Right RPM: " + str(RPM_RIGHT))
        print("---")

        data = {
            "motor_data": [
                direction,
                pwm_freq_1,
                pwm_left,
                direction,
                pwm_freq_2,
                pwm_right,
            ]
        }

        data = json.dumps(data)
        MCUSerialObject.write(formSerialData(data))

        drive_timer = time.time()


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
                # print("MCU in serial: " + device.split()[3])
                # print(device)
                foundMCU = True
                index = device.find("ttyUSB")
                # print(index)
                MCUSerial = device[index : index + 7]
                # print(MCUSerial)
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


def checksum():
    global error_receive
    dictionaryDataCheck = copy.deepcopy(dictionaryData)
    dictionaryDataCheck.pop("checksum", None)
    dictionaryDataCheckString = json.dumps(dictionaryDataCheck)
    dictionaryDataCheckString.replace(" ", "")
    checksumString = hashlib.md5(dictionaryDataCheckString.encode()).hexdigest()
    if checksumString != STORE_CHECKSUM:
        error_receive += 1


def updateStoreRPMFromSerial():
    global STORE_RPM_LEFT, STORE_RPM_RIGHT, STORE_CHECKSUM, successful_receive, error_receive
    # MCUSerialObject.write(formSerialData("{pwm_pulse:[1023,1023]}"))
    # print(
    #     "Error in serial communication: " + str(error_receive) + "/" + str(successful_receive)
    # )
    try:
        readSerialData()
        STORE_RPM_LEFT = dictionaryData["left_RPM"]
        STORE_RPM_RIGHT = dictionaryData["right_RPM"]
        STORE_CHECKSUM = dictionaryData["checksum"]
        checksum()
        successful_receive += 1
    except:
        error_receive += 1
        return


def updateRPMFromStorePos():
    global RPM_LEFT, RPM_RIGHT, CHECKSUM
    RPM_LEFT = STORE_RPM_LEFT
    RPM_RIGHT = STORE_RPM_RIGHT
    CHECKSUM = STORE_CHECKSUM


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
    global receiving_timer, publish_timer, RPM_LEFT, RPM_RIGHT
    rclpy.init()

    motor_driver_node = MotorDriverNode(NODE_NAME)

    LEFT_MOTOR.resetDataCount()
    RIGHT_MOTOR.resetDataCount()

    try:
        if DATA_RECORDING:
            index = 1
            timer = time.time()
            save_data_timer = time.time()
            varyPWM(0)

            while index <= DATA_AMOUNT:

                # if time.time() - timer >= 5:
                #     break

                if time.time() - receiving_timer >= RECEIVING_PERIOD:
                    updateStoreRPMFromSerial()
                    receiving_timer = time.time()

                if time.time() - publish_timer >= PUBLISH_PERIOD:
                    updateRPMFromStorePos()
                    motor_driver_node.setNeedPublish()
                    rclpy.spin_once(motor_driver_node)
                    motor_driver_node.resetNeedPublish()

                    publish_timer = time.time()

                driveMotors()
                rclpy.spin_once(motor_driver_node)

                if time.time() - save_data_timer >= LEFT_MOTOR_SAMPLE_TIME:
                    WORKBOOK.writeData(index + 1, 1, linear_velocity_left)
                    WORKBOOK.writeData(index + 1, 2, RPM_LEFT)
                    WORKBOOK.writeData(index + 1, 3, pwm_left / 1023.0 * 12.0)
                    WORKBOOK.writeData(index + 1, 5, linear_velocity_right)
                    WORKBOOK.writeData(index + 1, 6, RPM_RIGHT)
                    WORKBOOK.writeData(index + 1, 7, pwm_right / 1023.0 * 12.0)
                    WORKBOOK.writeData(index + 1, 8, successful_receive)
                    WORKBOOK.writeData(index + 1, 9, error_receive)
                    WORKBOOK.writeData(
                        index + 1,
                        10,
                        round(
                            successful_receive
                            / (error_receive + successful_receive)
                            * 100,
                            2,
                        ),
                    )
                    index += 1
                    save_data_timer = time.time()

        else:
            while True:
                # manuallyWrite()
                if time.time() - receiving_timer >= RECEIVING_PERIOD:
                    updateStoreRPMFromSerial()
                    receiving_timer = time.time()

                if time.time() - publish_timer >= PUBLISH_PERIOD:
                    updateRPMFromStorePos()
                    motor_driver_node.setNeedPublish()
                    rclpy.spin_once(motor_driver_node)
                    motor_driver_node.resetNeedPublish()

                    # print("Left RPM: " + str(RPM_LEFT))
                    # print("Right RPM: " + str(RPM_RIGHT))
                    # print("Checksum: " + CHECKSUM)
                    publish_timer = time.time()

                driveMotors()
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
