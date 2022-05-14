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
from motor.PIDController import PIDController


# =========== Configurable parameters =============
# Serial parameters
SKIP_SERIAL_LINES = 12
LIDAR_USB_NAME = "FTDI USB Serial Device"
MCU_USB_NAME = "cp210x"
BAUD_RATE = 115200
RECEIVING_FREQUENCY = 2000

# Node parameters
PUBLISH_FREQUENCY = 500
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
LEFT_MOTOR_Kp = 0.051
LEFT_MOTOR_Ki = 1.25
LEFT_MOTOR_Kd = 0
LEFT_MOTOR_MIN = 0
LEFT_MOTOR_MAX = 1023

RIGHT_MOTOR_Kp = 0.064
RIGHT_MOTOR_Ki = 1.57
RIGHT_MOTOR_Kd = 0.0002
RIGHT_MOTOR_MIN = 0
RIGHT_MOTOR_MAX = 1023


# Test data
DATA_RECORDING = True
DIRECTION_LEFT = 1
DIRECTION_RIGHT = 1
TEST_PWM_FREQUENCY = 1000
TEST_PWM = 1023

# DataRecorder parameters
DATA_AMOUNT = 50000

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
time_of_receive = 0
error_of_receive = 0

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

    # Testing
    # linear_velocity_left = linear_velocity_left * 1023 / LEFT_MOTOR_MAX_RPM
    # linear_velocity_right = linear_velocity_right * 1023 / RIGHT_MOTOR_MAX_RPM


def driveMotors():
    global linear_velocity, linear_velocity_left, linear_velocity_right
    global drive_timer

    if time.time() - drive_timer >= LEFT_MOTOR.getSampleTime():
        direction = getDirection(linear_velocity)

        pwm_freq_1 = LEFT_MOTOR.getPWMFrequency()
        pwm_freq_2 = RIGHT_MOTOR.getPWMFrequency()

        LEFT_MOTOR_PID_CONTROLLER.evaluate(
            linear_velocity_left, LEFT_MOTOR.getLowPassRPM()
        )
        RIGHT_MOTOR_PID_CONTROLLER.evaluate(
            linear_velocity_right, RIGHT_MOTOR.getLowPassRPM()
        )

        pwm_left = LEFT_MOTOR_PID_CONTROLLER.getOutputValue()
        pwm_right = RIGHT_MOTOR_PID_CONTROLLER.getOutputValue()

        print("---")
        print(
            "Left PWM: "
            + str(pwm_left)
            + "; Left RPM: "
            + str(LEFT_MOTOR.getLowPassRPM())
        )
        print(
            "Right PWM: "
            + str(pwm_right)
            + "; Right RPM: "
            + str(RIGHT_MOTOR.getLowPassRPM())
        )
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


def updateStorePosFromSerial():
    global STORE_TICK_1, STORE_TICK_2, time_of_receive, error_of_receive
    # MCUSerialObject.write(formSerialData("{pwm_pulse:[1023,1023]}"))
    # print(
    #     "Error in serial communication: " + str(error_of_receive) + "/" + str(time_of_receive)
    # )
    try:
        readSerialData()
        STORE_TICK_1 = dictionaryData["left_tick"]
        STORE_TICK_2 = dictionaryData["right_tick"]
        time_of_receive += 1
    except:
        error_of_receive += 1
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
            timer = time.time()
            varyPWM(500)

            while index <= DATA_AMOUNT:

                if index != LEFT_MOTOR.getDataCount():
                    # print(str(index) + "/" + str(DATA_AMOUNT))
                    index += 1

                    # Vary PWM
                    # if 0 < index <= DATA_AMOUNT / 3:
                    #     pwm_value = 1023
                    # elif DATA_AMOUNT / 3 < index <= DATA_AMOUNT * 2 / 3:
                    #     pwm_value = 714
                    # else:
                    #     pwm_value = 510

                    # WORKBOOK.writeData(index + 1, 1, LEFT_MOTOR.getLowPassRPM())
                    # WORKBOOK.writeData(index + 1, 2, LEFT_MOTOR.getKalmanFilterRPM())
                    # WORKBOOK.writeData(index + 1, 4, RIGHT_MOTOR.getLowPassRPM())
                    # WORKBOOK.writeData(index + 1, 5, RIGHT_MOTOR.getKalmanFilterRPM())

                if time.time() - timer >= 5:
                    break

                if time.time() - timer >= LEFT_MOTOR_SAMPLE_TIME:
                    WORKBOOK.writeData(index + 1, 1, LEFT_MOTOR.getLowPassRPM())
                    WORKBOOK.writeData(index + 1, 4, RIGHT_MOTOR.getLowPassRPM())

                if time.time() - receiving_timer >= RECEIVING_PERIOD:
                    updateStorePosFromSerial()
                    receiving_timer = time.time()

                updatePosFromStorePos()

                if time.time() - publish_timer >= PUBLISH_PERIOD:
                    motor_driver_node.setNeedPublish()
                    rclpy.spin_once(motor_driver_node)
                    motor_driver_node.resetNeedPublish()

                    # print("Left tick: " + str(LEFT_MOTOR.getTicks()))
                    # print("Right tick: " + str(RIGHT_MOTOR.getTicks()))

                    publish_timer = time.time()

                LEFT_MOTOR.calculateRPM(TICK_1)
                RIGHT_MOTOR.calculateRPM(TICK_2)
                # driveMotors()
                rclpy.spin_once(motor_driver_node)

        else:
            while True:
                # manuallyWrite()
                if time.time() - receiving_timer >= RECEIVING_PERIOD:
                    updateStorePosFromSerial()
                    receiving_timer = time.time()

                if time.time() - publish_timer >= PUBLISH_PERIOD:
                    updatePosFromStorePos()
                #     motor_driver_node.setNeedPublish()
                #     rclpy.spin_once(motor_driver_node)
                #     motor_driver_node.resetNeedPublish()
                    publish_timer = time.time()

                LEFT_MOTOR.calculateRPM(TICK_1)
                RIGHT_MOTOR.calculateRPM(TICK_2)
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
