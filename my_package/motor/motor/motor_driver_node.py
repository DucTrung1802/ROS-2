#!/usr/bin/python3

# Python libraries
import subprocess
import time
from click import prompt
import serial
import re
import json
import math
import copy
import hashlib
import numpy as np
import threading

# ROS 2 libraries
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

# User-defined class
from motor.MotorDriver import MotorDriver
from motor.DataRecoder import DataRecoder
from motor.PIDController import PIDController
from motor.ReadLine import ReadLine
from motor.PoseCalculator import PoseCalculator

# =========== Configurable parameters =============
# Serial parameters
SKIP_SERIAL_LINES = 12
LIDAR_USB_NAME = "FTDI USB Serial Device"
MCU_USB_NAME = "cp210x"
BAUD_RATE = 921600
RECEIVING_FREQUENCY = 500

# Node parameters
PUBLISH_FREQUENCY = 30
NODE_NAME = "motor_driver"

# Motor parameters
LEFT_MOTOR_MAX_RPM = 190
RIGHT_MOTOR_MAX_RPM = 190

WHEEL_BASE = 0.44

LEFT_MOTOR_DIAMETER = 0.09  # m
LEFT_MOTOR_PULSE_PER_ROUND_OF_ENCODER = 480  # ticks
LEFT_MOTOR_PWM_FREQUENCY = 1000  # Hz
LEFT_MOTOR_SAMPLE_TIME = 0.005  # s

RIGHT_MOTOR_DIAMETER = 0.09  # m
RIGHT_MOTOR_PULSE_PER_ROUND_OF_ENCODER = 480  # ticks
RIGHT_MOTOR_PWM_FREQUENCY = 1000  # Hz
RIGHT_MOTOR_SAMPLE_TIME = 0.005  # s

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
LEFT_MOTOR_Kp = 0.65
LEFT_MOTOR_Ki = 6.5
LEFT_MOTOR_Kd = 0.0032
LEFT_MOTOR_MIN = 0
LEFT_MOTOR_MAX = 12

RIGHT_MOTOR_Kp = 0.73
RIGHT_MOTOR_Ki = 7.1
RIGHT_MOTOR_Kd = 0.0036
RIGHT_MOTOR_MIN = 0
RIGHT_MOTOR_MAX = 12


# Test data
TEST_ONLY_ON_LAPTOP = True
MANUALLY_TUNE_PID = False
DATA_RECORDING = False
DIRECTION_LEFT = 1
DIRECTION_RIGHT = 1
TEST_PWM_FREQUENCY = 1000
TEST_PWM = 511

# DataRecorder parameters
DATA_AMOUNT = 700

if not (float(WHEEL_BASE) and WHEEL_BASE > 0):
    raise Exception("Invalid value of wheel base length!")

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
pwm_left = 0.0
pwm_right = 0.0


KINEMATICS_MODEL_MATRIX = np.matrix(
    [
        [RIGHT_MOTOR_DIAMETER / 4, LEFT_MOTOR_DIAMETER / 4],
        [
            RIGHT_MOTOR_DIAMETER / (2 * WHEEL_BASE),
            -LEFT_MOTOR_DIAMETER / (2 * WHEEL_BASE),
        ],
    ]
)

try:
    INVERSE_KINEMATICS_MODEL_MATRIX = KINEMATICS_MODEL_MATRIX.I
except:
    raise Exception("Cannot calculate inverse of kinematic model matrix!")

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
timer_test_PID = 0
step_test_PID = 0

# PoseCalculator instance
POSE_CALCULATOR = PoseCalculator(
    radius=LEFT_MOTOR.getRadius,
    left_wheel_tick_per_round=LEFT_MOTOR.getTickPerRoundOfEncoder(),
    right_wheel_tick_per_round=RIGHT_MOTOR.getTickPerRoundOfEncoder(),
    wheel_base=WHEEL_BASE,
)

# Node parameters
linear_velocity = 0
previous_linear_velocity = 0
current_state_is_straight = True
previous_current_state_is_straight = True
linear_RPM_left = 0
linear_RPM_right = 0
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
total_receive = 0
error_receive = 0

# JSON parameters
KEY = "pwm_pulse"
STORE_LEFT_TICK = 0
STORE_RIGHT_TICK = 0
STORE_LEFT_RPM = 0
STORE_RIGHT_RPM = 0
STORE_CHECKSUM = ""
LEFT_TICK = 0
RIGHT_TICK = 0
LEFT_RPM = 0
RIGHT_RPM = 0
CHECKSUM = ""

# Publishing dictionary
odom_dictionary = {
    "pose": {
        "pose": {
            "position": {"x": 0.0, "y": 0.0, "z": 0.0},
            "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
        }
    },
    "twist": {
        "twist": {
            "linear": {"x": 0.0, "y": 0.0, "z": 0.0},
            "angular": {"x": 0.0, "y": 0.0, "z": 0.0},
        }
    },
}


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
        self.odom_pub = self.create_publisher(Odometry, "/wheel/odometry", 1)
        # self.left_tick_pub = self.create_publisher(Int32, "left_tick", 1)
        # self.right_tick_pub = self.create_publisher(Int32, "right_tick", 1)
        # self.left_RPM_pub = self.create_publisher(Float32, "left_RPM", 1)
        # self.right_RPM_pub = self.create_publisher(Float32, "right_RPM", 1)
        self.timer = self.create_timer(PUBLISH_PERIOD, self.publisherCallback)

        self.controller_sub = self.create_subscription(
            Twist, "cmd_vel", self.subscriberCallback, 1
        )
        self.controller_sub  # prevent unused variable warning

    def publisherCallback(self):

        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "odom"

        msg.child_frame_id = "idunno"

        msg.pose.pose.position.x = odom_dictionary["pose"]["pose"]["position"]["x"]
        msg.pose.pose.position.y = odom_dictionary["pose"]["pose"]["position"]["y"]
        msg.pose.pose.position.z = odom_dictionary["pose"]["pose"]["position"]["z"]

        msg.pose.pose.orientation.x = odom_dictionary["pose"]["pose"]["orientation"][
            "x"
        ]
        msg.pose.pose.orientation.y = odom_dictionary["pose"]["pose"]["orientation"][
            "y"
        ]
        msg.pose.pose.orientation.z = odom_dictionary["pose"]["pose"]["orientation"][
            "z"
        ]
        msg.pose.pose.orientation.w = odom_dictionary["pose"]["pose"]["orientation"][
            "w"
        ]

        msg.twist.twist.linear.x = odom_dictionary["twist"]["twist"]["linear"]["x"]
        msg.twist.twist.linear.y = odom_dictionary["twist"]["twist"]["linear"]["y"]
        msg.twist.twist.linear.z = odom_dictionary["twist"]["twist"]["linear"]["z"]

        msg.twist.twist.angular.x = odom_dictionary["twist"]["twist"]["angular"]["x"]
        msg.twist.twist.angular.y = odom_dictionary["twist"]["twist"]["angular"]["y"]
        msg.twist.twist.angular.z = odom_dictionary["twist"]["twist"]["angular"]["z"]

        self.odom_pub.publish(msg)

        # left_RPM = Float32()
        # right_RPM = Float32()
        # left_RPM.data = float(LEFT_RPM)
        # right_RPM.data = float(RIGHT_RPM)
        # self.left_RPM_pub.publish(left_RPM)
        # self.right_RPM_pub.publish(right_RPM)

        # left_tick = Int32()
        # right_tick = Int32()
        # left_tick.data = int(LEFT_TICK)
        # right_tick.data = int(RIGHT_TICK)
        # self.left_tick_pub.publish(left_tick)
        # self.right_tick_pub.publish(right_tick)

        # self.get_logger().info('Publishing: "%s"' % msg.data)

    def subscriberCallback(self, msg):
        setupSetpoint(msg)


def MPStoRPM(mps):
    return mps / (LEFT_MOTOR_DIAMETER * math.pi) * 60


def RPMtoMPS(rpm):
    return rpm * (LEFT_MOTOR_DIAMETER * math.pi) / 60


def RAD_PER_SEC_to_RPM(matrix):
    return matrix * (60 / (2 * math.pi))


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


def differientialDriveCalculate(linear_velocity, angular_velocity):
    velocity_matrix = np.matrix([[linear_velocity], [angular_velocity]])
    RPM_matrix = INVERSE_KINEMATICS_MODEL_MATRIX.dot(velocity_matrix)
    return RPM_matrix


def setupSetpoint(msg):
    global previous_linear_velocity, current_state_is_straight, previous_current_state_is_straight
    global linear_velocity, linear_RPM_left, linear_RPM_right
    global odom_dictionary
    # Evaluate to have RPM value

    linear_velocity = msg.linear.x  # m/s
    angular_velocity = msg.angular.z  # rad/s
    odom_dictionary["twist"]["twist"]["linear"]["x"] = msg.linear.x
    odom_dictionary["twist"]["twist"]["angular"]["z"] = msg.angular.z

    differiential_drive_matrix = differientialDriveCalculate(
        linear_velocity, angular_velocity
    )

    differiential_drive_matrix = RAD_PER_SEC_to_RPM(differiential_drive_matrix)

    # Differential drive
    linear_RPM_right = differiential_drive_matrix.item(0)
    linear_RPM_left = differiential_drive_matrix.item(1)

    linear_RPM_left = saturate(linear_RPM_left, -LEFT_MOTOR_MAX_RPM, LEFT_MOTOR_MAX_RPM)

    linear_RPM_right = saturate(
        linear_RPM_right, -RIGHT_MOTOR_MAX_RPM, RIGHT_MOTOR_MAX_RPM
    )

    # print(differiential_drive_matrix)

    # print("linear_RPM_left: " + str(linear_RPM_left))
    # print("linear_RPM_right: " + str(linear_RPM_right))

    # Testing
    # linear_RPM_left = linear_RPM_left * 1023 / LEFT_MOTOR_MAX_RPM
    # linear_RPM_right = linear_RPM_right * 1023 / RIGHT_MOTOR_MAX_RPM


def driveMotors():
    global linear_RPM_left, linear_RPM_right
    global pwm_left, pwm_right

    # print("linear_RPM_left: " + str(linear_RPM_left))
    # print("linear_RPM_right: " + str(linear_RPM_right))

    # linear_RPM_left = 42.4
    # linear_RPM_right = -42.4

    start = time.time()

    direction_1 = getDirection(linear_RPM_left)
    direction_2 = getDirection(linear_RPM_right)

    linear_RPM_left_abs = abs(linear_RPM_left)
    linear_RPM_right_abs = abs(linear_RPM_right)

    pwm_freq_1 = LEFT_MOTOR.getPWMFrequency()
    pwm_freq_2 = RIGHT_MOTOR.getPWMFrequency()

    LEFT_MOTOR_PID_CONTROLLER.evaluate(linear_RPM_left_abs, LEFT_RPM)
    RIGHT_MOTOR_PID_CONTROLLER.evaluate(linear_RPM_right_abs, RIGHT_RPM)

    if linear_RPM_left_abs == 0:
        pwm_left = 0.0
    elif linear_RPM_left_abs > 0:
        pwm_left = LEFT_MOTOR_PID_CONTROLLER.getOutputValue() * 1023.0 / 12.0

    if linear_RPM_right_abs == 0:
        pwm_right = 0.0
    elif linear_RPM_right_abs > 0:
        pwm_right = RIGHT_MOTOR_PID_CONTROLLER.getOutputValue() * 1023.0 / 12.0

    # print("---")
    # print("Left PWM: " + str(pwm_left) + "; Left RPM: " + str(LEFT_RPM))
    # print("Right PWM: " + str(pwm_right) + "; Right RPM: " + str(RIGHT_RPM))
    # print("---")

    data = {
        "motor_data": [
            direction_1,
            pwm_freq_1,
            pwm_left,
            direction_2,
            pwm_freq_2,
            pwm_right,
        ]
    }

    data = json.dumps(data)
    # print(data)
    MCUSerialObject.write(formSerialData(data))

    end = time.time()

    # print(end - start)

    start = time.time()

    if LEFT_MOTOR.getSampleTime() - (end - start) >= 0:
        time.sleep((LEFT_MOTOR.getSampleTime() - (end - start)) * 5 / 5.11)

    end = time.time()

    # print(end - start)


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
    global MCUSerialObject, improved_serial
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
    improved_serial = ReadLine(MCUSerialObject)

    while skipLines:
        # Skip some lines of serial when MCU is reseted
        MCUSerialObject.readline()
        skipLines = skipLines - 1

    time.sleep(0.01)


def readSerialData():
    global serialData, dictionaryData
    rawData = improved_serial.readline()
    # print(rawData)
    serialData = rawData.decode("utf-8")  # decode s
    # print(len(serialData))
    # filteredSerialData = re.sub(
    #     "[^A-Za-z0-9\s[]{}]", "", serialData
    # )  # filter regular characters
    # print(filteredSerialData)
    try:
        dictionaryData = json.loads(serialData)
    except:
        return


def checksum():
    global error_receive
    dictionaryDataCheck = copy.deepcopy(dictionaryData)
    dictionaryDataCheck.pop("ck", None)
    dictionaryDataCheckString = json.dumps(dictionaryDataCheck)
    dictionaryDataCheckString = dictionaryDataCheckString.replace(" ", "")
    checksumString = hashlib.md5(dictionaryDataCheckString.encode()).hexdigest()

    # print("Dict: " + dictionaryDataCheckString)

    # print("STORE_CHECKSUM: " + STORE_CHECKSUM)
    # print("dictionaryDataCheck: " + checksumString)
    # print("---------")

    # print("STRING_SENT: |" + serialData + "|")
    # print("STRING_CHECK: |" + dictionaryDataCheckString + "|")
    # print("---------")

    if checksumString != STORE_CHECKSUM:
        error_receive += 1


def updateStoreRPMFromSerial():
    global STORE_LEFT_TICK, STORE_RIGHT_TICK, STORE_LEFT_RPM, STORE_RIGHT_RPM, STORE_CHECKSUM, total_receive, error_receive
    # MCUSerialObject.write(formSerialData("{pwm_pulse:[1023,1023]}"))
    # print(
    #     "Error in serial communication: " + str(error_receive) + "/" + str(total_receive)
    # )
    try:
        readSerialData()
        STORE_LEFT_TICK = dictionaryData["lt"]
        STORE_RIGHT_TICK = dictionaryData["rt"]
        STORE_LEFT_RPM = dictionaryData["lR"]
        STORE_RIGHT_RPM = dictionaryData["rR"]
        STORE_CHECKSUM = dictionaryData["ck"]
        checksum()

    except:
        error_receive += 1
    finally:
        total_receive += 1
        return


def updateRPMFromStorePos():
    global LEFT_TICK, RIGHT_TICK, LEFT_RPM, RIGHT_RPM, CHECKSUM
    LEFT_TICK = STORE_LEFT_TICK
    RIGHT_TICK = STORE_RIGHT_TICK
    LEFT_RPM = STORE_LEFT_RPM
    RIGHT_RPM = STORE_RIGHT_RPM
    CHECKSUM = STORE_CHECKSUM


def updatePublishDictionary():
    global odom_dictionary
    pose = POSE_CALCULATOR.getOutputPose()
    odom_dictionary["pose"]["pose"]["position"]["x"] = pose[0][0]
    odom_dictionary["pose"]["pose"]["position"]["y"] = pose[0][1]
    odom_dictionary["pose"]["pose"]["position"]["z"] = pose[0][2]

    odom_dictionary["pose"]["pose"]["orientation"]["x"] = pose[1][0]
    odom_dictionary["pose"]["pose"]["orientation"]["y"] = pose[1][1]
    odom_dictionary["pose"]["pose"]["orientation"]["z"] = pose[1][2]
    odom_dictionary["pose"]["pose"]["orientation"]["w"] = pose[1][3]


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


def testPIDResponse(max_range, step, time_interval=0):
    global linear_velocity, linear_RPM_left, linear_RPM_right
    global timer_test_PID, step_test_PID
    if max_range <= 0:
        raise Exception("max_range must be positive number!")
    if step <= 0:
        raise Exception("step must be positive number!")
    if time_interval < 0:
        raise Exception("time_interval must be positive number!")

    if time.time() - timer_test_PID >= time_interval:
        step_test_PID += max_range / step
        step_test_PID = saturate(step_test_PID, 0, max_range)
        linear_velocity = step_test_PID
        linear_RPM_left = MPStoRPM(linear_velocity)
        linear_RPM_right = MPStoRPM(linear_velocity)
        timer_test_PID = time.time()


def stopAllThreads():
    global flag_1, flag_2, flag_3, flag_4, flag_5

    flag_1 = True
    flag_2 = True
    flag_3 = True
    flag_4 = True
    flag_5 = True


def task_1():
    global flag_1
    while True:

        if flag_1:
            break

        updateStoreRPMFromSerial()
        updateRPMFromStorePos()
        POSE_CALCULATOR.calculatePose(LEFT_TICK, RIGHT_TICK)
        updatePublishDictionary()


def task_2():
    global flag_2
    while True:

        if flag_2:
            break

        comp_start = time.time()

        if not DATA_RECORDING:
            driveMotors()

        comp_end = time.time()


def task_3():
    global flag_3
    rclpy.init()
    motor_driver_node = MotorDriverNode(NODE_NAME)
    print("Start Publishing")
    while True:

        if flag_3:
            break

        rclpy.spin(motor_driver_node)


def task_4():
    global flag_4

    index = 1
    delta_time = 0
    time.sleep(1)

    print("Ready")

    # All testing must be after the "Ready" line!
    # ============ TESTING ============

    varyPWM(TEST_PWM)

    # =================================

    WORKBOOK.writeData(index + 1, 1, delta_time)

    while index <= DATA_AMOUNT:

        start = time.time()

        comp_start = time.time()

        WORKBOOK.writeData(index + 1, 2, linear_RPM_left)
        WORKBOOK.writeData(index + 1, 3, LEFT_RPM)
        WORKBOOK.writeData(index + 1, 4, pwm_left / 1023.0 * 12.0)
        WORKBOOK.writeData(index + 1, 6, linear_RPM_right)
        WORKBOOK.writeData(index + 1, 7, RIGHT_RPM)
        WORKBOOK.writeData(index + 1, 8, pwm_right / 1023.0 * 12.0)
        WORKBOOK.writeData(index + 1, 9, total_receive)
        WORKBOOK.writeData(index + 1, 10, error_receive)
        WORKBOOK.writeData(
            index + 1,
            11,
            round((total_receive - error_receive) / total_receive * 100, 2),
        )

        index += 1

        comp_end = time.time()

        if comp_end - comp_start <= LEFT_MOTOR_SAMPLE_TIME:
            # Run a test code to find the exceeding of time.sleep() then multiply the coefficient again
            target_time = (
                time.time()
                + (LEFT_MOTOR_SAMPLE_TIME - (comp_end - comp_start)) * 5 / 5.6
            )
            while time.time() <= target_time:
                time.sleep(0.0001)

        end = time.time()

        WORKBOOK.writeData(index + 1, 1, end - start)

        # print(delta_time)

    stopAllThreads()


def manuallyTunePID():
    print("Tune PID")

    print("=== LEFT MOTOR ===")
    left_Kp = float(input("Kp = "))
    left_Ki = float(input("Ki = "))
    left_Kd = float(input("Kd = "))

    print("=== RIGHT MOTOR ===")
    right_Kp = float(input("Kp = "))
    right_Ki = float(input("Ki = "))
    right_Kd = float(input("Kd = "))

    LEFT_MOTOR_PID_CONTROLLER.changeCoefficients(
        left_Kp, left_Ki, left_Kd, LEFT_MOTOR.getSampleTime()
    )
    RIGHT_MOTOR_PID_CONTROLLER.changeCoefficients(
        right_Kp, right_Ki, right_Kd, RIGHT_MOTOR.getSampleTime()
    )

    print()

    print("Setup setpoint")
    max_RPM = float(input("Max RPM = "))
    step = float(input("Step = "))
    time_interval = float(input("Time interval for each setpoint (s) = "))
    run_time = step * time_interval

    return [max_RPM, step, time_interval, run_time]


def task_5():
    global flag_5

    index = 0
    delta_time = 0
    time.sleep(1)

    print("Ready")

    # All testing must be after the "Ready" line!
    # ============ TESTING ============

    max_RPM, step, time_interval, run_time = manuallyTunePID()

    if run_time <= 0:
        stopAllThreads()
        raise Exception("Run time must be positive!")

    timer = time.time()
    # varyPWM(TEST_PWM)

    # =================================

    while time.time() - timer <= run_time:

        testPIDResponse(
            max_range=RPMtoMPS(max_RPM), step=step, time_interval=time_interval
        )

        start = time.time()

        comp_start = time.time()

        WORKBOOK.writeData(index + 1, 1, delta_time)
        WORKBOOK.writeData(index + 1, 2, linear_RPM_left)
        WORKBOOK.writeData(index + 1, 3, LEFT_RPM)
        # WORKBOOK.writeData(index + 1, 4, pwm_left / 1023.0 * 12.0)
        WORKBOOK.writeData(index + 1, 4, LEFT_MOTOR_PID_CONTROLLER.getKi())
        WORKBOOK.writeData(index + 1, 6, linear_RPM_right)
        WORKBOOK.writeData(index + 1, 7, RIGHT_RPM)
        # WORKBOOK.writeData(index + 1, 8, pwm_right / 1023.0 * 12.0)
        WORKBOOK.writeData(index + 1, 8, RIGHT_MOTOR_PID_CONTROLLER.getKi())
        WORKBOOK.writeData(index + 1, 9, total_receive)
        WORKBOOK.writeData(index + 1, 10, error_receive)
        WORKBOOK.writeData(
            index + 1,
            11,
            round((total_receive - error_receive) / total_receive * 100, 2),
        )
        index += 1

        comp_end = time.time()

        if LEFT_MOTOR_SAMPLE_TIME - (comp_end - comp_start) >= 0:
            time.sleep(LEFT_MOTOR_SAMPLE_TIME - (comp_end - comp_start))

        end = time.time()

        delta_time = end - start

    stopAllThreads()


def threadingHandler():
    global flag_1, flag_2, flag_3, flag_4, flag_5

    # Setup flags (Thread will be stopped when the flag is set to True)
    flag_1 = False
    flag_2 = False
    flag_3 = False
    flag_4 = False
    flag_5 = False

    # Create threads
    thread_1 = threading.Thread(target=task_1)
    thread_2 = threading.Thread(target=task_2)
    thread_3 = threading.Thread(target=task_3)

    if DATA_RECORDING:
        thread_4 = threading.Thread(target=task_4)

    if MANUALLY_TUNE_PID:
        thread_5 = threading.Thread(target=task_5)

    # Start threads
    if not TEST_ONLY_ON_LAPTOP:
        thread_1.start()
        thread_2.start()
        thread_3.start()

    if DATA_RECORDING:
        thread_4.start()

    if MANUALLY_TUNE_PID:
        # pass
        thread_5.start()

    # Wait for all threads to stop
    if not TEST_ONLY_ON_LAPTOP:
        thread_1.join()
        thread_2.join()

    thread_3.join()

    if DATA_RECORDING:
        thread_4.join()

    if MANUALLY_TUNE_PID:
        # pass
        thread_5.join()

    # Do something after all threads stop
    # do_something()


def setup():
    checkConditions()
    if not TEST_ONLY_ON_LAPTOP:
        initializeSerial()


def loop():
    global receiving_timer, publish_timer, LEFT_RPM, RIGHT_RPM

    LEFT_MOTOR.resetDataCount()
    RIGHT_MOTOR.resetDataCount()

    try:
        if DATA_RECORDING:
            varyPWM(0)

            threadingHandler()

        else:
            threadingHandler()

    except KeyboardInterrupt:
        # JSON
        print("Captured Ctrl + C")
        if not TEST_ONLY_ON_LAPTOP:
            MCUSerialObject.write(formSerialData("{motor_data:[0,1000,0,0,1000,0]}"))
            MCUSerialObject.close()
        if DATA_RECORDING or MANUALLY_TUNE_PID:
            WORKBOOK.saveWorkBook()

    finally:
        print("The program has been stopped!")
        if not TEST_ONLY_ON_LAPTOP:
            MCUSerialObject.write(formSerialData("{motor_data:[0,1000,0,0,1000,0]}"))
            MCUSerialObject.close()
        if DATA_RECORDING or MANUALLY_TUNE_PID:
            WORKBOOK.saveWorkBook()


def main():
    setup()
    loop()


if __name__ == "__main__":
    main()
