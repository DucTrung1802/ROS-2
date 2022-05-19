#!/usr/bin/python3

# adapted from https://github.com/recantha/EduKit3-RC-Keyboard/blob/master/rc_keyboard.py

import sys, termios, tty, os, time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from pathlib import Path
import math

# OS parameters
CLEAR = lambda: os.system("clear")

# Node parameters
NODE_NAME = Path(__file__).stem
PUBLISH_FREQUENCY = 100
TOPIC = "/cmd_vel"

# Key parameters
BUTTON_DELAY = 0.001

# Controling parameters
STEP = 1

# Motor parameters
LEFT_MOTOR_DIAMETER = 0.09
RIGHT_MOTOR_DIAMETER = 0.09


# Velocity: m/s
LINEAR_SPEED_MAX = 0.6  # m/s
LINEAR_SPEED_MIN = -LINEAR_SPEED_MAX  # m/s
LINEAR_SPEED_STEP = LINEAR_SPEED_MAX / STEP

# Degree: positive-clockwise; negative-counterclockwise
ANGULAR_SPPEED_MAX = 1.0  # rad/s
ANGULAR_SPPEED_MIN = -ANGULAR_SPPEED_MAX  # rad/s
ANGULAR_SPPEED_STEP = ANGULAR_SPPEED_MAX / STEP  # rad/s

msgInfo = ""
twist = Twist()
key = None
status = 0
target_linear_velocity = 0.0
target_angular_velocity = 0.0
control_linear_velocity = 0.0
control_angular_velocity = 0.0


def MPStoRPM(mps):
    return mps / (LEFT_MOTOR_DIAMETER * math.pi) * 60


def RPMtoMPS(rpm):
    return rpm * (LEFT_MOTOR_DIAMETER * math.pi) / 60


DASHBOARD = """
Manually Control ServingBot!
---------------------------
MAX LINEAR VELOCITY : {} m/s
MAX ANGULAR VELOCITY : {} rad/s

Moving around:
        w    
    a   s   d
        x   

w/x : increase/decrease linear speed by {} m/s
a/d : increase/decrease angular speed by {} rad/s
s : force stop

p: quit

---------------------------

""".format(
    round(LINEAR_SPEED_MAX, 2),
    round(ANGULAR_SPPEED_MAX, 2),
    round(LINEAR_SPEED_STEP, 2),
    round(ANGULAR_SPPEED_STEP, 2),
)


def checkParametersCondition():
    if PUBLISH_FREQUENCY <= 0:
        raise Exception("PUBLISH_FREQUENCY is smaller than zero (Default: 10).")


def displayInstruction():
    CLEAR()
    print(DASHBOARD)


def initializeTwist():
    global twist
    twist.linear.x = 0.0
    twist.linear.y = 0.0
    twist.linear.z = 0.0
    twist.angular.x = 0.0
    twist.angular.y = 0.0
    twist.angular.z = 0.0


def getKey():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        ch = sys.stdin.read(1)

    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch


def checkLinearLimitVelocity(input):

    if input <= LINEAR_SPEED_MIN:
        input = LINEAR_SPEED_MIN
    elif input >= LINEAR_SPEED_MAX:
        input = LINEAR_SPEED_MAX
    elif abs(input) < (LINEAR_SPEED_STEP / 20):
        input = 0.0
    else:
        input = input

    return input


def checkAngularLimitVelocity(input):

    if input <= ANGULAR_SPPEED_MIN:
        input = ANGULAR_SPPEED_MIN
    elif input >= ANGULAR_SPPEED_MAX:
        input = ANGULAR_SPPEED_MAX
    elif abs(input) < (ANGULAR_SPPEED_STEP / 20):
        input = 0.0
    else:
        input = input

    return input


def controlMotors():
    global key, target_linear_velocity, target_angular_velocity
    key = getKey()

    # Increase linear velocity
    if key == "w":
        target_linear_velocity = checkLinearLimitVelocity(
            target_linear_velocity + LINEAR_SPEED_STEP
        )
        updateRosInfo(key)

    # Decrease linear velocity
    elif key == "x":
        target_linear_velocity = checkLinearLimitVelocity(
            target_linear_velocity - LINEAR_SPEED_STEP
        )
        updateRosInfo(key)

    # Increase angular velocity - Turn left
    elif key == "a":
        target_angular_velocity = checkAngularLimitVelocity(
            target_angular_velocity + ANGULAR_SPPEED_STEP
        )
        updateRosInfo(key)

    # Decrease angular velocity - Turn right
    elif key == "d":
        target_angular_velocity = checkAngularLimitVelocity(
            target_angular_velocity - ANGULAR_SPPEED_STEP
        )
        updateRosInfo(key)

    # Stop immediately
    elif key == "s":
        target_linear_velocity = 0.0
        target_angular_velocity = 0.0
        updateRosInfo(key)

    # Terminate node
    elif key == "p" or key == "\x03":
        print("Terminated the node!")
        exit(0)

    time.sleep(BUTTON_DELAY)


def updateMessage():
    global twist
    twist.linear.x = target_linear_velocity
    twist.angular.z = target_angular_velocity


def updateRosInfo(msg):
    global msgInfo
    msgInfo = msg


class Publisher(Node):
    def __init__(self):
        super().__init__("motor_controller")
        self.publisher_ = self.create_publisher(Twist, TOPIC, 1)
        timer_period = 1 / PUBLISH_FREQUENCY  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        self.publisher_.publish(twist)
        controlMotors()
        updateMessage()
        displayInstruction()
        self.get_logger().info(
            "twist.linear.x = " + str(round(twist.linear.x, 2)) + " m/s"
        )
        self.get_logger().info(
            "twist.angular.z = " + str(round(twist.angular.z, 2)) + " rad/s"
        )
        self.get_logger().info("-----")
        self.get_logger().info(
            "twist.linear.x = " + str(round(MPStoRPM(twist.linear.x), 2)) + " RPM"
        )
        self.get_logger().info(
            "twist.angular.z = " + str(round(twist.angular.z, 2)) + " rad/s"
        )


def main(args=None):
    checkParametersCondition()
    rclpy.init(args=args)

    displayInstruction()
    initializeTwist()

    minimal_publisher = Publisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
