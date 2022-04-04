#!/usr/bin/python3

# adapted from https://github.com/recantha/EduKit3-RC-Keyboard/blob/master/rc_keyboard.py

import sys, termios, tty, os, time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from pathlib import Path

# Node parameters
NODE_NAME = Path(__file__).stem
PUBLISH_FREQUENCY = 10
TOPIC = "/cmd_vel"

# Key parameters
BUTTON_DELAY = 0.001

# Controling parameters
STEP = 3

# Velocity: m/s
LINEAR_SPEED_MAX = 0.6  # m/s
LINEAR_SPEED_MIN = -LINEAR_SPEED_MAX
LINEAR_SPEED_STEP = LINEAR_SPEED_MAX / STEP

# Degree: positive-clockwise; negative-counterclockwise
ANGULAR_SPPEED_MAX = 1.0  # rad/s
ANGULAR_SPPEED_MIN = -ANGULAR_SPPEED_MAX  # rad/s
ANGULAR_SPPEED_STEP = ANGULAR_SPPEED_MAX / STEP  # rad/s

DISPLAY_INSTRUCTION = """
Control Your Turtlebot!
---------------------------
Moving around:
        w    
    a   s   d
        x    

w/x : increase/decrease linear speed by {} m/s
a/d : increase/decrease angular speed by {} rad/s
s : force stop

CTRL-C to quit
""".format(
    round(LINEAR_SPEED_STEP, 2), round(ANGULAR_SPPEED_STEP, 2)
)

msgInfo = ""
twist = Twist()
key = None
status = 0
target_linear_velocity = 0.0
target_angular_velocity = 0.0
control_linear_velocity = 0.0
control_angular_velocity = 0.0


def checkParametersCondition():
    if PUBLISH_FREQUENCY <= 0:
        raise Exception("PUBLISH_FREQUENCY is smaller than zero (Default: 10).")


def displayInstruction():
    print(DISPLAY_INSTRUCTION)


def resetTwist():
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


def driveMotors():
    global key, target_linear_velocity, target_angular_velocity
    key = getKey()

    # Increase linear velocity
    if key == "w":
        target_linear_velocity = checkLinearLimitVelocity(
            target_linear_velocity + LINEAR_SPEED_STEP
        )

    # Decrease linear velocity
    elif key == "x":
        target_linear_velocity = checkLinearLimitVelocity(
            target_linear_velocity - LINEAR_SPEED_STEP
        )

    # Increase angular velocity - Turn left
    elif key == "a":
        target_angular_velocity = checkAngularLimitVelocity(
            target_angular_velocity + ANGULAR_SPPEED_STEP
        )

    # Decrease angular velocity - Turn right
    elif key == "d":
        target_angular_velocity = checkAngularLimitVelocity(
            target_angular_velocity - ANGULAR_SPPEED_STEP
        )

    # Stop immediately
    elif key == "s":
        resetTwist()

    # Terminate node
    elif key == "p":
        updateRosInfo("Terminated the node!")
        exit(0)

    updateRosInfo(key)
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
        super().__init__("minimal_publisher")
        self.publisher_ = self.create_publisher(Twist, TOPIC, 10)
        timer_period = 0.01  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        driveMotors()
        updateMessage()
        self.publisher_.publish(twist)
        self.get_logger().info("twist.linear.x = " + str(twist.linear.x))
        self.get_logger().info("twist.angular.z = " + str(twist.angular.z))


def main(args=None):
    checkParametersCondition()

    rclpy.init(args=args)

    resetTwist()
    minimal_publisher = Publisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
