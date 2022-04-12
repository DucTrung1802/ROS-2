#!/usr/bin/env python
# license removed for brevity
import json
import time
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import sys, select, os

if os.name == "nt":
    import msvcrt
else:
    import tty, termios

from pathlib import Path

# Parameters:

TOPIC = "/cmd_vel"
NODE_NAME = Path(__file__).stem

STEP = 3

# PWM percentage
LINEAR_SPEED_MAX = 0.6
LINEAR_SPEED_MIN = -0.6
LINEAR_SPEED_STEP = 0.6 / STEP

# Degree: positive-clockwise; negative-counterclockwise
ANGULAR_SPPEED_MAX = 1.0  # rad/s
ANGULAR_SPPEED_MIN = -1.0  # rad/s
ANGULAR_SPPEED_STEP = 1.0 / STEP  # rad/s

display_instruction = """
Control Your Turtlebot!
---------------------------
Moving around:
        w    
    a   s   d
        x    

w/x : increase/decrease only linear speed by {} m/s
a/d : increase/decrease only angular speed by {} %PWM
s : force stop

CTRL-C to quit
""".format(
    round(LINEAR_SPEED_STEP, 2), round(ANGULAR_SPPEED_STEP, 2)
)

error = """
Communications Failed
"""


def getKey():
    if os.name == "nt":
        if sys.version_info[0] >= 3:
            return msvcrt.getch().decode()
        else:
            return msvcrt.getch()

    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ""

    # termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def checkLinearLimitVelocity(input):

    if input <= LINEAR_SPEED_MIN:
        input = LINEAR_SPEED_MIN
    elif input >= LINEAR_SPEED_MAX:
        input = LINEAR_SPEED_MAX
    elif abs(input) < (LINEAR_SPEED_STEP / 20):
        input = 0
    else:
        input = input

    return input


def checkAngularLimitVelocity(input):

    if input <= ANGULAR_SPPEED_MIN:
        input = ANGULAR_SPPEED_MIN
    elif input >= ANGULAR_SPPEED_MAX:
        input = ANGULAR_SPPEED_MAX
    elif abs(input) < (ANGULAR_SPPEED_STEP / 20):
        input = 0
    else:
        input = input

    return input


def displayState(target_linear_vel, target_angular_vel):
    pass
    # return "Currently state: Linear vel= %s     Angular vel= %s " % (
    #     target_linear_vel,
    #     target_angular_vel,
    # )


def main():
    if os.name != "nt":
        settings = termios.tcgetattr(sys.stdin)

    rospy.init_node(NODE_NAME)

    pub = rospy.Publisher(TOPIC, Twist, queue_size=10)
    # rate = rospy.Rate(RATE) # unit: hz

    status = 0
    target_linear_PWM = 0.0
    target_angular_vel = 0.0
    control_linear_PWM = 0.0
    control_angular_PWM = 0.0

    try:
        print(display_instruction)

        while True:
            key = getKey()
            if key == "w":
                target_linear_PWM = checkLinearLimitVelocity(
                    target_linear_PWM + LINEAR_SPEED_STEP
                )
                status = status + 1
                # print(displayState(target_linear_PWM, target_angular_vel))
            elif key == "x":
                target_linear_PWM = checkLinearLimitVelocity(
                    target_linear_PWM - LINEAR_SPEED_STEP
                )
                status = status + 1
                # print(displayState(target_linear_PWM, target_angular_vel))
            elif key == "d":
                target_angular_vel = checkAngularLimitVelocity(
                    target_angular_vel - ANGULAR_SPPEED_STEP
                )
                status = status + 1
                # print(displayState(target_linear_PWM, target_angular_vel))
            elif key == "a":
                target_angular_vel = checkAngularLimitVelocity(
                    target_angular_vel + ANGULAR_SPPEED_STEP
                )
                status = status + 1
                # print(displayState(target_linear_PWM, target_angular_vel))
            elif key == "s":
                target_linear_PWM = 0.0
                target_angular_vel = 0.0
                control_linear_PWM = 0.0
                control_angular_PWM = 0.0
                # print(displayState(target_linear_PWM, target_angular_vel))
            else:
                if key == "\x03":
                    break

            twist = Twist()

            twist.linear.x = target_linear_PWM
            twist.linear.y = 0.0
            twist.linear.z = 0.0

            twist.angular.x = 0.0
            twist.angular.y = 0.0
            twist.angular.z = target_angular_vel

            pub.publish(twist)

    except:
        print(error)

    finally:
        twist = Twist()
        twist.linear.x = 0.0
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = 0.0
        pub.publish(twist)

    if os.name != "nt":
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

        # rospy.loginfo(dictionary_message)
        # pub.publish(dictionary_message)


if __name__ == "__main__":
    main()
    # except rospy.ROSInterruptException:
    #     pass
