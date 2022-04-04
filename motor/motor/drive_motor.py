#!/usr/bin/python3

# adapted from https://github.com/recantha/EduKit3-RC-Keyboard/blob/master/rc_keyboard.py

import sys, termios, tty, os, time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from pathlib import Path

BUTTON_DELAY = 0.001
NODE_NAME = Path(__file__).stem

STEP = 3

# Velocity: m/s
LINEAR_SPEED_MAX = 0.6  # m/s
LINEAR_SPEED_MIN = -LINEAR_SPEED_MAX
LINEAR_SPEED_STEP = LINEAR_SPEED_MAX / STEP

# Degree: positive-clockwise; negative-counterclockwise
ANGULAR_SPPEED_MAX = 1.0  # rad/s
ANGULAR_SPPEED_MIN = -LINEAR_SPEED_MAX  # rad/s
ANGULAR_SPPEED_STEP = LINEAR_SPEED_MAX / STEP  # rad/s

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


def getKey():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        ch = sys.stdin.read(1)

    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch


def keyDetecter():
    try:
        while True:
            key = getKey()

            if key == "w":
                print("Stop!")
                exit(0)

            elif key == "x":
                print("Left pressed")
                time.sleep(BUTTON_DELAY)

            elif key == "d":
                print("Right pressed")
                time.sleep(BUTTON_DELAY)

            elif key == "a":
                print("Up pressed")
                time.sleep(BUTTON_DELAY)

            elif key == "s":
                print("Down pressed")
                time.sleep(BUTTON_DELAY)

    except:
        pass


class Publisher(Node):
    def __init__(self):
        super().__init__("minimal_publisher")
        self.publisher_ = self.create_publisher(String, "topic", 10)
        timer_period = 0.01  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        key = getKey()
        if key == "p":
            exit(0)
        msg.data = "Key pressed: " + key
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = Publisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
