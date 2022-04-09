#!/usr/bin/python3

# adapted from https://github.com/recantha/EduKit3-RC-Keyboard/blob/master/rc_keyboard.py

import sys, termios, tty, os, time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


button_delay = 0.001


def getch():
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
            char = getch()

            if char == "p":
                print("Stop!")
                exit(0)

            if char == "a":
                print("Left pressed")
                time.sleep(button_delay)

            elif char == "d":
                print("Right pressed")
                time.sleep(button_delay)

            elif char == "w":
                print("Up pressed")
                time.sleep(button_delay)

            elif char == "s":
                print("Down pressed")
                time.sleep(button_delay)

            elif char == "1":
                print("Number 1 pressed")
                time.sleep(button_delay)
    except:
        pass


class MinimalPublisher(Node):
    def __init__(self):
        super().__init__("minimal_publisher")
        self.publisher_ = self.create_publisher(String, "topic", 10)
        timer_period = 0.01  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        key = getch()
        if key == "p":
            exit(0)
        msg.data = "Key pressed: " + key
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
