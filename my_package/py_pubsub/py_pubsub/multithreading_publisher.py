import rclpy
from rclpy.node import Node
import time
from std_msgs.msg import String

import threading

POS = 0
timer1 = time.time()
timer2 = time.time()


class MinimalPublisher(Node):
    def __init__(self):
        super().__init__("minimal_publisher")
        self.publisher_1 = self.create_publisher(String, "topic1", 10)

        timer_period1 = 1  # seconds
        # bind = lambda x: self.timer_callback(x)
        self.timer1 = self.create_timer(timer_period1, self.timer_callback)

        # self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = "hello"
        print("hello")
        self.publisher_1.publish(msg)
        # self.i += 1


def task_1():
    global POS
    while True:
        POS += 1
        print(POS)
        time.sleep(1)


def task_2():
    global POS
    rclpy.init()
    minimal_publisher_1 = MinimalPublisher()
    while True:
        rclpy.spin_once(minimal_publisher_1)


def threadingHandler():
    t1 = threading.Thread(target=task_1)
    t1.start()
    t1.join()

    # t2 = threading.Thread(target=task_2)
    # t2.start()
    # t2.join()


def main(args=None):
    global POS, timer1, timer2
    # minimal_publisher_2 = MinimalPublisher()

    while True:
        # rclpy.spin_once(minimal_publisher_1)
        threadingHandler()

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
