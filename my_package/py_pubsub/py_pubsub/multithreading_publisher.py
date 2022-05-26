import rclpy
from rclpy.node import Node
import time
from std_msgs.msg import String

import threading

POS_1 = 0
POS_2 = 0


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
        # print("hello")
        self.publisher_1.publish(msg)
        # self.i += 1


def task_1():
    global POS_1
    while True:
        start = time.time()
        POS_1 += 1
        # print(POS)
        time.sleep(0.005)
        end = time.time()
        print(
            "External job 1 thread ID: "
            + str(threading.get_ident())
            + "; Time: "
            + str(end - start)
        )
        print()


def task_2():
    global POS_2
    while True:
        start = time.time()
        POS_2 += 1
        # print(POS)
        time.sleep(0.04)
        end = time.time()
        print(
            "External job 2 thread ID: "
            + str(threading.get_ident())
            + "; Time: "
            + str(end - start)
        )
        print()


def task_3():
    global POS
    rclpy.init()
    minimal_publisher_1 = MinimalPublisher()
    while True:
        start = time.time()
        rclpy.spin_once(minimal_publisher_1)
        end = time.time()
        print(
            "Publishing thread ID: "
            + str(threading.get_ident())
            + "; Time: "
            + str(end - start)
        )
        print()


def threadingHandler():
    t1 = threading.Thread(target=task_1)
    t2 = threading.Thread(target=task_2)
    t3 = threading.Thread(target=task_3)

    t1.start()
    t2.start()
    t3.start()

    t1.join()
    t2.join()
    t3.join()


def main(args=None):
    # minimal_publisher_2 = MinimalPublisher()

    # rclpy.spin_once(minimal_publisher_1)
    threadingHandler()

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    # minimal_publisher_1.destroy_node()
    # rclpy.shutdown()


if __name__ == "__main__":
    main()
