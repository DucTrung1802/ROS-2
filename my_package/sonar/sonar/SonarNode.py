# Libraries
import rclpy
from rclpy.node import Node
import time
from sensor_msgs.msg import Range
import RPi.GPIO as GPIO
import time
from sonar.SonarClass import Sonar
import threading

# Node parameters
PUBLISH_FREQUENCY = 10
NODE_NAME = "sonar"


def checkConditions():
    global PUBLISH_PERIOD

    # GPIO Mode (BOARD / BCM)
    GPIO.setmode(GPIO.BCM)

    if PUBLISH_FREQUENCY <= 0:
        raise Exception("PUBLISH_FREQUENCY must be an positive integer!")
    PUBLISH_PERIOD = 1 / PUBLISH_FREQUENCY


class SonarNode(Node):
    def __init__(self, order, node_name, sonar_instance):
        if not isinstance(sonar_instance, Sonar):
            raise Exception(
                "Invalid type of variable sonar_instance, must be an instance of 'Sonar'!"
            )

        super().__init__(node_name + "_" + str(order))
        self._order = order
        self._node_name = node_name
        self._sonar_instance = sonar_instance

        self.sonar_pub = self.create_publisher(
            Range, "/ultrasonic_sensor_" + str(self._order), 1
        )

        self.timer = self.create_timer(0, self.timer_callback)

    def timer_callback(self):
        msg = Range()
        msg.header.frame_id = "ultrasonic_" + str(self._order) + "_link"
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.radiation_type = self._sonar_instance.getRadiationType()
        msg.field_of_view = (
            self._sonar_instance.getFieldOfView()
        )  # rad ~ 15 degree (according to feature of HC-SR 04)
        msg.min_range = self._sonar_instance.getMinRange()
        msg.max_range = self._sonar_instance.getMaxRange()
        msg.range = self._sonar_instance.getMeasureDistance()
        self.sonar_pub.publish(msg)


def task_1():
    global flag_1, sonar_node_1
    while True:

        if flag_1:
            break

        rclpy.spin_once(sonar_node_1)

        time.sleep(PUBLISH_PERIOD)


def task_2():
    global flag_2, sonar_node_2
    while True:

        if flag_2:
            break

        rclpy.spin_once(sonar_node_2)

        time.sleep(PUBLISH_PERIOD)


def task_3():
    global flag_3, sonar_node_3
    while True:

        if flag_3:
            break

        rclpy.spin_once(sonar_node_3)

        time.sleep(PUBLISH_PERIOD)


def task_4():
    global flag_4, sonar_node_4
    while True:

        if flag_4:
            break

        rclpy.spin_once(sonar_node_4)

        time.sleep(PUBLISH_PERIOD)


def task_5():
    global flag_5, sonar_node_5
    while True:

        if flag_5:
            break

        rclpy.spin_once(sonar_node_5)

        time.sleep(PUBLISH_PERIOD)


def threadingHandler():
    global flag_1, flag_2, flag_3, flag_4, flag_5

    flag_1 = False
    flag_2 = False
    flag_3 = False
    flag_4 = False
    flag_5 = False

    thread_1 = threading.Thread(target=task_1)
    thread_2 = threading.Thread(target=task_2)
    thread_3 = threading.Thread(target=task_3)
    thread_4 = threading.Thread(target=task_4)
    thread_5 = threading.Thread(target=task_5)

    thread_1.start()
    thread_2.start()
    thread_3.start()
    thread_4.start()
    thread_5.start()

    thread_1.join()
    thread_2.join()
    thread_3.join()
    thread_4.join()
    thread_5.join()


def setup():
    checkConditions()


def loop():
    global sonar_node_1, sonar_node_2, sonar_node_3, sonar_node_4, sonar_node_5

    rclpy.init()

    # Configure this when add more sonar sensors
    sonar_1 = Sonar(
        trigger_pin=17, echo_pin=27, min_range=0.04, max_range=1.5, field_of_view=0.78
    )
    sonar_2 = Sonar(
        trigger_pin=10, echo_pin=9, min_range=0.04, max_range=1.5, field_of_view=0.78
    )
    sonar_3 = Sonar(
        trigger_pin=5, echo_pin=6, min_range=0.04, max_range=1.5, field_of_view=0.78
    )
    sonar_4 = Sonar(
        trigger_pin=13, echo_pin=19, min_range=0.04, max_range=1.5, field_of_view=0.78
    )
    sonar_5 = Sonar(
        trigger_pin=23, echo_pin=24, min_range=0.04, max_range=1.5, field_of_view=0.78
    )

    sonar_node_1 = SonarNode(order=1, node_name=NODE_NAME, sonar_instance=sonar_1)
    sonar_node_2 = SonarNode(order=2, node_name=NODE_NAME, sonar_instance=sonar_2)
    sonar_node_3 = SonarNode(order=3, node_name=NODE_NAME, sonar_instance=sonar_3)
    sonar_node_4 = SonarNode(order=4, node_name=NODE_NAME, sonar_instance=sonar_4)
    sonar_node_5 = SonarNode(order=5, node_name=NODE_NAME, sonar_instance=sonar_5)

    try:
        threadingHandler()

        # Reset by pressing CTRL + C
    except KeyboardInterrupt:
        print("Measurement stopped by User")
        GPIO.cleanup()


def main():
    setup()
    loop()


if __name__ == "__main__":
    main()
