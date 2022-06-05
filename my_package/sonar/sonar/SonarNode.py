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


def task_1(sonar):
    global flag_1
    if not isinstance(sonar, Sonar):
        raise Exception("Task 1: parameter sonar is not an instance of Sonar!")

    while True:

        if flag_1:
            break

        sonar.measureRange()

        time.sleep(PUBLISH_PERIOD)


def task_2():
    global flag_2
    while True:

        if flag_2:
            break

        rclpy.spin_once(sonar_node_2)

        time.sleep(PUBLISH_PERIOD)


def task_3():
    global flag_3
    while True:

        if flag_3:
            break

        rclpy.spin_once(sonar_node_3)

        time.sleep(PUBLISH_PERIOD)


def task_4():
    global flag_4
    while True:

        if flag_4:
            break

        rclpy.spin_once(sonar_node_4)

        time.sleep(PUBLISH_PERIOD)


def task_5():
    global flag_5
    while True:

        if flag_5:
            break

        rclpy.spin_once(sonar_node_5)

        time.sleep(PUBLISH_PERIOD)


def task_6():
    global flag_6, sonar_node_6
    while True:

        if flag_6:
            break

        rclpy.spin_once(sonar_node_6)

        time.sleep(PUBLISH_PERIOD)


def threadingHandler(sonar_array):
    global flag_1, flag_2, flag_3, flag_4, flag_5, flag_6

    flag_1 = False
    flag_2 = False
    flag_3 = False
    flag_4 = False
    flag_5 = False
    flag_6 = False

    thread_1 = threading.Thread(target=task_1, args=(sonar_array[0],))
    thread_2 = threading.Thread(target=task_2, args=(sonar_array[1],))
    thread_3 = threading.Thread(target=task_3, args=(sonar_array[2],))
    thread_4 = threading.Thread(target=task_4, args=(sonar_array[3],))
    thread_5 = threading.Thread(target=task_5, args=(sonar_array[4],))
    thread_6 = threading.Thread(target=task_6, args=(sonar_array,))

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

    sonar_array = []

    # Configure this when add more sonar sensors
    sonar_1 = Sonar(
        trigger_pin=17, echo_pin=27, min_range=0.04, max_range=1.5, field_of_view=0.244
    )
    sonar_2 = Sonar(
        trigger_pin=10, echo_pin=9, min_range=0.04, max_range=1.5, field_of_view=0.244
    )
    sonar_3 = Sonar(
        trigger_pin=5, echo_pin=6, min_range=0.04, max_range=1.5, field_of_view=0.244
    )
    sonar_4 = Sonar(
        trigger_pin=13, echo_pin=19, min_range=0.04, max_range=1.5, field_of_view=0.244
    )
    sonar_5 = Sonar(
        trigger_pin=23, echo_pin=24, min_range=0.04, max_range=1.5, field_of_view=0.244
    )

    sonar_array.append(sonar_1)
    sonar_array.append(sonar_2)
    sonar_array.append(sonar_3)
    sonar_array.append(sonar_4)
    sonar_array.append(sonar_5)

    try:
        threadingHandler(sonar_array)

        # Reset by pressing CTRL + C
    except KeyboardInterrupt:
        print("Measurement stopped by User")
        GPIO.cleanup()


def main():
    setup()
    loop()


if __name__ == "__main__":
    main()
