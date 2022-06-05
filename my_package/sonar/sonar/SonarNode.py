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
NODE_NAME = "sonars"


def checkConditions():
    global PUBLISH_PERIOD

    # GPIO Mode (BOARD / BCM)
    GPIO.setmode(GPIO.BCM)

    if PUBLISH_FREQUENCY <= 0:
        raise Exception("PUBLISH_FREQUENCY must be an positive integer!")
    PUBLISH_PERIOD = 1 / PUBLISH_FREQUENCY


class SonarNode(Node):
    def __init__(self, node_name, sonar_array_instance):

        self.__checkConditions(sonar_array_instance)

        super().__init__(node_name)
        self.__node_name = node_name
        self.__sonar_array = sonar_array_instance
        self.__sonar_publisher = []

        for i in range(len(self.__sonar_array)):
            self.__sonar_publisher.append(
                self.create_publisher(Range, "/ultrasonic_sensor_" + str(i + 1), 1)
            )

        self.__timer = self.create_timer(PUBLISH_PERIOD, self.timer_callback)

    def __checkConditions(self, sonar_array_instance):
        for i in range(len(sonar_array_instance)):
            if not isinstance(sonar_array_instance[i], Sonar):
                raise Exception(
                    "Invalid type of variable sonar_array_instance, must an array of 'Sonar' intance!"
                )

    def timer_callback(self):
        for i in range(len(self.__sonar_array)):
            msg = Range()
            msg.header.frame_id = "ultrasonic_" + str(i + 1) + "_link"
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.radiation_type = self.__sonar_array[i].getRadiationType()
            msg.field_of_view = self.__sonar_array[
                i
            ].getFieldOfView()  # rad ~ 15 degree (according to feature of HC-SR 04)
            msg.min_range = self.__sonar_array[i].getMinRange()
            msg.max_range = self.__sonar_array[i].getMaxRange()
            msg.range = self.__sonar_array[i].getRange()
            self.__sonar_publisher[i].publish(msg)


def task_1(sonar):
    global flag_1
    if not isinstance(sonar, Sonar):
        raise Exception("Task 1: parameter sonar is not an instance of Sonar!")

    while True:

        if flag_1:
            break

        sonar.measureRange()


def task_2(sonar):
    global flag_2
    if not isinstance(sonar, Sonar):
        raise Exception("Task 2: parameter sonar is not an instance of Sonar!")

    while True:

        if flag_2:
            break

        sonar.measureRange()


def task_3(sonar):
    global flag_3
    if not isinstance(sonar, Sonar):
        raise Exception("Task 3: parameter sonar is not an instance of Sonar!")

    while True:

        if flag_3:
            break

        sonar.measureRange()


def task_4(sonar):
    global flag_4
    if not isinstance(sonar, Sonar):
        raise Exception("Task 4: parameter sonar is not an instance of Sonar!")

    while True:

        if flag_4:
            break

        sonar.measureRange()


def task_5(sonar):
    global flag_5
    if not isinstance(sonar, Sonar):
        raise Exception("Task 5: parameter sonar is not an instance of Sonar!")

    while True:

        if flag_5:
            break

        sonar.measureRange()


def task_6(sonar_array):
    global flag_6
    rclpy.init()
    while True:

        if flag_6:
            break

        sonar_node = SonarNode(node_name=NODE_NAME, sonar_array_instance=sonar_array)

        rclpy.spin(sonar_node)


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
    thread_6.start()

    thread_1.join()
    thread_2.join()
    thread_3.join()
    thread_4.join()
    thread_5.join()
    thread_6.join()


def setup():
    checkConditions()


def loop():
    global sonar_node_1, sonar_node_2, sonar_node_3, sonar_node_4, sonar_node_5

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
