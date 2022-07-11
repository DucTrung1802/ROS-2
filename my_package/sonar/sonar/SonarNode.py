# Libraries
from distutils.log import error
import rclpy
from rclpy.node import Node
import time
from sensor_msgs.msg import Range
import RPi.GPIO as GPIO
import time
from sonar.Sonar import Sonar
import threading

# Node parameters
PUBLISH_FREQUENCY = 20
NODE_NAME = "sonars"
NUMBER_OF_MEDIAN_FILTER_ELEMENT = 10
SAMPLE_RATE = 60


def checkConditions():
    global PUBLISH_PERIOD, SAMPLE_PERIOD

    # GPIO Mode (BOARD / BCM)
    GPIO.setmode(GPIO.BCM)

    if PUBLISH_FREQUENCY <= 0:
        raise Exception("PUBLISH_FREQUENCY must be an positive integer!")
    PUBLISH_PERIOD = 1 / PUBLISH_FREQUENCY

    if SAMPLE_RATE <= 0:
        raise Exception("SAMPLE_RATE must be an positive integer!")
    SAMPLE_PERIOD = 1 / SAMPLE_RATE


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

        self.__timer_1 = self.create_timer(PUBLISH_PERIOD, self.timer_callback_1)

        self.__timer_2 = self.create_timer(PUBLISH_PERIOD / 2, self.timer_callback_2)

    def __checkConditions(self, sonar_array_instance):
        for i in range(len(sonar_array_instance)):
            if not isinstance(sonar_array_instance[i], Sonar):
                raise Exception(
                    "Invalid type of variable sonar_array_instance, must an array of 'Sonar' intance!"
                )

    def timer_callback_1(self):
        for i in range(len(self.__sonar_array)):

            if i == 2:
                continue

            msg = Range()
            msg.header.frame_id = "ultrasonic_" + str(i + 1) + "_link"
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.radiation_type = self.__sonar_array[i].getRadiationType()
            msg.field_of_view = self.__sonar_array[
                i
            ].getFieldOfView()  # rad ~ 30 degree (according to feature of HC-SR 04)
            msg.min_range = self.__sonar_array[i].getMinRange()
            msg.max_range = self.__sonar_array[i].getMaxRange()
            msg.range = self.__sonar_array[i].getRange()
            self.__sonar_publisher[i].publish(msg)

    def timer_callback_2(self):
        msg = Range()
        msg.header.frame_id = "ultrasonic_" + str(3) + "_link"
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.radiation_type = self.__sonar_array[2].getRadiationType()
        msg.field_of_view = self.__sonar_array[
            2
        ].getFieldOfView()  # rad ~ 30 degree (according to feature of HC-SR 04)
        msg.min_range = self.__sonar_array[2].getMinRange()
        msg.max_range = self.__sonar_array[2].getMaxRange()
        msg.range = self.__sonar_array[2].getRange()
        self.__sonar_publisher[2].publish(msg)


def task_1(sonar):
    global flag_1
    print("Start thread 1")
    if not isinstance(sonar, Sonar):
        raise Exception("Task 1: parameter sonar is not an instance of Sonar!")
    error_number = 0

    while True:
        time.sleep(SAMPLE_PERIOD)

        if flag_1:
            break

        try:
            sonar.measureRange()
            # print("Task 1 is running...")
        except:
            print("Sonar 1 occured error " + str(error_number) + " times")


def task_2(sonar):
    global flag_2
    print("Start thread 2")
    if not isinstance(sonar, Sonar):
        raise Exception("Task 2: parameter sonar is not an instance of Sonar!")
    error_number = 0

    while True:
        time.sleep(SAMPLE_PERIOD)

        if flag_2:
            break

        try:
            sonar.measureRange()
            # print("Task 2 is running...")
        except:
            print("Sonar 2 occured error " + str(error_number) + " times")


def task_3(sonar):
    global flag_3
    print("Start thread 3")
    if not isinstance(sonar, Sonar):
        raise Exception("Task 3: parameter sonar is not an instance of Sonar!")
    error_number = 0

    while True:
        time.sleep(SAMPLE_PERIOD)

        if flag_3:
            break

        try:
            sonar.measureRange()
            # print("Task 3 is running...")
        except:
            error_number += 1
            print("Sonar 3 occured error " + str(error_number) + " times")


def task_4(sonar):
    global flag_4
    print("Start thread 4")
    if not isinstance(sonar, Sonar):
        raise Exception("Task 4: parameter sonar is not an instance of Sonar!")
    error_number = 0

    while True:
        time.sleep(SAMPLE_PERIOD)

        if flag_4:
            break

        try:
            sonar.measureRange()
            # print("Task 4 is running...")
        except:
            print("Sonar 4 occured error " + str(error_number) + " times")


def task_5(sonar):
    global flag_5
    print("Start thread 5")
    if not isinstance(sonar, Sonar):
        raise Exception("Task 5: parameter sonar is not an instance of Sonar!")
    error_number = 0

    while True:
        time.sleep(SAMPLE_PERIOD)

        if flag_5:
            break

        try:
            sonar.measureRange()
            # print("Task 5 is running...")
        except:
            print("Sonar 5 occured error " + str(error_number) + " times")


def task_6(sonar_node):
    global flag_6
    print("Start thread 6")
    while True:

        if flag_6:
            break

        rclpy.spin(sonar_node)
        # print("Task 6 is running...")


def threadingHandler(sonar_array):
    global flag_1, flag_2, flag_3, flag_4, flag_5, flag_6

    rclpy.init()
    print("Creating instance...")
    sonar_node = SonarNode(node_name=NODE_NAME, sonar_array_instance=sonar_array)
    print("Created instance!")

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
    thread_6 = threading.Thread(target=task_6, args=(sonar_node,))

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
    print("Start setup")
    checkConditions()


def loop():
    print("Start loop")
    global sonar_node_1, sonar_node_2, sonar_node_3, sonar_node_4, sonar_node_5

    sonar_array = []

    # Configure this when add more sonar sensors
    sonar_1 = Sonar(
        trigger_pin=27,
        echo_pin=17,
        min_range=0.02,
        max_range=1.05,
        field_of_view=0.558,
        number_of_value_median_filter=NUMBER_OF_MEDIAN_FILTER_ELEMENT,
    )
    sonar_2 = Sonar(
        trigger_pin=9,
        echo_pin=10,
        min_range=0.02,
        max_range=1.05,
        field_of_view=0.558,
        number_of_value_median_filter=NUMBER_OF_MEDIAN_FILTER_ELEMENT,
    )
    sonar_3 = Sonar(
        trigger_pin=6,
        echo_pin=5,
        min_range=0.02,
        max_range=1.05,
        field_of_view=0.558,
        number_of_value_median_filter=NUMBER_OF_MEDIAN_FILTER_ELEMENT,
    )
    sonar_4 = Sonar(
        trigger_pin=26,
        echo_pin=19,
        min_range=0.02,
        max_range=1.05,
        field_of_view=0.558,
        number_of_value_median_filter=NUMBER_OF_MEDIAN_FILTER_ELEMENT,
    )
    sonar_5 = Sonar(
        trigger_pin=24,
        echo_pin=23,
        min_range=0.02,
        max_range=1.05,
        field_of_view=0.558,
        number_of_value_median_filter=NUMBER_OF_MEDIAN_FILTER_ELEMENT,
    )

    sonar_array.append(sonar_1)
    sonar_array.append(sonar_2)
    sonar_array.append(sonar_3)
    sonar_array.append(sonar_4)
    sonar_array.append(sonar_5)

    print(sonar_array)

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
