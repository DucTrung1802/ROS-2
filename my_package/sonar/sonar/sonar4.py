# Libraries
from errno import ECHILD
from math import floor
import rclpy
from rclpy.node import Node
import time
from sensor_msgs.msg import Range
import RPi.GPIO as GPIO
import time
from sonar.Sonar import Sonar


# Node parameters
ORDER = 4
TRIGGER_PIN = 26
ECHO_PIN = 19

NUMBER_OF_MEDIAN_FILTER_ELEMENT = 9
PUBLISH_FREQUENCY = 10
NODE_NAME = "sonar" + str(ORDER)


def checkConditions():
    global PUBLISH_PERIOD

    # GPIO Mode (BOARD / BCM)
    GPIO.setmode(GPIO.BCM)

    if PUBLISH_FREQUENCY <= 0:
        raise Exception("PUBLISH_FREQUENCY must be an positive integer!")
    PUBLISH_PERIOD = 1 / PUBLISH_FREQUENCY

    if NUMBER_OF_MEDIAN_FILTER_ELEMENT <= 0:
        raise Exception("NUMBER_OF_MEDIAN_FILTER_ELEMENT must be an positive integer!")


class SonarNode(Node):
    def __init__(self, node_name, sonar_instance):

        self.__checkConditions(sonar_instance)

        super().__init__(node_name)
        self.__sonar = sonar_instance
        self.__value_list = []
        self.__sorted_list = []
        self.__old_value = 0.0
        self.__sonar_publisher = self.create_publisher(
            Range, "/ultrasonic_sensor_" + str(ORDER), 1
        )
        self.__error_time = 0

        self.__timer = self.create_timer(PUBLISH_PERIOD, self.timer_callback)

    def __checkConditions(self, sonar_instance):
        if not isinstance(sonar_instance, Sonar):
            raise Exception(
                "Invalid type of variable sonar_array_instance, must an array of 'Sonar' intance!"
            )

    def timer_callback(self):
        self.__sonar.measureRange()
        msg = Range()
        msg.header.frame_id = "ultrasonic_" + str(ORDER) + "_link"
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.radiation_type = self.__sonar.getRadiationType()
        msg.field_of_view = (
            self.__sonar.getFieldOfView()
        )  # rad ~ 15 degree (according to feature of HC-SR 04)
        msg.min_range = self.__sonar.getMinRange()
        msg.max_range = self.__sonar.getMaxRange()
        msg.range = self.__sonar.getRange()
        self.__sonar_publisher.publish(msg)


def setup():
    print("Start setup")
    checkConditions()


def loop():
    print("Start loop")

    rclpy.init()
    sonar = Sonar(
        trigger_pin=TRIGGER_PIN,
        echo_pin=ECHO_PIN,
        min_range=0.02,
        max_range=1.05,
        field_of_view=0.558,
        number_of_value_median_filter=NUMBER_OF_MEDIAN_FILTER_ELEMENT,
    )
    sonar_node = SonarNode(node_name=NODE_NAME, sonar_instance=sonar)

    try:
        while True:
            rclpy.spin(sonar_node)

    # Reset by pressing CTRL + C
    except KeyboardInterrupt:
        print("Measurement stopped by User")
        GPIO.cleanup()


def main():
    setup()
    loop()


if __name__ == "__main__":
    main()

