# Libraries
import rclpy
from rclpy.node import Node
import time
from sensor_msgs.msg import Range
import RPi.GPIO as GPIO
import time

timer1 = time.time()


class Sonar(object):
    """A class that handles a single ultrasonic sensor.

    Args:
    trigger_pin (int): GPIO.BCM
    echo_pin (int): GPIO.BCM
    min_range (float): unit: meter
    max_range (float): unit: meter
    field_of_view (float): unit: radian
    """

    def __init__(self, trigger_pin, echo_pin, min_range, max_range, field_of_view):
        if self.__checkCondition(
            trigger_pin, echo_pin, min_range, max_range, field_of_view
        ):
            self.__trigger_pin = trigger_pin
            self.__echo_pin = echo_pin
            self.__min_range = min_range
            self.__max_range = max_range
            self.__field_of_view = field_of_view
            self.__radiation_type = 0

    def __checkCondition(
        self, trigger_pin, echo_pin, min_range, max_range, field_of_view
    ):
        trigger_pin_condition = isinstance(trigger_pin, int) and 2 <= trigger_pin <= 27
        echo_pin_condition = (
            isinstance(trigger_pin, int)
            and 2 <= trigger_pin <= 27
            and trigger_pin != echo_pin
        )
        range_condition = 0 < min_range <= max_range
        field_of_view_condition = field_of_view > 0

        if trigger_pin_condition and echo_pin_condition and range_condition and field_of_view_condition:
            return True
        else:
            return False

    def __saturate(self, index, min_index, max_index):
        if index < min_index:
            return min_index
        elif index > max_index:
            return max_index
        else:
            return index

    def getMeasureDistance(self):
        # GPIO Mode (BOARD / BCM)
        GPIO.setmode(GPIO.BCM)

        # set GPIO direction (IN / OUT)
        GPIO.setup(self.__trigger_pin, GPIO.OUT)
        GPIO.setup(self.__echo_pin, GPIO.IN)

        # set Trigger to HIGH
        GPIO.output(self.__trigger_pin, True)

        # set Trigger after 0.01ms to LOW
        time.sleep(0.00001)
        GPIO.output(self.__trigger_pin, False)

        StartTime = time.time()
        StopTime = time.time()

        # save StartTime
        while GPIO.input(self.__echo_pin) == 0:
            StartTime = time.time()

        # save time of arrival
        while GPIO.input(self.__echo_pin) == 1:
            StopTime = time.time()

        # time difference between start and arrival
        TimeElapsed = StopTime - StartTime
        # multiply with the sonic speed (34300 cm/s)
        # and divide by 2, because there and back
        distance = (TimeElapsed * 34300) / 2
        distance = self.__saturate(distance, self.__min_range, self.__max_range)
        return distance

    def getMinRange(self):
        return self.__min_range

    def getMaxRange(self):
        return self.__max_range

    def getFieldOfView(self):
        return self.__field_of_view

    def getRadiationType(self):
        return self.__radiation_type
