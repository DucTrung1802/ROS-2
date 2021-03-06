# Libraries
import time
import RPi.GPIO as GPIO
import time

from hcsr04sensor import sensor
from sonar.MedianFilter import MedianFilter

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)


class Sonar(object):
    """A class that handles a single ultrasonic sensor.

    Args:
    trigger_pin (int): GPIO.BCM
    echo_pin (int): GPIO.BCM
    min_range (float): unit: meter
    max_range (float): unit: meter
    field_of_view (float): unit: radian
    """

    def __init__(
        self,
        trigger_pin,
        echo_pin,
        min_range,
        max_range,
        field_of_view,
        number_of_value_median_filter,
    ):
        if self.__checkCondition(
            trigger_pin,
            echo_pin,
            min_range,
            max_range,
            field_of_view,
            number_of_value_median_filter,
        ):
            self.__trigger_pin = trigger_pin
            self.__echo_pin = echo_pin
            self.__min_range = min_range
            self.__max_range = max_range
            self.__field_of_view = field_of_view
            self.__range = 0.0
            self.__radiation_type = 0
            self.__sonar = sensor.Measurement
            self.__output_value = 0.0
            self.__number_of_value = number_of_value_median_filter
            self.__median_filter = MedianFilter(self.__number_of_value)
            self.__number_of_run = 0.0
            self.__number_of_error = 0.0

            # set GPIO direction (IN / OUT)
            GPIO.setup(self.__trigger_pin, GPIO.OUT)
            GPIO.setup(self.__echo_pin, GPIO.IN)

        else:
            raise Exception(
                "Invalid initial values of Sonar with Trigger Pin: "
                + str(trigger_pin)
                + " and Echo Pin: "
                + str(echo_pin)
                + "!"
            )

        print(
            "Complete initializing Sonar with TRIGGER PIN: "
            + str(self.__trigger_pin)
            + " and ECHO PIN: "
            + str(self.__echo_pin)
        )

    def __checkCondition(
        self,
        trigger_pin,
        echo_pin,
        min_range,
        max_range,
        field_of_view,
        number_of_value_median_filter,
    ):
        trigger_pin_condition = isinstance(trigger_pin, int) and 2 <= trigger_pin <= 27
        echo_pin_condition = (
            isinstance(trigger_pin, int)
            and 2 <= trigger_pin <= 27
            and trigger_pin != echo_pin
        )
        range_condition = 0 < min_range <= max_range
        field_of_view_condition = field_of_view > 0
        number_of_value_median_filter_condition = (
            int(number_of_value_median_filter) and number_of_value_median_filter >= 1
        )

        if (
            trigger_pin_condition
            and echo_pin_condition
            and range_condition
            and field_of_view_condition
            and number_of_value_median_filter_condition
        ):
            return True
        else:
            return False

    def __saturate(self, index, min, max):
        if index <= min:
            return min
        elif index >= max:
            return max
        else:
            return index

    def measureRange(self):
        try:
            self.__number_of_run += 1
            range = (
                self.__sonar.basic_distance(self.__trigger_pin, self.__echo_pin) / 100.0
            )  # m
        except:
            self.__number_of_run += 1
            self.__number_of_error += 1
            print(
                "Rate of error of Sonar with TRIGGER PIN "
                + str(self.__trigger_pin)
                + ": "
                + str((self.__number_of_error * 100) / self.__number_of_run)
                + " %"
            )
            return

        self.__median_filter.addValue(range)

        self.__range = self.__saturate(
            self.__median_filter.getResult(), self.__min_range, self.__max_range
        )

    def getMinRange(self):
        return self.__min_range

    def getMaxRange(self):
        return self.__max_range

    def getFieldOfView(self):
        return self.__field_of_view

    def getRadiationType(self):
        return self.__radiation_type

    def getRange(self):
        return self.__range
