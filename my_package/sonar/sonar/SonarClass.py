# Libraries
import time
import RPi.GPIO as GPIO
import time

# limit measure time:
# maximum measure distane: 3m
# sonic speed: 343 m/s
# maximum measure time:3 / 343 * 2 = 0.017492s

MAXIMUM_MEASURE_TIME = 0.017492  # s
MEASURE_FREQUENCY = 200

if not (int(MEASURE_FREQUENCY) and MEASURE_FREQUENCY > 0):
    raise Exception("Invalid value of MEASURE_FREQUENCY!")

MEASURE_PERIOD = 1 / MEASURE_FREQUENCY


timer1 = time.time()

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

    def __init__(self, trigger_pin, echo_pin, min_range, max_range, field_of_view):
        if self.__checkCondition(
            trigger_pin, echo_pin, min_range, max_range, field_of_view
        ):
            self.__trigger_pin = trigger_pin
            self.__echo_pin = echo_pin
            self.__min_range = min_range
            self.__max_range = max_range
            self.__field_of_view = field_of_view
            self.__range = 0.0
            self.__radiation_type = 0

            self.__number_of_error = 0

            # set GPIO direction (IN / OUT)
            GPIO.setup(self.__trigger_pin, GPIO.OUT)
            GPIO.setup(self.__echo_pin, GPIO.IN)

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

        if (
            trigger_pin_condition
            and echo_pin_condition
            and range_condition
            and field_of_view_condition
        ):
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

    def __checkEchoPinStatus(self, status_to_check):
        pivot_time = time.time()
        current_time = time.time()

        target_time = time.time() + MAXIMUM_MEASURE_TIME

        while (
            GPIO.input(self.__echo_pin) == bool(status_to_check)
            and time.time() <= target_time
        ):
            current_time = time.time()

        if current_time - pivot_time > MAXIMUM_MEASURE_TIME:
            self.__number_of_error += 1
        else:
            self.__number_of_error = 0

        if self.__number_of_error >= 3:
            raise Exception(
                "An error has occurred with the sonar has TRIGGER PIN: "
                + str(self.__trigger_pin)
                + " and ECHO PIN: "
                + str(self.__echo_pin)
                + "!"
            )

        return current_time

    def measureRange(self):

        # set Trigger to HIGH
        GPIO.output(self.__trigger_pin, True)

        # set Trigger to LOW after 0.01ms
        time.sleep(0.00001)
        GPIO.output(self.__trigger_pin, False)

        start_time = self.__checkEchoPinStatus(False)
        stop_time = self.__checkEchoPinStatus(True)

        # start_time = time.time()
        # stop_time = time.time()

        # # save StartTime
        # while GPIO.input(self.__echo_pin) == 0:
        #     start_time = time.time()

        # # save time of arrival
        # while GPIO.input(self.__echo_pin) == 1:
        #     stop_time = time.time()

        # time difference between start and arrival
        time_elapsed = stop_time - start_time
        # multiply with the sonic speed (343 m/s)
        # and divide by 2, because there and back
        distance = (time_elapsed * 343) / 2
        distance = self.__saturate(distance, self.__min_range, self.__max_range)

        self.__range = distance

        time.sleep(MEASURE_PERIOD)

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
