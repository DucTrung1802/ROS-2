import math


class PoseCalculator(object):
    def __init__(
        self,
        radius,
        left_wheel_tick_per_round,
        right_wheel_tick_per_round,
        wheel_base,
    ):
        """
        Args:
            radius (float): radius of both wheels (m)
            left_wheel_tick_per_round (int): tick per round of left wheel (tick)
            right_wheel_tick_per_round (int): tick per round of right wheel (tick)
            wheel_base (float): the distance between 2 centers of the wheels (m)
        """
        self.__checkConditions(
            radius,
            left_wheel_tick_per_round,
            right_wheel_tick_per_round,
            wheel_base,
        )
        self.__initializeParameters()

    def __checkRadius(self, radius):
        if float(radius) and radius > 0:
            return True
        else:
            return False

    def __checkTick(self, tick):
        if float(tick) and tick > 0:
            return True
        else:
            return False

    def __checkWheelBase(self, wheel_base):
        if float(wheel_base) and wheel_base > 0:
            return True
        else:
            return False

    def __checkConditions(
        self,
        radius,
        left_wheel_tick_per_round,
        right_wheel_tick_per_round,
        wheel_base,
    ):

        if self.__checkRadius(radius):
            self.__radius = radius
        elif not self.__checkRadius(self, radius):
            raise Exception("Invalid value of wheels radius!")

        if self.__checkTick(left_wheel_tick_per_round):
            self.__left_wheel_tick_per_round = left_wheel_tick_per_round
        elif not self.__checkTick(left_wheel_tick_per_round):
            raise Exception("Invalid value of left wheel tick per round!")

        if self.__checkTick(right_wheel_tick_per_round):
            self.__right_wheel_tick_per_round = right_wheel_tick_per_round
        elif not self.__checkTick(right_wheel_tick_per_round):
            raise Exception("Invalid value of right wheel tick per round!")

        if self.__checkWheelBase(wheel_base):
            self.__wheel_base = wheel_base
        elif not self.__checkTick(wheel_base):
            raise Exception("Invalid value of wheel base!")

    def __initializeParameters(self):
        """Initialize private parameters."""

        self.__postion_x = 0.0
        self.__postion_y = 0.0
        self.__postion_z = 0.0

        self.__position = (self.__postion_x, self.__postion_y, self.__postion_z)

        self.__orientation_x = 0.0
        self.__orientation_y = 0.0
        self.__orientation_z = 0.0
        self.__orientation_w = 1.0

        self.__orientation = (
            self.__orientation_x,
            self.__orientation_y,
            self.__orientation_z,
            self.__orientation_w,
        )

        self.__x = 0.0
        self.__y = 0.0
        self.__theta = 0.0

    def __tickToRad(self, tick):
        return (tick / 480) * 2 * math.pi

    def calculatePose(self, left_tick, right_tick):
        self.__theta = math.fmod(
            (
                self.__radius
                * (self.__tickToRad(right_tick) - self.__tickToRad(left_tick))
                / self.__wheel_base
            ),
            2 * math.pi,
        )  # rad (0 <= self.__theta < 2 pi)
