import math

import tf_transformations


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

        self.__position = [self.__postion_x, self.__postion_y, self.__postion_z]

        self.__orientation_x = 0.0
        self.__orientation_y = 0.0
        self.__orientation_z = 0.0
        self.__orientation_w = 1.0

        self.__orientation = [
            self.__orientation_x,
            self.__orientation_y,
            self.__orientation_z,
            self.__orientation_w,
        ]

        self.__x = 0.0
        self.__y = 0.0
        self.__last_x = 0.0
        self.__last_y = 0.0

        self.__theta = 0.0

        self.__current_left_angle = 0.0
        self.__current_right_angle = 0.0
        self.__last_left_angle = 0.0
        self.__last_right_angle = 0.0

    def __leftTickToRad(self, tick):
        return (tick / self.__left_wheel_tick_per_round) * 2 * math.pi

    def __rightTickToRad(self, tick):
        return (tick / self.__right_wheel_tick_per_round) * 2 * math.pi

    def calculatePose(self, left_tick, right_tick):

        self.__current_left_angle = self.__leftTickToRad(left_tick)
        self.__current_right_angle = self.__rightTickToRad(right_tick)

        # Calculate theta
        self.__theta = float(
            math.fmod(
                (
                    self.__radius
                    * (self.__current_right_angle - self.__current_left_angle)
                    / self.__wheel_base
                ),
                2 * math.pi,
            )
        )  # rad (0 <= self.__theta < 2 pi)

        orientation = tf_transformations.quaternion_from_euler(0.0, 0.0, self.__theta)

        # Calculate x
        self.__x = self.__last_x + self.__radius * (
            (self.__current_right_angle - self.__last_right_angle)
            + (self.__current_left_angle - self.__last_left_angle)
        ) / 2 * math.cos(self.__theta)

        # Calculate y
        self.__y = self.__last_y + self.__radius * (
            (self.__current_right_angle - self.__last_right_angle)
            + (self.__current_left_angle - self.__last_left_angle)
        ) / 2 * math.sin(self.__theta)

        # Update output values
        self.__position[0] = self.__x
        self.__position[1] = self.__y

        self.__orientation[0] = orientation[0]
        self.__orientation[1] = orientation[1]
        self.__orientation[2] = orientation[2]
        self.__orientation[3] = orientation[3]

        # Update past values
        self.__last_x = self.__x
        self.__last_y = self.__y

        self.__last_left_angle = self.__current_left_angle
        self.__last_right_angle = self.__current_right_angle

    def getOutputPose(self):
        return (self.__position, self.__orientation)
