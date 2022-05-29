import time


class PIDController:
    def __init__(self, Kp, Ki, Kd, T, min, max):
        self.__checkConditions(Kp, Ki, Kd, T, min, max)
        self.__initializeCoefficient(Kp, Ki, Kd, T, min, max)

    def __checkConditions(self, Kp, Ki, Kd, T, min, max):
        if Kp < 0:
            raise Exception("Kp must not be negative!")
        if Ki < 0:
            raise Exception("Ki must not be negative!")
        if Kd < 0:
            raise Exception("Kd must not be negative!")
        if T <= 0:  # T is sample time
            raise Exception("T must be positive!")
        if min > max:
            raise Exception(
                "Minimum value must smaller or equal to maximum value!")

    def __applyCoefficients(self, Kp, Ki, Kd):
        self.__Kp = Kp
        self.__Ki = Ki
        self.__Kd = Kd

    def __initializeCoefficient(self, Kp, Ki, Kd, T, min, max):

        self.__applyCoefficients(Kp, Ki, Kd)

        self.__alpha = 2 * T * self.__Kp + self.__Ki * T * T + 2 * self.__Kd
        self.__beta = self.__Ki * T * T - 4 * self.__Kd - 2 * T * self.__Kp
        self.__gamma = 2 * self.__Kd
        self.__delta = 2 * T

        self.__uk = 0
        self.__uk_1 = 0
        self.__ek = 0
        self.__ek_1 = 0
        self.__ek_2 = 0

        self.__state = 0

        self.__min = min
        self.__max = max

    def __saturate(self, value):
        if value > self.__max:
            return self.__max
        elif value < self.__min:
            return self.__min
        else:
            return value

    def changeCoefficients(self, Kp, Ki, Kd):
        if Kp < 0:
            raise Exception("Kp must not be negative!")
        if Ki < 0:
            raise Exception("Ki must not be negative!")
        if Kd < 0:
            raise Exception("Kd must not be negative!")

        self.__applyCoefficients(Kp, Ki, Kd)

    def evaluate(self, setpoint, current_measure_value):
        self.__ek = setpoint - current_measure_value

        if self.__state == 0:
            self.__uk = self.__alpha * self.__ek / self.__delta

        elif self.__state == 1:
            self.__uk = (
                self.__alpha * self.__ek
                + self.__beta * self.__ek_1
                + self.__delta * self.__uk_1
            ) / self.__delta

        elif self.__state >= 2:
            self.__uk = (
                self.__alpha * self.__ek
                + self.__beta * self.__ek_1
                + self.__gamma * self.__ek_2
                + self.__delta * self.__uk_1
            ) / self.__delta

        self.__uk_1 = self.__uk
        self.__ek_2 = self.__ek_1
        self.__ek_1 = self.__ek
        self.__state += 1

    def getOutputValue(self):
        return self.__saturate(self.__uk)
