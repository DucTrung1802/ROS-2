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

    def __applyCoefficients(self, Kp, Ki, Kd, T):
        self.__Kp = Kp
        self.__Ki = Ki
        self.__Ki_on = Ki
        self.__Kd = Kd
        self.__T = T

    def __computeCoefficients(self):
        self.__alpha = (
            2 * self.__T * self.__Kp + self.__Ki * self.__T * self.__T + 2 * self.__Kd
        )
        self.__beta = (
            self.__Ki * self.__T * self.__T - 4 * self.__Kd - 2 * self.__T * self.__Kp
        )
        self.__gamma = 2 * self.__Kd
        self.__delta = 2 * self.__T

    def __initializeCoefficient(self, Kp, Ki, Kd, T, min, max):

        self.__applyCoefficients(Kp, Ki, Kd, T)

        self.__computeCoefficients()

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

    def __getSign(self, value):
        if value > 0:
            return 1
        elif value < 0:
            return -1
        else:
            return 0

    def __antiWindup(self):
        self.__is_saturating = self.__uk != self.__saturate(self.__uk)
        self.__sign_of_error = self.__getSign(self.__ek)
        self.__sign_of_control_output = self.__getSign(self.__uk)
        self.__same_sign = self.__sign_of_error == self.__sign_of_control_output

        if self.__is_saturating and self.__same_sign:
            self.__Ki = 0
        else:
            self.__Ki = self.__Ki_on

        self.__computeCoefficients()

    def changeCoefficients(self, Kp, Ki, Kd, T):
        if Kp < 0:
            raise Exception("Kp must not be negative!")
        if Ki < 0:
            raise Exception("Ki must not be negative!")
        if Kd < 0:
            raise Exception("Kd must not be negative!")
        if T <= 0:  # T is sample time
            raise Exception("T must be positive!")

        self.__applyCoefficients(Kp, Ki, Kd, T)

        self.__computeCoefficients()

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

        self.__antiWindup()

    def getOutputValue(self):
        return self.__saturate(self.__uk)

    def getKi(self):
        return self.__Ki
