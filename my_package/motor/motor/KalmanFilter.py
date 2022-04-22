class Kalman_Filter(object):
    def __init__(self):
        self.__initializeValues()

    def __initializeValues(self):
        """Initialize parameters of Kalman_Filter.

        Note:
            (float) Measure system state: Z \n
            (float) Current state estimate: Xn \n
            (float) Previous state estimate: Xn_1 \n
            (float) Currnet estimate uncertainty: Pn \n
            (float) Previous estimate uncertainty: Pn_1 \n
            (float) Kalman Gain: K \n
            (float) Measurement uncertainty: R \n
            (float) Process uncertainty: Q \n
        """
        self.__Z = 0
        self.__Xn = 0
        self.__Xn_1 = 0
        self.__Pn = 0
        self.__Pn_1 = 0
        self.__K = 0
        self.__R = 0
        self.__Q = 0

    def __update(self):
        self.__K = self.__Pn_1 / (self.__Pn_1 + self.__R)
        self.__Xn = self.__Xn_1 + self.__K * (self.__Z - self.__Xn_1)
        self.__Pn = (1 - self.__K) * self.__Pn_1

    def __predict(self):
        self.__Pn_1 = self.__Pn + self.__Q
        self.__Xn_1 = self.__Xn

    def setupValues(self, X, P, R, Q):
        self.__Xn_1 = X
        self.__Pn_1 = P
        self.__R = R
        self.__Q = Q

    def filter(self, Z):
        self.__Z = Z
        self.__update()
        self.__predict()

    def getCurrentStateEstimate(self):
        return self.__Xn
