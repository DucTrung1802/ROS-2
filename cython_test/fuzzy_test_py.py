class MadamniFuzzySystem(object):
    def __init__(self, x, time):
        self.__x = x
        self.__time = int(time)

    def output(self):
        sum = 0
        for i in range(self.__time):
            sum += i
        return sum

