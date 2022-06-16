from typing import List, Dict

class MadamniFuzzySystem(object):
    def __init__(self, x, time):
        cdef double self.__x = x
        cdef int self.__time = int(time)

    def output(self):
        sum = 0
        for i in range(self.__time):
            sum += self.__x
        return sum

