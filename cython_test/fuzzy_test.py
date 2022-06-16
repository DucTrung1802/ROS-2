import cython
from typing import List, Dict

class MadamniFuzzySystem(object):
    def __init__(self, x: cython.double, time: cython.int):
        self.__x: cython.double = x
        self.__time: cython.int = int(time)

    def output(self):
        sum: cython.double = 0
        i: cython.int
        for i in range(self.__time):
            sum += i
        return sum

