import cython

@cython.cclass
class MadamniFuzzySystem(object):
    __x: cython.double 
    __time: cython.int
    
    def __init__(self, x: cython.double, time: cython.int):
        self.__x = x
        self.__time = time

    @cython.ccall
    def output(self):
        sum: cython.double = 0
        i: cython.int
        for i in range(self.__time):
            sum += i
        return sum

