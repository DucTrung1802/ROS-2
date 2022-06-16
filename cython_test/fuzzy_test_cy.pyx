from cpython cimport array
import array
cimport numpy as np
import numpy as np

cdef class MadamniFuzzySystem(object):
    cdef double __x
    cdef int __time
    cdef np.ndarray __sum
    cdef double __result 

    def __cinit__(self, x, time):
        self.__x = x
        self.__time = int(time)
        self.__sum = np.zeros(1)

    def output(self):
        for i in range(self.__time):
            self.__sum = np.append(self.__sum, [i])
        self.__result = np.sum(self.__sum)
        return self.__result

