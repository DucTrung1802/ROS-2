from logging import exception
from math import floor


class MedianFilter(object):
    def __init__(self, number_of_value):
        if int(number_of_value) and number_of_value < 1:
            raise exception("Invalid number of values!")
        self.__number_of_value = int(number_of_value)
        self.__result = 0.0
        self.__list_of_value = []
        self.__sorted_list = []
        for i in range(self.__number_of_value - 1):
            self.__list_of_value.append(0.0)

    def addValue(self, value):
        self.__list_of_value.append(float(value))
        self.__sorted_list = self.__list_of_value.copy()
        self.__sorted_list.sort()
        self.__result = self.__sorted_list[floor(self.__number_of_value / 2)]
        del self.__list_of_value[0]

    def getResult(self):
        return self.__result
