import ctypes
import sys
import os

dir_path = os.path.dirname(os.path.realpath(__file__))
handle = ctypes.CDLL(dir_path + "/libTest.so")

handle.My_Function.argtypes = [ctypes.c_float]


def My_Function(num):
    return handle.My_Function(num)

