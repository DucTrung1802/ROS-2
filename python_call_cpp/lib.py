import ctypes
import sys
import os

name_of_cpp_shared_library = "/lib_cpp.so"

dir_path = os.path.dirname(os.path.realpath(__file__))
handle = ctypes.CDLL(dir_path + name_of_cpp_shared_library)

handle.My_Function.argtypes = [ctypes.c_float]


def My_Function(num):
    return handle.My_Function(num)
