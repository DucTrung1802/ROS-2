import ctypes
import sys
import os

name_of_cpp_shared_library = "/lib_cpp.so"

dir_path = os.path.dirname(os.path.realpath(__file__))
handle = ctypes.CDLL(dir_path + name_of_cpp_shared_library)

# For input
handle.calculateFuzzy.argtypes = [
    ctypes.c_float,
    ctypes.c_float,
    ctypes.c_float,
    ctypes.c_float,
    ctypes.c_char_p,
]
# For output
handle.calculateFuzzy.restype = ctypes.POINTER(ctypes.c_float)


def calculateFuzzy(argu_1, argu_2, argu_3, argu_4, output_name):
    return handle.calculateFuzzy(argu_1, argu_2, argu_3, argu_4, output_name)
