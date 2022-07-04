from time import time

from pkg_resources import resource_listdir
from lib import *
import timeit

sum = 0

for i in range(100):
    start = timeit.default_timer()
    result = calculateFuzzy(86.97, 18090, 86.97, 18090, b"output")
    # print(result[0])
    # print(result[1])
    end = timeit.default_timer()
    sum += end - start

sum /= 100
print(sum)
