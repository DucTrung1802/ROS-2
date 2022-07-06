from time import time

from lib import *
import timeit

sum = 0

for i in range(100):
    start = timeit.default_timer()
    result = calculateFuzzy(42, 0, 42, 0, b"output")
    end = timeit.default_timer()
    if i == 0:
        print(result[0])
        print(result[1])
    sum += end - start

sum /= 100

print(sum)
