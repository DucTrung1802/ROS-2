from time import time
from lib import *
import timeit

sum = 0

for i in range(1):
    start = timeit.default_timer()
    result = calculateFuzzy(86.97, 18090, b"output")
    end = timeit.default_timer()
    print(result)
    sum += end - start

sum /= 1
print(sum)
