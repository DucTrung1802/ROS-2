import fuzzy_test_cypy
import fuzzy_test_py
import timeit
import numpy as np

arr_py = []
arr_cypy = []

for i in range(1):
    start = timeit.default_timer()
    x = fuzzy_test_py.MadamniFuzzySystem(100000, 630000)
    y = x.output()
    end = timeit.default_timer()
    
    arr_py = np.append(arr_py, [end - start])

    start = timeit.default_timer()
    x = fuzzy_test_cypy.MadamniFuzzySystem(100000, 630000)
    y = x.output()
    end = timeit.default_timer()
    
    arr_cypy = np.append(arr_cypy, [end - start])

result_py = np.average(arr_py)
result_cypy = np.average(arr_cypy)
print("Pure python runtime: " + str(result_py))
print("Cythonized python runtime: " + str(result_cypy))
print("Faster: " + str(result_py / result_cypy))