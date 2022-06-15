import timeit

cy = timeit.timeit('''''',setup='import test_fuzzy_logic_cython',number=100)
py = timeit.timeit('''''',setup='import test_fuzzy_logic_python', number=100)

print(cy, py)
print('Cython is {}x faster'.format(py/cy))