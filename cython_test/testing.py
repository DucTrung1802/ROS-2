import example_cython
import example_original
import timeit

start = timeit.default_timer()
print(example_cython.test(100000000))
end = timeit.default_timer()

print(end - start)

start = timeit.default_timer()
print(example_original.test(100000000))
end = timeit.default_timer()

print(end - start)