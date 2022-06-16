import fuzzy_test
import timeit

start = timeit.default_timer()
x = fuzzy_test.MadamniFuzzySystem(100000, 200000)
print(x.output())
end = timeit.default_timer()

print(end - start)