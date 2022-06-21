from fuzzylogic.classes import Domain, Set, Rule
from fuzzylogic.hedges import very
from fuzzylogic.functions import R, S, triangular
import timeit

temp = Domain("input1", 0, 1)
hum = Domain("input2", 0, 1)
motor = Domain("output", 0, 1)

temp.t1 = triangular(0, 0.5, c=0.000001)
temp.t2 = triangular(0, 1, c=0.5)
temp.t3 = triangular(0.5, 1, c=0.999999)

hum.t1 = triangular(0, 0.5, c=0.000001)
hum.t2 = triangular(0, 1, c=0.5)
hum.t3 = triangular(0.5, 1, c=0.999999)

motor.t1 = triangular(0, 0.5, c=0.000001)
motor.t2 = triangular(0, 1, c=0.5)
motor.t3 = triangular(0.5, 1, c=0.999999)

R1 = Rule({(temp.t1, hum.t1): motor.t1})
R2 = Rule({(temp.t2, hum.t2): motor.t2})


# R3 = Rule({(temp.hot, hum.wet): very(motor.fast)})
# R4 = Rule({(temp.cold, hum.wet): motor.slow})

rules = Rule({(temp.t1, hum.t1): motor.t1, (temp.t2, hum.t2): motor.t2})

# rules == R1 | R2 == sum([R1, R2])

values = {hum: 0.45, temp: 0.45}
print(rules(values))

start = timeit.default_timer()

x = rules(values)

end = timeit.default_timer()

print(end - start)
