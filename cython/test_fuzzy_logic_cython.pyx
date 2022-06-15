from pprint import pprint
from fuzzy_logic.terms import Term
from fuzzy_logic.variables import FuzzyVariable
from fuzzy_logic.mamdani_fs import MamdaniFuzzySystem
from fuzzy_logic.mf import TriangularMF
import time

cdef int e1_min = -200
cdef int e1_med = -140
cdef int e1_max = -70

cdef int e2_min = -105
cdef int e2_med = -40
cdef int e2_max = 0

cdef int e3_min = -35
cdef int e3_med = 0
cdef int e3_max = 35

cdef int e4_min = 0
cdef int e4_med = 40
cdef int e4_max = 105

cdef int e5_min = 70
cdef int e5_med = 140
cdef int e5_max = 200


cdef int de1_min = -42000
cdef int de1_med = -28000
cdef int de1_max = -14000

cdef int de2_min = -21000
cdef int de2_med = -7000
cdef int de2_max = 0

cdef int de3_min = -5000
cdef int de3_med = 0
cdef int de3_max = 5000

cdef int de4_min = 0
cdef int de4_med = 7000
cdef int de4_max = 21000

cdef int de5_min = 14000
cdef int de5_med = 28000
cdef int de5_max = 42000


# input1
e1 = Term('NB', TriangularMF(e1_min, e1_med, e1_max))
e2 = Term('NS', TriangularMF(e2_min, e2_med, e2_max))
e3 = Term('Z', TriangularMF(e3_min, e3_med, e3_max))
e4 = Term('PS', TriangularMF(e4_min, e4_med, e4_max))
e5 = Term('PB', TriangularMF(e5_min, e5_med, e5_max))

# input2
de1 = Term('DF', TriangularMF(de1_min, de1_med, de1_max))
de2 = Term('DS', TriangularMF(de2_min, de2_med, de2_max))
de3 = Term('M', TriangularMF(de3_min, de3_med, de3_max))
de4 = Term('IS', TriangularMF(de4_min, de4_med, de4_max))
de5 = Term('IF', TriangularMF(de5_min, de5_med, de5_max))

# output
k1 = Term('S', TriangularMF(-0.25, 0, 0.25))
k2 = Term('MS', TriangularMF(0, 0.25, 0.5))
k3 = Term('Me', TriangularMF(0.25, 0.5, 0.75))
k4 = Term('MB', TriangularMF(0.5, 0.75, 1))
k5 = Term('B', TriangularMF(0.75, 1, 1.25))
input1: FuzzyVariable = FuzzyVariable('input1', -140, 140, e1, e2, e3, e4, e5)
input2: FuzzyVariable = FuzzyVariable(
    'input2', -28000, 28000, de1, de2, de3, de4, de5)
output = FuzzyVariable('output', 0, 1, k1, k2, k3, k4, k5)

mf: MamdaniFuzzySystem = MamdaniFuzzySystem([input1, input2], [output])
mf.rules.append(mf.parse_rule(
    'if (input1 is NB) and (input2 is DF) then (output is S)'))
mf.rules.append(mf.parse_rule(
    'if (input1 is NB) and (input2 is DS) then (output is S)'))
mf.rules.append(mf.parse_rule(
    'if (input1 is NB) and (input2 is M) then (output is MS)'))
mf.rules.append(mf.parse_rule(
    'if (input1 is NB) and (input2 is IS) then (output is MS)'))
mf.rules.append(mf.parse_rule(
    'if (input1 is NB) and (input2 is IF) then (output is Me)'))
mf.rules.append(mf.parse_rule(
    'if (input1 is NS) and (input2 is DF) then (output is S)'))
mf.rules.append(mf.parse_rule(
    'if (input1 is NS) and (input2 is DS) then (output is MS)'))
mf.rules.append(mf.parse_rule(
    'if (input1 is NS) and (input2 is M) then (output is MS)'))
mf.rules.append(mf.parse_rule(
    'if (input1 is NS) and (input2 is IS) then (output is Me)'))
mf.rules.append(mf.parse_rule(
    'if (input1 is NS) and (input2 is IF) then (output is MB)'))
mf.rules.append(mf.parse_rule(
    'if (input1 is Z) and (input2 is DF) then (output is MS)'))
mf.rules.append(mf.parse_rule(
    'if (input1 is Z) and (input2 is DS) then (output is MS)'))
mf.rules.append(mf.parse_rule(
    'if (input1 is Z) and (input2 is M) then (output is Me)'))
mf.rules.append(mf.parse_rule(
    'if (input1 is Z) and (input2 is IS) then (output is MB)'))
mf.rules.append(mf.parse_rule(
    'if (input1 is Z) and (input2 is IF) then (output is MB)'))
mf.rules.append(mf.parse_rule(
    'if (input1 is PS) and (input2 is DF) then (output is MS)'))
mf.rules.append(mf.parse_rule(
    'if (input1 is PS) and (input2 is DS) then (output is Me)'))
mf.rules.append(mf.parse_rule(
    'if (input1 is PS) and (input2 is M) then (output is MB)'))
mf.rules.append(mf.parse_rule(
    'if (input1 is PS) and (input2 is IS) then (output is MB)'))
mf.rules.append(mf.parse_rule(
    'if (input1 is PS) and (input2 is IF) then (output is B)'))
mf.rules.append(mf.parse_rule(
    'if (input1 is PB) and (input2 is DF) then (output is Me)'))
mf.rules.append(mf.parse_rule(
    'if (input1 is PB) and (input2 is DS) then (output is MB)'))
mf.rules.append(mf.parse_rule(
    'if (input1 is PB) and (input2 is M) then (output is MB)'))
mf.rules.append(mf.parse_rule(
    'if (input1 is PB) and (input2 is IS) then (output is B)'))
mf.rules.append(mf.parse_rule(
    'if (input1 is PB) and (input2 is IF) then (output is B)'))

start = time.time()

result = mf.calculate({input1: -125, input2: -1723})

end = time.time()

print(end - start)
