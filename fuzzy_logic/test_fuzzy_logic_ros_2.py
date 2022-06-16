from pprint import pprint
from fuzzy_logic.terms import Term
from fuzzy_logic.variables import FuzzyVariable
from fuzzy_logic.mamdani_fs import MamdaniFuzzySystem
from fuzzy_logic.mf import TriangularMF
import timeit

# input1
e1 = Term('NB', TriangularMF(-200, -140, -70))
e2 = Term('NS', TriangularMF(-105, -40, 0))
e3 = Term('Z', TriangularMF(-35, 0, 35))
e4 = Term('PS', TriangularMF(0, 40, 105))
e5 = Term('PB', TriangularMF(70, 140, 200))

# input2
de1 = Term('DF', TriangularMF(-42000, -28000, -14000))
de2 = Term('DS', TriangularMF(-21000, -7000, 0))
de3 = Term('M', TriangularMF(-5000, 0, 5000))
de4 = Term('IS', TriangularMF(0, 7000, 21000))
de5 = Term('IF', TriangularMF(14000, 28000, 42000))

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


def calulate(mf, num1, num2):
    result = mf.calculate({input1: num1, input2: num2})
    return result

start = timeit.default_timer()

result = calulate(mf, -125, -1723)

end = timeit.default_timer()

print(end - start)
