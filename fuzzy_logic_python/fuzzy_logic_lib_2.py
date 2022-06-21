from simpful import *
import timeit
import time

# A simple fuzzy inference system for the tipping problem
# Create a fuzzy system object
FS = FuzzySystem()

# Input
E_1 = FuzzySet(function=Triangular_MF(a=-200, b=-140, c=-70), term="NB")
E_2 = FuzzySet(function=Triangular_MF(a=-105, b=-40, c=0), term="NS")
E_3 = FuzzySet(function=Triangular_MF(a=-35, b=0, c=35), term="Z")
E_4 = FuzzySet(function=Triangular_MF(a=0, b=40, c=105), term="PS")
E_5 = FuzzySet(function=Triangular_MF(a=70, b=140, c=200), term="PB")
FS.add_linguistic_variable(
    "input1",
    LinguisticVariable(
        [E_1, E_2, E_3, E_4, E_5], concept="input1", universe_of_discourse=[-140, 140]
    ),
)

DE_1 = FuzzySet(function=Triangular_MF(
    a=-42000, b=-28000, c=-14000), term="DeF")
DE_2 = FuzzySet(function=Triangular_MF(a=-21000, b=-7000, c=0), term="DeS")
DE_3 = FuzzySet(function=Triangular_MF(a=-5000, b=0, c=5000), term="M")
DE_4 = FuzzySet(function=Triangular_MF(a=0, b=7000, c=21000), term="InS")
DE_5 = FuzzySet(function=Triangular_MF(a=14000, b=28000, c=42000), term="InF")
FS.add_linguistic_variable(
    "input2",
    LinguisticVariable(
        [DE_1, DE_2, DE_3, DE_4, DE_5], concept="input2", universe_of_discourse=[-28000, 28000]
    ),
)

# Define output fuzzy sets and linguistic variable
K_1 = FuzzySet(function=Triangular_MF(a=-0.25, b=0, c=0.25), term="S")
K_2 = FuzzySet(function=Triangular_MF(a=0, b=0.25, c=0.5), term="MS")
K_3 = FuzzySet(function=Trapezoidal_MF(a=0.25, b=0.5, c=0.75), term="Me")
K_4 = FuzzySet(function=Triangular_MF(a=0.5, b=0.75, c=1), term="MB")
K_5 = FuzzySet(function=Trapezoidal_MF(a=0.75, b=1, c=1.25), term="B")
FS.add_linguistic_variable(
    "output", LinguisticVariable(
        [K_1, K_2, K_3, K_4, K_5], universe_of_discourse=[0, 1])
)

# Define fuzzy rules
R1 = "IF (input1 IS NB) AND (input2 IS DeF) THEN (output IS S)"
R2 = "IF (input1 IS NB) AND (input2 IS DeS) THEN (output IS S)"
R3 = "IF (input1 IS NB) AND (input2 IS M) THEN (output IS MS)"
R4 = "IF (input1 IS NB) AND (input2 IS InS) THEN (output IS MS)"
R5 = "IF (input1 IS NB) AND (input2 IS InF) THEN (output IS Me)"

R6 = "IF (input1 IS NS) AND (input2 IS DeF) THEN (output IS S)"
R7 = "IF (input1 IS NS) AND (input2 IS DeS) THEN (output IS MS)"
R8 = "IF (input1 IS NS) AND (input2 IS M) THEN (output IS MS)"
R9 = "IF (input1 IS NS) AND (input2 IS InS) THEN (output IS Me)"
R10 = "IF (input1 IS NS) AND (input2 IS InF) THEN (output IS MB)"

R11 = "IF (input1 IS Z) AND (input2 IS DeF) THEN (output IS MS)"
R12 = "IF (input1 IS Z) AND (input2 IS DeS) THEN (output IS MS)"
R13 = "IF (input1 IS Z) AND (input2 IS M) THEN (output IS Me)"
R14 = "IF (input1 IS Z) AND (input2 IS InS) THEN (output IS MB)"
R15 = "IF (input1 IS Z) AND (input2 IS InF) THEN (output IS MB)"

R16 = "IF (input1 IS PS) AND (input2 IS DeF) THEN (output IS MS)"
R17 = "IF (input1 IS PS) AND (input2 IS DeS) THEN (output IS Me)"
R18 = "IF (input1 IS PS) AND (input2 IS M) THEN (output IS MB)"
R19 = "IF (input1 IS PS) AND (input2 IS InS) THEN (output IS MB)"
R20 = "IF (input1 IS PS) AND (input2 IS InF) THEN (output IS B)"

R21 = "IF (input1 IS PB) AND (input2 IS DeF) THEN (output IS Me)"
R22 = "IF (input1 IS PB) AND (input2 IS DeS) THEN (output IS MB)"
R23 = "IF (input1 IS PB) AND (input2 IS M) THEN (output IS MB)"
R24 = "IF (input1 IS PB) AND (input2 IS InS) THEN (output IS B)"
R25 = "IF (input1 IS PB) AND (input2 IS InF) THEN (output IS B)"

FS.add_rules([R1, R2, R3, R4, R5, R6, R7, R8, R9, R10, R11, R12,
             R13, R14, R15, R16, R17, R18, R19, R20, R21, R22, R23, R24, R25])

# FS.add_rules([R1])

# Set antecedents values
FS.set_variable("input1", 3)
FS.set_variable("input2", 3)


def calculate():
    return FS.Mamdani_inference(["output"])

# Perform Mamdani inference and print output


sum = 0

for i in range(1):
    start = timeit.default_timer()

    x = FS.Mamdani_inference(["output"], 100)

    end = timeit.default_timer()

    sum += end - start

    print(end - start)
    print(x)
    time.sleep(0.4)


print(sum / 1000)
