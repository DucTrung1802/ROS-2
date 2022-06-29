#include <iostream>
#include "fuzzy_logic_diy_lib/Term.h"
#include "fuzzy_logic_diy_lib/FuzzyVariable.h"
#include "fuzzy_logic_diy_lib/RuleHandler.h"
#include "fuzzy_logic_diy_lib/MamdaniFuzzySystem.h"

using namespace FLD;
using namespace std;

int main()
{
    Term term_1 = Term("mf1", new TriangularMF(0.0, 0.0, 0.5));
    Term term_2 = Term("mf2", new TriangularMF(0.0, 0.5, 1.0));
    Term term_3 = Term("mf3", new TriangularMF(0.5, 1.0, 1.0));

    FuzzyVariable input_1 = FuzzyVariable("input1", 0.0, 1.0);
    input_1.addTerm(term_1);
    input_1.addTerm(term_2);
    input_1.addTerm(term_3);

    FuzzyVariable input_2 = FuzzyVariable("input2", 0.0, 1.0);
    input_2.addTerm(term_1);
    input_2.addTerm(term_2);
    input_2.addTerm(term_3);

    FuzzyVariable output_1 = FuzzyVariable("output1", 0.0, 1.0);
    output_1.addTerm(term_1);
    output_1.addTerm(term_2);
    output_1.addTerm(term_3);

    MamdaniFuzzySystem mamdani_fuzzy_system = MamdaniFuzzySystem({input_1, input_2}, {output_1});

    mamdani_fuzzy_system.addRule("if input1 is mf1 and input2 is mf2 then output1 is mf3");

    mamdani_fuzzy_system.addRule("if input1 is mf2 and input2 is mf2 then output2 is mf3"); // output2 does not exist in system, write antecedentCheck() and consequentCheck()

    std::cout
        << mamdani_fuzzy_system.getNumberOfFuzzyRule() << std::endl;

    return 0;
}