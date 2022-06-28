#include <iostream>
#include "fuzzy_logic_diy_lib/Term.h"
#include "fuzzy_logic_diy_lib/FuzzyVariable.h"
#include "fuzzy_logic_diy_lib/RuleHandler.h"

using namespace FLD;
using namespace std;

int main()
{
    Term term_1 = Term("mf1", new TriangularMF(0.0, 0.0, 0.5));
    Term term_2 = Term("mf2", new TriangularMF(0.0, 0.5, 1.0));
    Term term_3 = Term("mf3", new TriangularMF(0.5, 1.0, 1.0));
    Term term_4 = Term("mf4", new TriangularMF(0.75, 1.0, 1.0));

    FuzzyVariable input_1 = FuzzyVariable("input1", 0.0, 1.0);
    input_1.addTerm(term_1);
    input_1.addTerm(term_2);
    input_1.addTerm(term_3);
    input_1.addTerm(term_4);

    input_1.calculateFuzzificatedSet(0.75);
    // input_1.printFuzzificatedSet();

    RuleHandler a_new_rule_handler = RuleHandler();

    a_new_rule_handler.parseRule("if input1 is mf1 and input2 is mf2 then output1 is mf2");

    return 0;
}