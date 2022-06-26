#include <iostream>
#include "fuzzy_logic_diy_lib/Term.h"
#include "fuzzy_logic_diy_lib/FuzzyVariable.h"

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

    cout << test.compute(6) << endl;

    return 0;
}