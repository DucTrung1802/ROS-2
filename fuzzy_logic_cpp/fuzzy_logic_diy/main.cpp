#include <iostream>
#include "fuzzy_logic_diy_lib/FuzzyInput.h"

using namespace FLD;
using namespace std;

int main()
{
    FuzzyInput test = FuzzyInput("rancid", new TriangularMF(0.0, 5.0, 8.0));
    cout << test.compute(6) << endl;

    return 0;
}