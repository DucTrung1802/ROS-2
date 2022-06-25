#include <iostream>
#include "fuzzy_logic_diy_lib/FuzzyInput.h"

using namespace FLD;
using namespace std;

int main()
{
    FuzzyInput test = FuzzyInput(4.0, 5.0);
    cout << test.compute() << endl;

    return 0;
}