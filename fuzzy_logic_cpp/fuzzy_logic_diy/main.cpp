#include <iostream>
#include "fuzzy_logic_diy_lib/Term.h"

using namespace FLD;
using namespace std;

int main()
{
    Term test = Term("rancid", new TriangularMF(0.0, 5.0, 8.0));
    cout << test.compute(6) << endl;

    return 0;
}