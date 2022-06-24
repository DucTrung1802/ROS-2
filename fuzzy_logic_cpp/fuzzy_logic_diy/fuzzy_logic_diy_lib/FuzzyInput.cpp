#include "FuzzyInput.h"

using namespace FLD;
using namespace std;

FuzzyInput::FuzzyInput(float a, float b)
{
    this->a = a;
    this->b = b;
}

float FuzzyInput::compute()
{
    return this->a + this->b;
}