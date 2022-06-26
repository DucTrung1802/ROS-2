#include "FuzzyInput.h"

using namespace FLD;

FuzzyInput::FuzzyInput(std::string name, MF *mf)
{
    this->name = name;
    this->mf = mf;
}

float FuzzyInput::compute(float x)
{
    return this->mf->getValue(x);
}