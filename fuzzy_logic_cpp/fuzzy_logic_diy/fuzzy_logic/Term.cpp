#include "Term.h"

using namespace FLD;

Term::Term(std::string name, MF *mf)
{
    this->name = name;
    this->mf = mf;
}

float Term::compute(float x)
{
    return this->mf->getValue(x);
}

std::string Term::getName()
{
    return this->name;
}
