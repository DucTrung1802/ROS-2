#include "FuzzyVariable.h"

using namespace FLD;

FuzzyVariable::FuzzyVariable(std::string name, float min_value, float max_value)
{
    this->name = name;
    this->min_value = min_value;
    this->max_value = max_value;
}

void FuzzyVariable::addTerm(Term new_term)
{
    this->list_of_term.push_back(new_term);
}
