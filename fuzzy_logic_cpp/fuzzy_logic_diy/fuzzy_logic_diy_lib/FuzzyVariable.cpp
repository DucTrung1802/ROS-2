#include "FuzzyVariable.h"

using namespace FLD;

FuzzyVariable::FuzzyVariable(std::string name, float min_value, float max_value)
{
    this->name = name;
    this->min_value = min_value;
    this->max_value = max_value;
    this->fuzzificated_set = {};
}

void FuzzyVariable::addTerm(Term new_term)
{
    this->list_of_term.push_back(new_term);
    this->list_name_of_term.push_back(new_term.getName());
    this->number_of_term = this->list_of_term.size();
}

void FuzzyVariable::calculateFuzzificatedSet()
{
    auto term = list_of_term.begin();
    // std::cout << this->list_of_term.begin() << std::endl;
    for (int i = 0; i < number_of_term; i++)
    {
        this->fuzzificated_set.insert(std::pair<std::string, float>(term->getName(), term->compute(this->input_value)));
        std::advance(term, 1);
    }
}

void FuzzyVariable::printFuzzificatedSet()
{
    for (auto output : this->fuzzificated_set)
    {
        std::cout << "key: " << output.first << ", "
                  << "value: " << output.second << std::endl;
    }
}

std::string FuzzyVariable::getName()
{
    return this->name;
}

std::list<std::string> FuzzyVariable::getListNameOfTerm()
{
    return this->list_name_of_term;
}

std::list<Term> FuzzyVariable::getListOfTerm()
{
    return this->list_of_term;
}

void FuzzyVariable::setInputValue(float input_value)
{
    this->input_value = float(input_value);
}

float FuzzyVariable::getInputValue()
{
    return this->input_value;
}