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
    this->number_of_term = this->list_of_term.size();
}

void FuzzyVariable::calculateFuzzificatedSet(float value)
{
    auto output_value = list_of_term.begin();
    // std::cout << this->list_of_term.begin() << std::endl;
    for (int i = 0; i < number_of_term; i++)
    {
        this->fuzzificated_set.insert(std::pair<std::string, float>(output_value->getName(), output_value->compute(value)));
        std::advance(output_value, 1);
    }
}

void FuzzyVariable::printFuzzificatedSet()
{
    for (auto output = this->fuzzificated_set.begin(); output != this->fuzzificated_set.end(); ++output)
    {
        std::cout << "key: " << output->first << ", "
                  << "value: " << output->second << std::endl;
    }
}
