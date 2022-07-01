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

std::pair<std::string, float> FuzzyVariable::getMaxFuzzificatedSet(float input_value)
{
    std::pair<std::string, float> temp_max_fuzzificated_set;
    std::string first_name = this->list_of_term.front().getName();
    float first_value = this->list_of_term.front().compute(input_value);
    for (auto term : this->list_of_term)
    {
        std::cout << "hello" << std::endl;
        float value = term.compute(input_value);
        if (value > first_value)
        {
            temp_max_fuzzificated_set.first = term.getName();
            temp_max_fuzzificated_set.second = value;
        }

        else
        {
            temp_max_fuzzificated_set.first = first_name;
            temp_max_fuzzificated_set.second = first_value;
        }
    }
    // std::cout << temp_max_fuzzificated_set.second << std::endl;
    return temp_max_fuzzificated_set;
}

void FuzzyVariable::printFuzzificatedSet()
{
    std::cout << "Variable: " << this->name << std::endl;
    for (auto output : this->fuzzificated_set)
    {
        std::cout << "key: " << output.first << ", "
                  << "value: " << output.second << std::endl;
    }
    std::cout << std::endl;
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

std::map<std::string, float> FuzzyVariable::getFuzzificatedSet()
{
    return this->fuzzificated_set;
}

float FuzzyVariable::getMinValue()
{
    return this->min_value;
}

float FuzzyVariable::getMaxValue()
{
    return this->max_value;
}