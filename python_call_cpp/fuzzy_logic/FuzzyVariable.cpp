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
    for (auto term : this->list_of_term)
    {
        this->fuzzificated_set.insert({term.getName(), term.compute(this->input_value)});
    }
    // this->printFuzzificatedSet();
}

std::map<std::string, float> FuzzyVariable::directlyCalculateFuzzificatedSet(float input_value)
{
    std::map<std::string, float> temp_fuzzificated_set;
    for (auto &term : this->list_of_term)
    {
        temp_fuzzificated_set.insert({term.getName(), term.compute(input_value)});
    }

    // std::cout << "Variable: " << this->name << std::endl;
    // for (auto output : temp_fuzzificated_set)
    // {
    //     std::cout << "key: " << output.first << ", "
    //               << "value: " << output.second << std::endl;
    // }
    // std::cout << std::endl;

    return temp_fuzzificated_set;
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