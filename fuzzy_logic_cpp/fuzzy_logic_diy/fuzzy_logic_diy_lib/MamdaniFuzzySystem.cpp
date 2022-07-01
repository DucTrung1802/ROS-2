#include <algorithm>
#include "MamdaniFuzzySystem.h"

using namespace FLD;

template <typename T>
bool contains(std::list<T> &listOfElements, const T &element)
{
    // Find the iterator if element in list
    auto it = std::find(listOfElements.begin(), listOfElements.end(), element);
    // return if iterator points to end or not. It points to end then it means element
    //  does not exists in list
    return it != listOfElements.end();
}

int MamdaniFuzzySystem::checkInputVariables(std::list<FuzzyVariable> input_variables)
{
    bool duplicate_keyword = false;
    std::list<std::string> list_of_keyword = rule_handler.getListKeyword();
    for (auto input_var : input_variables)
    {
        // std::cout << input_var->getName() << std::endl;
        // std::cout << *rule_handler.getListKeyword().begin() << std::endl;
        std::list<std::string> list_name = input_var.getListNameOfTerm();

        duplicate_keyword = contains(list_of_keyword, input_var.getName());

        if (duplicate_keyword)
            return 1;

        for (auto name : list_name)
        {
            // std::cout << *name << std::endl;
            duplicate_keyword = contains(list_of_keyword, name);
            if (duplicate_keyword)

                return 2;

            // std::cout << duplicate_keyword << std::endl;
        }
    }
    return 0;
}

void MamdaniFuzzySystem::initializeInferenceSet(std::list<FuzzyVariable> output_variables)
{
    for (auto output_var : output_variables)
    {
        std::map<std::string, float> temp_mf_set;
        for (auto term : output_var.getListOfTerm())
        {
            temp_mf_set.insert({term.getName(), 0.0});
        }
        this->list_of_inference_set.push_back({output_var.getName(), temp_mf_set});
    }
}

MamdaniFuzzySystem::MamdaniFuzzySystem(std::list<FuzzyVariable> input_variables, std::list<FuzzyVariable> output_variables)
{

    if (checkInputVariables(input_variables) == 0)
    {
        // std::cout << "not duplicate" << std::endl;
        this->input_variables = input_variables;
        // for (auto input_var : this->input_variables)
        // {
        //     std::cout << input_var.getName() << std::endl;
        // }
    }
    else
    {
        std::list<std::string> list_of_keyword = rule_handler.getListKeyword();
        std::cout << "List of keyword:";
        for (auto keyword : list_of_keyword)
        {
            std::cout << " \"" << keyword << "\"";
        }
        std::cout << std::endl;

        if (checkInputVariables(input_variables) == 1)
        {
            throw std::invalid_argument("There is at least 01 input fuzzy variable with the same name as keyword!");
        }
        else if (checkInputVariables(input_variables) == 2)
        {
            throw std::invalid_argument("There is at least 01 membership function of input fuzzy variable with the same name as keyword!");
        }
    }

    if (checkInputVariables(output_variables) == 0)
    {
        // std::cout << "not duplicate" << std::endl;
        this->output_variables = output_variables;
    }
    else
    {
        std::list<std::string> list_of_keyword = rule_handler.getListKeyword();
        std::cout << "List of keyword:";
        for (auto keyword : list_of_keyword)
        {
            std::cout << " \"" << keyword << "\"";
        }
        std::cout << std::endl;

        if (checkInputVariables(output_variables) == 1)
        {
            throw std::invalid_argument("There is at least 01 output fuzzy variable with the same name as keyword!");
        }
        else if (checkInputVariables(output_variables) == 2)
        {
            throw std::invalid_argument("There is at least 01 membership function of output fuzzy variable with the same name as keyword!");
        }
    }

    this->rule_handler.addInputFuzzyVariableList(input_variables);
    this->rule_handler.addOutputFuzzyVariableList(output_variables);
    this->rule_handler.makeListInputVariableParameters();
    this->rule_handler.makeListOutputVariableParameters();
    this->initializeInferenceSet(output_variables);
    // for (auto set : this->list_of_inference_set)
    // {
    //     set.printInferenceSet();
    // }
}

size_t MamdaniFuzzySystem::getNumberOfFuzzyRule()
{
    return this->rule_handler.getNumberOfFuzzyRule();
}

void MamdaniFuzzySystem::addRule(std::string rule)
{

    this->rule_handler.addRule(this->rule_handler.parseRule(rule));
}

void MamdaniFuzzySystem::printAllRules()
{
    this->rule_handler.printAllRules();
}

void MamdaniFuzzySystem::addInputValue(std::string variable_name, float input_value)
{
    bool found = false;
    for (auto &variable : this->input_variables)
    {
        if (variable.getName() == variable_name)
        {
            variable.setInputValue(input_value);
            // std::cout << variable.getInputValue() << std::endl;
            found = true;
        }
    }

    if (!found)
    {
        throw std::invalid_argument("\"" + variable_name + "\" is not a name of input variable!");
    }
}

void MamdaniFuzzySystem::checkAllInputValues()
{
    for (auto input_var : this->input_variables)
    {
        if (isnan(input_var.getInputValue()))
        {
            throw std::invalid_argument("Input variable \"" + input_var.getName() + "\"'s input value has not been set!");
        }
    }
}
void MamdaniFuzzySystem::calculateFuzzificatedSets()
{
    for (auto &input_var : this->input_variables)
    {
        input_var.calculateFuzzificatedSet();
        // input_var.printFuzzificatedSet();
    }
}

void MamdaniFuzzySystem::calculateInferenceSets()
{
    for (auto rule : this->rule_handler.getListOfFuzzyRule())
    {
        float *array_of_mf_value;
        int length_array = rule.getAntecedentList().size();
        array_of_mf_value = new float[length_array];

        int index = 0;

        for (auto statement : rule.getAntecedentList())
        {
            for (auto input_var : this->input_variables)
            {
                if (input_var.getName() == statement.front())
                {
                    for (auto pair : input_var.getFuzzificatedSet())
                    {
                        if (pair.first == statement.back())
                        {
                            array_of_mf_value[index] = pair.second;
                            index++;
                            // std::cout << pair.first << std::endl;
                        }
                    }
                }
            }
        }

        float *min = std::min_element(array_of_mf_value, array_of_mf_value + sizeof(array_of_mf_value) / sizeof(array_of_mf_value[0]));

        // std::cout << *min << std::endl;

        for (auto statement : rule.getConsequentList())
        {
            for (auto &set : this->list_of_inference_set)
            {
                if (statement.front() == set.getOutputVarName())
                {
                    if (*min >= set.getValueMf(statement.back()))
                    {
                        set.setValueMf(statement.back(), *min);
                    }
                    // std::cout << statement.front() << std::endl;
                }
                set.printInferenceSet();
            }
        }

        // std::cout << length_array << std::endl;
        // for (auto statement :)
    }
}

void MamdaniFuzzySystem::defuzzify()
{
}

float MamdaniFuzzySystem::calculate()
{
    checkAllInputValues();

    calculateFuzzificatedSets();

    calculateInferenceSets();

    defuzzify();
    // std::cout << "hello" << std::endl;
}

std::list<FuzzyVariable> MamdaniFuzzySystem::getInputVariables()
{
    return this->input_variables;
}

std::list<FuzzyVariable> MamdaniFuzzySystem::getOutputVariables()
{
    return this->output_variables;
}