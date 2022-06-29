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
    for (auto input_var = input_variables.begin(); input_var != input_variables.end(); input_var++)
    {
        // std::cout << input_var->getName() << std::endl;
        // std::cout << *rule_handler.getListKeyword().begin() << std::endl;
        std::list<std::string> list_name = input_var->getListNameOfTerm();

        duplicate_keyword = contains(list_of_keyword, input_var->getName());

        if (duplicate_keyword)
            return 1;

        for (auto name = list_name.begin(); name != list_name.end(); name++)
        {
            // std::cout << *name << std::endl;
            duplicate_keyword = contains(list_of_keyword, *name);
            if (duplicate_keyword)

                return 2;

            // std::cout << duplicate_keyword << std::endl;
        }
    }
    return 0;
}

MamdaniFuzzySystem::MamdaniFuzzySystem(std::list<FuzzyVariable> input_variables, std::list<FuzzyVariable> output_variables)
{

    if (checkInputVariables(input_variables) == 0)
    {
        // std::cout << "not duplicate" << std::endl;
        this->input_variables = input_variables;
    }
    else
    {
        std::list<std::string> list_of_keyword = rule_handler.getListKeyword();
        std::cout << "List of keyword:";
        for (auto keyword = list_of_keyword.begin(); keyword != list_of_keyword.end(); keyword++)
        {
            std::cout << " \"" << *keyword << "\"";
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
        for (auto keyword = list_of_keyword.begin(); keyword != list_of_keyword.end(); keyword++)
        {
            std::cout << " \"" << *keyword << "\"";
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
}