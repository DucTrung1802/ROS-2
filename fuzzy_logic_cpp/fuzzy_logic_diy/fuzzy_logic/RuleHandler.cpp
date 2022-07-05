#include <iostream>
#include <map>
#include <algorithm>
#include "RuleHandler.h"
#include "FuzzyRule.h"

using namespace FLD;

template <typename T>
bool contains(std::set<T> &listOfElements, const T &element)
{
    // Find the iterator if element in list
    auto it = std::find(listOfElements.begin(), listOfElements.end(), element);
    // return if iterator points to end or not. It points to end then it means element
    //  does not exists in list
    return it != listOfElements.end();
}

std::string removeSpaces(std::string str)
{
    str.erase(remove(str.begin(), str.end(), ' '), str.end());
    return str;
}

void printListStr(std::list<std::string> const &list)
{
    std::cout << "list string: [";
    for (auto i : list)
    {
        std::cout << " " << i << " ";
    }
    std::cout << "]" << std::endl;
}

void printNestedListStr(std::list<std::list<std::string>> const &list)
{
    std::cout << "1-layer nested list string: [";

    for (auto i : list)
    {
        std::cout << "[";
        for (auto j : i)
        {
            std::cout << " " << j << " ";
        }
        std::cout << "]";
    }
    std::cout << "]" << std::endl;
}

void eraseSubStr(std::string &mainStr, const std::string &toErase)
{
    // Search for the substring in string
    size_t pos = mainStr.find(toErase);
    if (pos != std::string::npos)
    {
        // If found then erase it from string
        mainStr.erase(pos, toErase.length());
    }
}

void sliceString(std::string str, std::string delimiter)
{

    str = "input1 is mf1 and input2 is mf2";
    delimiter = " and ";

    size_t pos = 0;
    std::string token;
    while ((pos = str.find(delimiter)) != std::string::npos)
    {
        token = str.substr(0, pos);
        // each statement except the last statement
        std::cout << token << std::endl;
        str.erase(0, pos + delimiter.length());
    }
    // the last statement
    std::cout << str << std::endl;
}

void RuleHandler::printNestedList(std::list<std::list<std::string>> const &list)
{
    printNestedListStr(list);
}

// support AND only
std::list<std::list<std::string>> RuleHandler::antecedentParser(std::string antecedent)
{
    // std::cout << antecedent << std::endl;

    // LIST_OF_STATEMENT_BETWEEN_AND
    std::list<std::string> list_of_statement_between_AND = {};

    // ====== slice ======

    std::string delimiter = " and ";
    size_t pos = 0;
    std::string token;
    while ((pos = antecedent.find(delimiter)) != std::string::npos)
    {
        token = antecedent.substr(0, pos);
        // each statement except the last statement
        list_of_statement_between_AND.push_back(token);
        // std::cout << token << std::endl;
        antecedent.erase(0, pos + delimiter.length());
    }
    // the last statement
    list_of_statement_between_AND.push_back(antecedent);

    // ====== slice ======#

    // std::cout << str << std::endl;
    // printList(list_of_statement_between_AND);

    // ANTECEDENT_LIST

    std::list<std::list<std::string>> _antecedent_list;

    for (auto statement : list_of_statement_between_AND)
    {
        // std::cout << statement << std::endl;

        std::list<std::string> temp_list = {};
        std::string delimiter = " is ";
        size_t pos = 0;
        std::string token;
        while ((pos = statement.find(delimiter)) != std::string::npos)
        {
            token = statement.substr(0, pos);
            // each statement except the last statement
            temp_list.push_back(token);
            // std::cout << token << std::endl;
            statement.erase(0, pos + delimiter.length());
        }
        // the last statement
        statement = removeSpaces(statement);
        temp_list.push_back(statement);
        // printListStr(temp_list);
        _antecedent_list.push_back(temp_list);
    }

    // printNestedList(_antecedent_list);

    return _antecedent_list;
}

// support AND only
std::list<std::list<std::string>> RuleHandler::consequentParser(std::string consequent)
{
    // std::cout << consequent << std::endl;

    // std::cout << consequent << std::endl;

    // LIST_OF_STATEMENT_BETWEEN_AND
    std::list<std::string> list_of_statement_between_AND = {};

    // ====== slice ======

    std::string delimiter = " and ";
    size_t pos = 0;
    std::string token;
    while ((pos = consequent.find(delimiter)) != std::string::npos)
    {
        token = consequent.substr(0, pos);
        // each statement except the last statement
        list_of_statement_between_AND.push_back(token);
        // std::cout << token << std::endl;
        consequent.erase(0, pos + delimiter.length());
    }
    // the last statement
    list_of_statement_between_AND.push_back(consequent);

    // ====== slice ======#

    // std::cout << str << std::endl;
    // printList(list_of_statement_between_AND);

    // consequent_LIST

    std::list<std::list<std::string>> _consequent_list;

    for (auto statement : list_of_statement_between_AND)
    {
        // std::cout << statement << std::endl;

        std::list<std::string> temp_list = {};
        std::string delimiter = " is ";
        size_t pos = 0;
        std::string token;
        while ((pos = statement.find(delimiter)) != std::string::npos)
        {
            token = statement.substr(0, pos);
            // each statement except the last statement
            temp_list.push_back(token);
            // std::cout << token << std::endl;
            statement.erase(0, pos + delimiter.length());
        }
        // the last statement
        statement = removeSpaces(statement);
        temp_list.push_back(statement);
        // printListStr(temp_list);
        _consequent_list.push_back(temp_list);
    }

    // printNestedList(_consequent_list);

    return _consequent_list;
}

void RuleHandler::FuzzyRuleCheck(FuzzyRule rule)
{
    std::list<std::list<std::string>>
        temp_antecedent_list = rule.getAntecedentList();
    for (auto list_str : temp_antecedent_list)
    {
        std::list<std::string> temp_list_str = list_str;
        std::string temp_variable_name = temp_list_str.front();
        std::string temp_term_name = temp_list_str.back();

        if (!(contains(this->input_variable_names, temp_variable_name)))
        {
            throw std::invalid_argument("Error occured: \"" + temp_variable_name + "\" is not a name of an input variable!");
        }

        if (!(contains(this->input_variable_term_names, temp_term_name)))
        {
            throw std::invalid_argument("Error occured: \"" + temp_term_name + "\" is not a name of a term of an input variable!");
        }

        // for (auto name : temp_list_str)
        // {
        //     std::cout << name << std::endl;
        // }
    }

    std::list<std::list<std::string>>
        temp_consequent_list = rule.getConsequentList();
    for (auto list_str : temp_consequent_list)
    {
        std::list<std::string> temp_list_str = list_str;
        std::string temp_variable_name = temp_list_str.front();
        std::string temp_term_name = temp_list_str.back();

        if (!(contains(this->output_variable_names, temp_variable_name)))
        {
            throw std::invalid_argument("Error occured: \"" + temp_variable_name + "\" is not a name of an output variable!");
        }

        if (!(contains(this->output_variable_term_names, temp_term_name)))
        {
            throw std::invalid_argument("Error occured: \"" + temp_term_name + "\" is not a name of a term of an output variable!");
        }

        // for (auto name : temp_list_str)
        // {
        //     std::cout << name << std::endl;
        // }
    }
}

FuzzyRule RuleHandler::parseRule(std::string rule)
{

    temp_fuzzy_rule.setTextFormOfRule(rule);

    if (rule.find("if") == -1)
    {
        throw std::invalid_argument("No \"if\" in the rule!");
    }

    if (rule.find("then") == -1)
    {
        throw std::invalid_argument("No \"then\" in the rule!");
    }

    std::string delimiter = "then";

    // antecedent
    std::string antecedent = rule.substr(0, rule.find(delimiter));
    eraseSubStr(antecedent, "if ");
    // std::cout << "antecedent: " << antecedent << std::endl;

    // consequent
    std::string consequent = rule;
    consequent.erase(consequent.begin() + 0, consequent.begin() + consequent.find(delimiter));
    eraseSubStr(consequent, "then ");
    // std::cout << "consequent: " << consequent << std::endl;

    // std::cout << "rule: " << rule << std::endl;

    temp_fuzzy_rule.addAntecedentList(antecedentParser(antecedent));
    temp_fuzzy_rule.addConsequentList(consequentParser(consequent));

    return temp_fuzzy_rule;
}

std::list<std::string> RuleHandler::getListKeyword()
{
    return this->keywords;
}

size_t RuleHandler::getNumberOfFuzzyRule()
{
    return this->list_of_fuzzy_rules.size();
}

void RuleHandler::addRule(FuzzyRule rule)
{
    FuzzyRuleCheck(rule);
    this->list_of_fuzzy_rules.push_back(rule);
}

void RuleHandler::addInputFuzzyVariableList(std::list<FuzzyVariable> &input_variables)
{
    this->input_variables = input_variables;
}

void RuleHandler::addOutputFuzzyVariableList(std::list<FuzzyVariable> &output_variables)
{
    this->output_variables = output_variables;
}

void RuleHandler::makeListInputVariableParameters()
{
    for (auto variable : this->input_variables)
    {
        this->input_variable_names.insert(variable.getName());

        std::list<std::string> list_name = variable.getListNameOfTerm();

        for (auto name : list_name)
        {
            this->input_variable_term_names.insert(name);
        }
    }

    // for (auto name = this->input_variable_names.begin(); name != this->input_variable_names.end(); name++)
    // {
    //     std::cout << *name << std::endl;
    // }

    // for (auto name = this->input_variable_term_names.begin(); name != this->input_variable_term_names.end(); name++)
    // {
    //     std::cout << *name << std::endl;
    // }
}

void RuleHandler::makeListOutputVariableParameters()
{
    for (auto variable : this->output_variables)
    {
        this->output_variable_names.insert(variable.getName());

        std::list<std::string> list_name = variable.getListNameOfTerm();

        for (auto name : list_name)
        {
            this->output_variable_term_names.insert(name);
        }
    }

    // for (auto name = thi->output_variable_names.begin(); name != this->output_variable_names.end(); name++)
    // {
    //     std::cout << *name << std::endl;
    // }

    // for (auto name = this->output_variable_term_names.begin(); name != this->output_variable_term_names.end(); name++)
    // {
    //     std::cout << *name << std::endl;
    // }
}

void RuleHandler::printAllRules()
{
    if (this->list_of_fuzzy_rules.size() == 0)
    {
        std::cout << "This system has no rule!" << std::endl;
        return;
    }

    int i = 1;
    for (auto rule : this->list_of_fuzzy_rules)
    {
        std::cout << "Rule " << i << ": " << std::endl;
        std::cout << "[";
        for (auto statement : rule.getAntecedentList())
        {
            std::cout << "[";
            for (auto element : statement)
            {
                std::cout << " " << element << " ";
            }
            std::cout << "]";
        }
        std::cout << "]";
        std::cout << " => ";

        std::cout << "[";
        for (auto statement : rule.getConsequentList())
        {
            std::cout << "[";
            for (auto element : statement)
            {
                std::cout << " " << element << " ";
            }
            std::cout << "]";
        }
        std::cout << "]";

        std::cout << std::endl;
        i++;
    }
}

std::list<FuzzyRule> RuleHandler::getListOfFuzzyRule()
{
    return this->list_of_fuzzy_rules;
}