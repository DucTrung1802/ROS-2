#include <iostream>
#include <map>
#include <algorithm>
#include "RuleHandler.h"
#include "FuzzyRule.h"

using namespace FLD;

std::string removeSpaces(std::string str)
{
    str.erase(remove(str.begin(), str.end(), ' '), str.end());
    return str;
}

void printListStr(std::list<std::string> const &list)
{
    std::cout << "list string: [";
    for (auto &i : list)
    {
        std::cout << " " << i << " ";
    }
    std::cout << "]" << std::endl;
}

void printNestedList(std::list<std::list<std::string>> const &list)
{
    std::cout << "1-layer nested list string: [";

    for (auto &i : list)
    {
        std::cout << "[";
        for (auto &j : i)
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

void RuleHandler::printAntecedentList()
{
    printNestedList(this->_antecedent_list);
}

// support AND only
std::list<std::list<std::string>> RuleHandler::antecedentHandler(std::string antecedent)
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

    for (auto &statement : list_of_statement_between_AND)
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

    // printNestedListStr(antecedent_list);

    return _antecedent_list;
}

void RuleHandler::parseRule(std::string rule)
{
    rule = "if input1 is mf1 and input2 is mf2 then output1 is mf2";

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

    temp_fuzzy_rule.addAntecedentList(antecedentHandler(antecedent));

    // printNestedList(temp_fuzzy_rule.getAntecedentList());
}
