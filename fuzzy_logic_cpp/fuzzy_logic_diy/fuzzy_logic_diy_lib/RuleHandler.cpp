#include "RuleHandler.h"
#include <iostream>

using namespace FLD;

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
    std::cout << "antecedent: " << antecedent << std::endl;

    // consequent
    std::string consequent = rule;
    consequent.erase(consequent.begin() + 0, consequent.begin() + consequent.find(delimiter));
    eraseSubStr(consequent, "then ");
    std::cout << "consequent: " << consequent << std::endl;

    std::cout << "rule: " << rule << std::endl;
}