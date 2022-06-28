#ifndef RULEHANDLER_H
#define RULEHANDLER_H

#include <string>
#include <list>
#include "FuzzyRule.h"

namespace FLD
{

    class RuleHandler
    {
    public:
        std::list<std::string> keywords = {"if", "then", "and", "or", "is"};
        std::list<std::list<std::string>> _antecedent_list;
        std::list<FuzzyRule> list_of_fuzzy_rules;
        FuzzyRule temp_fuzzy_rule;

        void parseRule(std::string rule);
        void addRule(FuzzyRule parsed_rule);
        std::list<std::list<std::string>> antecedentHandler(std::string antecedent);
        void printAntecedentList();
        bool checkKeyword(std::string word);
    };
}

#endif