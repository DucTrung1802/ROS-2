#ifndef RULEHANDLER_H
#define RULEHANDLER_H

#include <string>
#include <list>

namespace FLD
{
    class FuzzyRule
    {
    };

    class RuleHandler
    {
    public:
        std::list<std::string> keywords = {"if", "then", "and", "or", "is"};

        std::list<FuzzyRule> list_of_fuzzy_rules;

        void parseRule(std::string rule);
        void addRule(FuzzyRule parsed_rule);
        bool checkKeyword(std::string word);
    };
}

#endif