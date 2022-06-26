#ifndef RULEHANDLER_H
#define RULEHANDLER_H

#include <string>
#include <list>

namespace FLD
{
    class ParsedRule
    {
    };

    class RulecHandler
    {
    public:
        std::list<std::string> keywords = {"if", "then", "and", "or", "is"};

        std::list<ParsedRule>
            list_of_parsed_rules;

        RulecHandler();
        ParsedRule parseRule(std::string rule);
    };
}

#endif