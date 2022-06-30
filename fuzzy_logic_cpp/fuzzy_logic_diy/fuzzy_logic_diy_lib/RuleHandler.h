#ifndef RULEHANDLER_H
#define RULEHANDLER_H

#include <string>
#include <list>
#include <set>
#include "FuzzyRule.h"
#include "FuzzyVariable.h"

namespace FLD
{

    class RuleHandler
    {
    public:
        std::list<std::string> keywords = {"if", "then", "and", "or", "is"};
        std::list<std::list<std::string>> _antecedent_list;
        std::list<std::list<std::string>> _consequent_list;
        std::list<FuzzyRule> list_of_fuzzy_rules;
        FuzzyRule temp_fuzzy_rule;

        std::list<FuzzyVariable> input_variables;
        std::list<FuzzyVariable> output_variables;

        std::set<std::string> input_variable_names;
        std::set<std::string> input_variable_term_names;

        std::set<std::string> output_variable_names;
        std::set<std::string> output_variable_term_names;

        FuzzyRule
        parseRule(std::string rule);

        void addRule(FuzzyRule parsed_rule);

        void printNestedList(std::list<std::list<std::string>> const &list);

        bool checkKeyword(std::string word);

        bool FuzzyRuleCheck(FuzzyRule rule);

        std::list<std::list<std::string>> antecedentParser(std::string antecedent);

        std::list<std::list<std::string>> consequentParser(std::string consequent);

        std::list<std::string> getListKeyword();

        size_t getNumberOfFuzzyRule();

        void addInputFuzzyVariableList(std::list<FuzzyVariable> input_variables);
        void addOutputFuzzyVariableList(std::list<FuzzyVariable> output_variables);

        void makeListInputVariableParameters();
        void makeListOutputVariableParameters();
    };
}

#endif