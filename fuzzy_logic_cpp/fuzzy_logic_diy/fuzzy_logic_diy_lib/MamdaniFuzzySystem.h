#ifndef MAMDANIFUZZYSYSTEM_H
#define MAMDANIFUZZYSYSTEM_H

#include <list>
#include <algorithm>
#include <string>
#include "FuzzyVariable.h"
#include "RuleHandler.h"

namespace FLD
{
    class MamdaniFuzzySystem
    {
    public:
        std::list<FuzzyVariable> input_variables;
        std::list<FuzzyVariable> output_variables;
        RuleHandler rule_handler;

        MamdaniFuzzySystem(std::list<FuzzyVariable> input_variables, std::list<FuzzyVariable> output_variables);
        int checkInputVariables(std::list<FuzzyVariable> input_variables);
        int checkOutputVariables(std::list<FuzzyVariable> output_variables);
        float calculate();
        size_t getNumberOfFuzzyRule();
        void addRule(std::string rule);
        void printAllRules();
    };
}

#endif