#ifndef MAMDANIFUZZYSYSTEM_H
#define MAMDANIFUZZYSYSTEM_H

#include <list>
#include <algorithm>
#include <string>
#include "FuzzyVariable.h"
#include "RuleHandler.h"
#include "InferenceSet.h"

namespace FLD
{
    class MamdaniFuzzySystem
    {
    public:
        std::list<FuzzyVariable> input_variables;
        std::list<FuzzyVariable> output_variables;
        RuleHandler rule_handler;
        std::list<InferenceSet> list_of_inference_set;

        MamdaniFuzzySystem(std::list<FuzzyVariable> input_variables, std::list<FuzzyVariable> output_variables);
        std::list<FuzzyVariable> getOutputVariables();
        std::list<FuzzyVariable> getInputVariables();
        int checkInputVariables(std::list<FuzzyVariable> input_variables);
        int checkOutputVariables(std::list<FuzzyVariable> output_variables);
        float calculate();
        size_t getNumberOfFuzzyRule();
        void addRule(std::string rule);
        void addInputValue(std::string variable_name, float input_value);
        void checkAllInputValues();
        void printAllRules();
        void initializeInferenceSet(std::list<FuzzyVariable> output_variables);
        void calculateFuzzificatedSets();
        void calculateInferenceSets();
        void defuzzify();
    };
}

#endif