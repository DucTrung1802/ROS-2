#ifndef FUZZYVARIABLE_H
#define FUZZYVARIABLE_H

#include <string>
#include <list>
#include <set>
#include <map>
#include <iostream>
#include <math.h>
#include "Term.h"

namespace FLD
{
    class FuzzyVariable
    {
    public:
        int number_of_term;
        float min_value;
        float max_value;
        float input_value = sqrt(-1);
        std::list<Term> list_of_term;
        std::list<std::string> list_name_of_term;
        std::string name;
        std::map<std::string, float> fuzzificated_set;

        FuzzyVariable(std::string name, float min_value, float max_value);
        void setInputValue(float input_value);
        float getInputValue();
        void addTerm(Term new_term);
        void calculateFuzzificatedSet(float value);
        void printFuzzificatedSet();
        std::string getName();
        std::list<Term> getListOfTerm();
        std::list<std::string> getListNameOfTerm();
    };
}
#endif