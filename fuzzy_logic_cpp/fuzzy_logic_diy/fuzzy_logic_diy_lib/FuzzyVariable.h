#ifndef FUZZYVARIABLE_H
#define FUZZYVARIABLE_H

#include <string>
#include <list>
#include "Term.h"
#include <iostream>

namespace FLD
{
    enum variable_type
    {
        INPUT,
        OUTPUT,
        NONE,
    };

    class FuzzyVariable
    {
    public:
        int number_of_term;
        float min_value;
        float max_value;
        float value;
        std::list<Term> list_of_term;
        std::string name;
        std::list<float> fuzzificated_set;
        variable_type type = NONE;

        FuzzyVariable(std::string name, float min_value, float max_value);
        void addTerm(Term new_term);
        void calculateFuzzificatedSet(float value);
        void printFuzzificatedSet();
    };
}
#endif