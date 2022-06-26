#ifndef FUZZYVARIABLE_H
#define FUZZYVARIABLE_H

#include <string>
#include <list>
#include "Term.h"

namespace FLD
{
    class FuzzyVariable
    {
    public:
        std::string name;
        float min_value;
        float max_value;
        std::list<Term> list_of_term;

        FuzzyVariable(std::string name, float min_value, float max_value);
        void addTerm(Term new_term);
    };
}
#endif