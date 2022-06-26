#ifndef FUZZYINPUT_H
#define FUZZYINPUT_H

#include <string>
#include "MembershipFunction.h"

namespace FLD
{
    class FuzzyInput
    {
    public:
        std::string name;
        MF *mf;

        // FuzzyInput(string name, MembershipFunction mf);
        FuzzyInput(std::string name, MF *mf);
        float compute(float x);
    };
}

#endif