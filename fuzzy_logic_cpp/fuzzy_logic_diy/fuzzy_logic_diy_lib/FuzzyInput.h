#ifndef FUZZYINPUT_H
#define FUZZYINPUT_H

#include <string>

namespace FLD
{
    class FuzzyInput
    {
    private:
        // string name;
        // MembershipFunction mf;
        float a;
        float b;

    public:
        // FuzzyInput(string name, MembershipFunction mf);
        FuzzyInput(float a, float b);
        float compute();
    };
}

#endif