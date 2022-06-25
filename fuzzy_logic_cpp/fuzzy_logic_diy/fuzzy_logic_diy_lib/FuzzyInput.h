#ifndef FUZZYINPUT_H
#define FUZZYINPUT_H

#include <string>

namespace FLD
{
    class FuzzyInput
    {
    private:
        string name;
        TriangularMF mf;

    public:
        // FuzzyInput(string name, MembershipFunction mf);
        FuzzyInput(string name, TriangularMF mf);
        float compute();
    };
}

#endif