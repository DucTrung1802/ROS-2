#ifndef Term_H
#define Term_H

#include <string>
#include "MembershipFunction.h"

namespace FLD
{
    class Term
    {
    public:
        std::string name;
        MF *mf;

        Term(std::string name, MF *mf);
        float compute(float x);
    };
}

#endif