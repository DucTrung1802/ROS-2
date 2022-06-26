#ifndef MEMBERSHIPFUNCTION_H
#define MEMBERSHIPFUNCTION_H

namespace FLD
{
    class MF
    {
    public:
        virtual float getValue(float x) = 0;
    };

    class TriangularMF : public MF
    {
    public:
        float a;
        float b;
        float c;

        TriangularMF(float a, float b, float c);
        float getValue(float x);
    };
}

#endif