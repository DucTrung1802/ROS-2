#include "MembershipFunction.h"

using namespace FLD;

TriangularMF::TriangularMF(float a, float b, float c)
{
    if (!((a <= b) && (b <= c)))
    {
        throw("%f <= %f <= %f is not true!");
    }

    else
    {
        this->a = a;
        this->b = b;
        this->c = c;
    }
}

float TriangularMF::getValue(float x)
{
    if (((this->a == this->b) && (this->b == x)) || ((this->b == this->c) && (this->b == x)) || (this->b == x))
    {
        return 1.0;
    }

    else if (this->a < x && x < this->b)
    {
        return (x - this->a) / (this->b - this->a);
    }

    else if (this->b < x && x < this->c)
    {
        return (this->c - x) / (this->c - this->b);
    }

    else
        return 0.0;
}