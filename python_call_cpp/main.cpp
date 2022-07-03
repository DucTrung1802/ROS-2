#include "lib.h"

using namespace hello;

extern "C"
{
    void My_Function(float a)
    {
        Test test = Test();
        test.print(a);
    }
}