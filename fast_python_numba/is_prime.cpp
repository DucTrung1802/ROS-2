#include <iostream>
#include <cmath>
#include <time.h>

using namespace std;

bool isPrime(int number)
{
    if (number == 2)
        return true;
    if (number <= 1 || number % 2 == 0)
        return false;
    double sqrt_num = sqrt(double(number));
    for (int div = 3; div <= sqrt_num; div += 2)
    {
        if (number % div == 0)
            return false;
    }
    return true;
}

void runProgram(int max_number)
{
    for (int number = 0; number < max_number; number++)
        isPrime(number);
}

int main()
{
    int MAX_NUMBER = 10000000;
    clock_t start, end;
    start = clock();
    runProgram(MAX_NUMBER);
    end = clock();
    cout << (end - start) / ((double)CLOCKS_PER_SEC);
    cout << " (seconds)\n";
    return 0;
}