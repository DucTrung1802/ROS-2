// CPP program to demonstrate multithreading
// using three different callables.
#include <chrono>
#include <iostream>
#include <thread>
#include <unistd.h>

using namespace std;
using std::chrono::duration;
using std::chrono::high_resolution_clock;

void foo(int &Z)
{
    Z += 1;
    usleep(1000);
}

int main()
{

    int x = 1;
    int y = 2;
    int z = 3;

    auto t1 = high_resolution_clock::now();

    thread th1(foo, std::ref(x));
    thread th2(foo, std::ref(y));
    thread th3(foo, std::ref(z));

    th1.join();
    th2.join();
    th3.join();

    auto t2 = high_resolution_clock::now();

    duration<double, std::milli> runtime = t2 - t1;

    std::cout << x << " " << y << " " << z << std::endl;
    std::cout << runtime.count() << " ms" << std::endl;

    return 0;
}
