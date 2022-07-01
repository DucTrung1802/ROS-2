
#include <iostream>
#include <set>
#include <list>

int main()
{
    std::list<int> list = {
        1,
        2,
        3,
        3,
        2,
        3,
        2,
        4};

    std::set<int> s;
    for (int const &i : list)
    {
        s.insert(i);
    }

    for (int const &i : s)
    {
        std::cout << i << ' ';
    }
    std::cout << std::endl;

    return 0;
}