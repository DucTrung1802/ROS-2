#include <iostream>
#include <list>
#include <typeinfo>
#include <map>
#include <cstring>
#include <algorithm>
#include <math.h>

using namespace std;

class TestClass
{
public:
    std::list<float> new_list;
    std::list<std::string> keywords;

    TestClass(std::list<float> new_list)
    {
        keywords = {"abcd", "hello"};
        this->new_list = new_list;
    }

    void addNewValue(float new_value)
    {
        new_list.push_back(new_value);
        // std::cout << typeid(new_list.size()).name() << std::endl;
    }

    void printList()
    {
        // for (auto item = keywords.cbegin(); item != keywords.cend(); item++)
        // {
        //     cout << *item << endl;
        // }
        auto output_value = new_list.begin();
        std::advance(output_value, 100);
        std::cout << *output_value << std::endl;
    }
};

void eraseSubStr(std::string &mainStr, const std::string &toErase)
{
    // Search for the substring in string
    size_t pos = mainStr.find(toErase);
    if (pos != std::string::npos)
    {
        // If found then erase it from string
        mainStr.erase(pos, toErase.length());
    }
}

std::string removeSpaces(std::string str)
{
    str.erase(remove(str.begin(), str.end(), ' '), str.end());
    return str;
}

void add(std::list<int> &a)
{
    for (auto &index : a)
    {
        index += 1;
    }
}

void display(std::list<int> a)
{
    for (auto i : a)
    {
        std::cout << i << std::endl;
    }
}

bool comp(int a, int b)
{
    return (a < b);
}

int min_in_list(std::list<int> the_list)
{
    int *min_value = std::min_element(the_list, the_list + sizeof(the_list) / sizeof(the_list[0]), comp);
    return *min_value;
}

int main()
{
    // std::list<int> test_list = {1, 2, 3, 4, 5, 6, 7};

    // for (auto &i : test_list)
    // {
    //     i += 1;
    // }

    // for (auto index : test_list)
    // {
    //     std::cout << index << std::endl;
    // }

    // std::map<std::string, float> gquiz1;

    // gquiz1.insert({"mf1", 40});
    // gquiz1.insert({"mf2", 30});
    // gquiz1.insert({"mf3", 60});
    // gquiz1.insert({"mf4", 20});
    // gquiz1.insert({"mf5", 50});

    // // for (auto _pair : gquiz1)
    // // {
    // //     std::cout << _pair.first << " " << _pair.second << std::endl;
    // // }

    // std::map<std::string, float> gquiz2 = gquiz1;

    // for (auto _pair : gquiz2)
    // {
    //     std::cout << _pair.first << " " << _pair.second << std::endl;
    // }
    // C++ program to demonstrate the use of std::min

    // int a = 5;
    // int b = 7;
    // std::cout << std::min(a, b) << "\n";

    // // Returns the first one if both the numbers
    // // are same
    // std::cout << std::min(5, 6);

    // std::cout << std::endl;

    // int init_list[] = {1, 2, 3, -10, 5, 6, 4, 3, 2, -99};

    // int min_val = min_in_list(init_list);

    // std::cout << min_val << std::endl;

        return 0;
}