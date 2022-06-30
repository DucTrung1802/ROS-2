#include <iostream>
#include <list>
#include <typeinfo>
#include <map>
#include <cstring>
#include <algorithm>

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

int main()
{
    std::list<int> test_list = {1, 2, 3, 4, 5, 6, 7};

    for (auto i : test_list)
    {
        std::cout << typeid(i).name() << std::endl;
    }

    return 0;
}