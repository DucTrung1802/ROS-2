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
    // TestClass a_new_class = TestClass({100, 200});
    // a_new_class.addNewValue(300);
    // a_new_class.addNewValue(400);
    // // a_new_class.printList();

    // std::map<string, string> myMap = {
    //     {"A", "1"},
    //     {"B", "2"},
    //     {"C", "3"}};

    // for (auto it = myMap.cbegin(); it != myMap.cend(); ++it)
    // {
    //     if (it->first == "A")
    //     {
    //         std::cout << it->first << std::endl;
    //     }
    // }

    // std::string s = "scott>=tiger";
    // std::string delimiter = ">=";
    // std::string token = s.substr(0, s.find(delimiter)); // token is "scott"

    // std::string s = "scott>=tiger>=mushroom";
    // std::string delimiter = ">=";

    // size_t pos = 0;
    // std::string token;
    // while ((pos = s.find(delimiter)) != std::string::npos)
    // {
    //     token = s.substr(0, pos);
    //     std::cout << token << std::endl;
    //     s.erase(0, pos + delimiter.length());
    // }
    // std::cout << s << std::endl;

    std::string rule = "if input1 is mf1 and input2 is mf2 then output1 is mf2";

    std::string antecedent;

    // if (rule.find("if") == -1)
    // {
    //     throw std::invalid_argument("No \"if\" in the rule!");
    // }

    // if (rule.find("then") == -1)
    // {
    //     throw std::invalid_argument("No \"then\" in the rule!");
    // }

    antecedent = rule;
    eraseSubStr(antecedent, "if ");

    // std::cout << antecedent << std::endl;

    // std::string delimiter = "then";
    // std::string token = rule.substr(0, rule.find(delimiter));

    // std::cout << token << std::endl;
    // rule.erase(rule.begin() + 0, rule.begin() + rule.find(delimiter));

    std::string s = "input1 is mf1 and input2 is mf2";
    // s = removeSpaces(s);
    std::string delimiter = " and ";

    size_t pos = 0;
    std::string token;
    while ((pos = s.find(delimiter)) != std::string::npos)
    {
        token = s.substr(0, pos);
        std::cout << token << std::endl;
        s.erase(0, pos + delimiter.length());
    }
    std::cout << s << std::endl;

    return 0;
}