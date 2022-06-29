// CPP program to Remove spaces
// from a given string

#include <iostream>
#include <list>
#include <algorithm>
using namespace std;

// Function to remove all spaces from a given string
string removeSpaces(string str)
{
    str.erase(remove(str.begin(), str.end(), ' '), str.end());
    return str;
}

// Driver program to test above function
int main()
{
    int my_var = 3;
    std::list<int> my_list{1, 2, 3, 4};

    bool found = (std::find(my_list.begin(), my_list.end(), my_var) != my_list.end());

    std::cout << found << std::endl;
    return 0;
}

// This code is contributed by Divyam Madaan
