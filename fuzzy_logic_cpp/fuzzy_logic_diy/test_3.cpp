#include <iostream>
#include <list>
#include <string>
#include <algorithm>
/*
 * Generic function to find if an element of any type exists in list
 */
template <typename T>
bool contains(std::list<T> &listOfElements, const T &element)
{
    // Find the iterator if element in list
    auto it = std::find(listOfElements.begin(), listOfElements.end(), element);
    // return if iterator points to end or not. It points to end then it means element
    //  does not exists in list
    return it != listOfElements.end();
}
int main()
{
    std::list<std::string> listOfStrs =
        {"is", "of", "the", "Hi", "Hello", "from"};
    /* Use the same generic function with list of Strings */
    // Check if an element exists in list
    bool result = contains(listOfStrs, std::string("is"));
    std::cout << result << std::endl;
    // Check if an element exists in list
    result = contains(listOfStrs, std::string("day"));
    std::cout << result << std::endl;
    /* Use the same generic function with list of int */
    // List of ints
    std::list<int> listOfNum =
        {1, 2, 3, 4, 6, 7, 8};
    // Check if an element exists in list
    result = contains(listOfNum, 3);
    std::cout << result << std::endl;
    // Check if an element exists in list
    result = contains(listOfNum, 3);
    std::cout << result << std::endl;

    auto element = listOfNum.begin();
    std::advance(element, );
    std::cout << *element << std::endl;
}