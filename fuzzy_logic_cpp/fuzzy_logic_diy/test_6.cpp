#include <algorithm>
#include <iostream>
#include <new>

int main()
{
    int i = 10;
    float *array = new float[i];
    for (int index = 0; index < i; index++)
    {
        array[index] = index - 8 + 1;
        std::cout << array[index] << std::endl;
    }

    float *min = std::min_element(array, array + sizeof(array) / sizeof(array[0]));

    std::cout << *min << std::endl;

    delete array;

    std::cout << array[0] << std::endl;

    return 0;
}