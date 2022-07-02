// C++ program to demonstrate default behaviour of
// sort() in STL.
#include <bits/stdc++.h>
using namespace std;

int main()
{
    int n = 20;
    int *new_arr = new int[n];
    for (int i = 0; i < n; i++)
    {
        new_arr[i] = i;
    }
    // {1, 5, 8, 9, 6, 7, 3, 4, 2, 0};

    /*Here we take two parameters, the beginning of the
    array and the length n upto which we want the array to
    be sorted*/
    sort(new_arr, new_arr + n);

    cout << "\nArray after sorting using "
            "default sort is : \n";
    for (int i = 0; i < n; ++i)
        cout << new_arr[i] << " ";
    cout << endl;

    return 0;
}
