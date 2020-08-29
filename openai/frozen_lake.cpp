#include "stdlib.h"
#include <iostream>
#include <vector>
#include <string>

using namespace std;

char MAPS[][4] = {{'S','F','F','F'},
                 {'F','H','F','H'},
                 {'F','F','F','H'},
                 {'H','F','F','G'}};

int main()
{
    for (int i=0 ;i < 15;i++)
    {
        char t = MAPS[i/4][i%4];
        cout << t << " ";
    }
    cout << endl;
}