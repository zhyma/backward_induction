#include "iostream"
using namespace std;
#include "model.h"

int main()
{
    int a;
    while(true)
    {
        cin >> a;
        float x = linear_model(a, 1, 0.5);
        cout << x << endl;
    }
    
}