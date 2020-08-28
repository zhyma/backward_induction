#include "iostream"
using namespace std;
#include "model.h"

int main()
{
    int a;
    while(true)
    {
        cin >> a;
        float x = linear_model(1, 0.5, a);
        cout << x << endl;
    }
    
}