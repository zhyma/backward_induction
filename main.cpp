#include "iostream"
using namespace std;
#include "model.h"
#include "forward_search.h"

// int main()
// {
//     int a;
//     while(true)
//     {
//         cin >> a;
//         float x = linear_model(a, 1, 0.5);
//         cout << x << endl;
//     }
    
// }
int main()
{
    DPModel model;
    model.estimate_model(100);

    cout << "done" << endl;

    return 0;
}