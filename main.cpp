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
    int N = 10;
    float x_con[2] = {-2.0, 2.0};
    float u_con[2] = {0.2, 1.6};
    int gran = 10;
    DPModel model(N, x_con, u_con, gran);
    model.estimate_model(100);

    cout << "done" << endl;

    return 0;
}