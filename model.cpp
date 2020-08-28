# include "model.h"

float disturb(int k)
{
    float base[] = {1.1, 1.2, 1.8, 1.7, 1.3, 0.5, 0.4, 0.6, 0.7, 0.8};
    // $k\in[0,9]$
    if(k<0)
        k = 0;
    else if(k>9)
        k = 9;
    float w = base[k];
    int diverse = k/3 + 1;
    //random number based on time
    srand(time(NULL));
    int r = rand();
    cout << r << endl;
    r = (r % (diverse));// $r\in[0,diverse-1]$
    switch(diverse)
    {
        case 2:
            // -0.1 or +0.1
            w += (r > 0.5)? 0.1:-0.1;
            break;
        case 3:
            // -0.1 or 0 or +0.1
            w += 0.1*r;
        case 4:
            // -0.2 or -0.1 or 0 or +0.1 or +0.2
            w += (r > 1.5)? 0.1*(r-1):0.1*(r-2);
        default:
            // also case 1
            break;
    }
    cout << "diverse: " << diverse << "; r: " << r << " w:"<< w << endl;
    return w;
}

float linear_model(float x, float u, int k)
{
    //x\in[-2, 2]
    if(x < -2)
        x = -2;
    else if(x > 2)
        x = 2;
    //u\in[0.2, 1.6]
    if(u < 0.2)
        u = 0.2;
    else if(u > 1.6)
        u = 1.6;
    //generate disturbance base on given k
    float wk = disturb(k);
    float x_ = 0.9*x + u - wk;
    if(x_<-2)
        x_ = -2;
    else if(x>2)
        x_ = 2;
    return x_;
}

// int main()
// {
//     // float x = linear_model(10, 2, 1);
//     // cout << x << " ";
//     int k;
//     while(true)
//     {
//         cin >> k;
//         float w = disturb(k);
//     }
//     return 0;
// }