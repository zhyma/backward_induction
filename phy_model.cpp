# include "phy_model.h"

float fix_disturb(int k)
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
    //srand(time(NULL));
    int r = rand();
    //cout << r << endl;
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
            break;
        case 4:
            // -0.2 or -0.1 or 0 or +0.1 or +0.2
            w += (r > 1.5)? 0.1*(r-1):0.1*(r-2);
            break;
        default:
            // also case 1
            break;
    }
    // cout << "diverse: " << diverse << "; r: " << r << " w:"<< w << endl;
    return w;
    //return base[k]; // for test
}

// The input is the current disturbance state
// The disturbance follows a Markov chain
// The output (next disturbance) only depends on the current one.
float mc_disturb(float w)
{
    //random number based on time
    //srand(time(NULL));
    int r = rand();
    //cout << r << endl;
    r = (r % 100);// $r\in[0,diverse-1]$
    float w_ = w;
    if(r < 12)
    {
        //w_ = w-0.2, 12% chance
        if(abs(w-0) < 0.001)
            w_ = 0;
        else if(abs(w-0.1) < 0.001)
            w_ = 0;
        else
            w_ = w-0.2;
    }
    else if(r < 12+25)
    {
        // w_ = w-0.1, 25% chance
        if(abs(w-0) < 0.001)
            w_ = 0;
        else
            w_ = w-0.1;
    }
    else if(r < 12+25+26)
    {
        // w_ = w, 26% chance
        w_ = w;
    }
    else if(r < 12+25+26+25)
    {
        //w_ = w+0.1, 25% chance
        if(abs(w-2) < 0.001)
            w_ = 2.0;
        else
            w_ = w + 0.1;
    }
    else
    {
        //w_ = w+0.2
        if(abs(w-2) < 0.001)
            w_ = 2.0;
        else if(abs(w-1.9) < 0.001)
            w_ = 2.0;
        else
            w_ = w + 0.2;
    }
    // cout << "diverse: " << diverse << "; r: " << r << " w:"<< w << endl;
    return w_;
    //return base[k]; // for test
}

float linear_model(int k, float x, float u)
{
    if(k<0)
        k = 0;
    else if(k>9)
        k = 9;
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
    float w = 0;//disturb(k);
    float x_ = 0.9*x + u - w;
    if(x_<-2)
        x_ = -2;
    else if(x_>2)
        x_ = 2;
    return x_;
}

// Markov chain disturbance test
int mc_test()
{
    float w_ = 0;
    ofstream out_prob;
    out_prob.open("../mc_test.csv", ios::out);
    for(int w = 0;w <=20; w += 1)
    {
        float in = (float) w/10.0;
        cout << in << endl;
        int box[21]={};
        for(int i = 0;i < 1000; ++i)
        {
            
            w_ = mc_disturb(in);
            int idx = round(w_*10);
            box[idx] += 1;
        }
        for(int i = 0;i<=20; ++i)
            out_prob << box[i] << ',';
        out_prob << '\n';
    }
    out_prob.close();
    cout << "Test saved to file." << endl;
    return 0;
}

// // For test
// int main()
// {
//     // float x = linear_model(1, 2, 10);
//     // cout << x << " ";
    
//     mc_test();
    
//     return 0;
// }