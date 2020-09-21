# include "phy_model.h"

//initial function
PHYModel::PHYModel(int disturb_selector, float s)
{
    //random number based on time
    srand(time(NULL));
    disturb_type = disturb_selector;
    sigma = s;
    gen.seed(rd());
    return;
}

float PHYModel::fix_disturb(int k)
{
    float base[] = {1.1, 1.2, 1.8, 1.7, 1.3, 0.5, 0.4, 0.6, 0.7, 0.8};
    // $k\in[0,9]$
    if(k<0)
        k = 0;
    else if(k>9)
        k = 9;
    float w = base[k];
    int diverse = k/3 + 1;

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
    return w;
}

// The input is the current disturbance state
// The disturbance follows a Markov chain
// The output (next disturbance) only depends on the current one.
float PHYModel::mc_disturb(float prev_w)
{
    // f(x)=\frac{1}{\sigma\sqrt{2\pi}}e^{-\frac{(x-\mu)^2}{2\sigma}}
    // center is determined by \mu=w
    // \sigma is defined by the environment

    // instance of class std::normal_distribution with specific mean and stddev
    std::normal_distribution<float> d(prev_w, sigma);

    // get random number with normal distribution using gen as random source
    float w = d(gen);
    w > w_bound[1] ? w = w_bound[1] : w = w ;
    w < w_bound[0] ? w = w_bound[0] : w = w ;
    prev_w = w;

    return w;
    //return base[k]; // for test
}

float PHYModel::linear_model(int k, float x, float u, float prev_w)
{
    if (k<0)
        k = 0;
    else if (k>9)
        k = 9;
    //x\in[-2, 2]
    if (x < x_bound[0])
        x = x_bound[0];
    else if (x > x_bound[1])
        x = x_bound[1];
    //u\in[0.2, 1.6]
    if (u < u_bound[0])
        u = u_bound[0];
    else if (u > u_bound[1])
        u = u_bound[1];

    //generate disturbance base by given parameters
    float w = 0;
    if (disturb_type == NO_DISTURB)
        w = 0;
    else if (disturb_type == MC_DISTURB)
        w = mc_disturb(prev_w);
    else if (disturb_type == FIX_DISTURB)
        w = fix_disturb(k);

    float x_ = 0.9*x + u - w;
    if(x_<-2)
        x_ = -2;
    else if(x_>2)
        x_ = 2;
    return x_;
}
