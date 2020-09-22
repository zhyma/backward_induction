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

// The input is the current disturbance state
// The disturbance follows a Markov chain
// The output (next disturbance) only depends on the current one.
float PHYModel::mc_disturb(float w)
{
    // f(x)=\frac{1}{\sigma\sqrt{2\pi}}e^{-\frac{(x-\mu)^2}{2\sigma}}
    // center is determined by \mu=w
    // \sigma is defined by the environment

    // instance of class std::normal_distribution with specific mean and stddev
    std::normal_distribution<float> d(w, sigma);

    // get random number with normal distribution using gen as random source
    float w_ = d(gen);
    w_ > w_bound[1] ? w_ = w_bound[1] : w_ = w_ ;
    w_ < w_bound[0] ? w_ = w_bound[0] : w_ = w_ ;
    w_ = w;

    return w_;
}

int PHYModel::linear_model(int k, float x, float u, float w, float * next)
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
    float w_ = 0;
    if (disturb_type == NO_DISTURB)
        w_ = 0;
    else if (disturb_type == MC_DISTURB)
        w_ = mc_disturb(w);

    float x_ = 0.9*x + u - w;
    if(x_<-2)
        x_ = -2;
    else if(x_>2)
        x_ = 2;
    next[0] = x_;
    next[1] = w_;
    return 0;
}
