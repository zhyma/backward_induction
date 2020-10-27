# include "shared_phy.h"


linear_model(int k, float x, float w, float u, float * next)
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
    float w_;
    if (disturb_type == NO_DISTURB)
        w_ = 0;
    else if (disturb_type == MC_DISTURB)
        w_ = mc_disturb(w);

    float x_ = 0.9*x + u - w;
    if(x_ < x_bound[0])
        x_ = x_bound[0];
    else if(x_>x_bound[1])
        x_ = x_bound[1];
    next[0] = x_;
    next[1] = w_;
    return 0;
}