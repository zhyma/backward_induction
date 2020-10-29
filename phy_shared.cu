#include "phy_shared.h"

__host__ __device__ float shared_linear_model(float x, float w, float u)
{
    //x\in[-2, 2]
    if (x < x_lower)
        x = x_lower;
    else if (x > x_upper)
        x = x_upper;
    //u\in[0.2, 1.6]
    if (u < u_lower)
        u = u_lower;
    else if (u > u_upper)
        u = u_upper;

    float x_ = 0.9*x + u - w;
    if(x_ < x_lower)
        x_ = x_lower;
    else if(x_ > x_upper)
        x_ = x_upper;

    return x_;
}

float linear_model_cpu(float x, float w, float u)
{
    return shared_linear_model(x, w, u);
}