#include "phy_model.h"

float PHYModel::linear_model(float x, float w, float u)
{
    //x\in[-2, 2]
    x = (x<x_bound[0])?x_bound[0]:x;
    x = (x>x_bound[1])?x_bound[1]:x;

    w = (w<w_bound[0])?w_bound[0]:w;
    w = (w>w_bound[1])?w_bound[1]:w;

    u = (u<u_bound[0])?u_bound[0]:u;
    u = (u>u_bound[1])?u_bound[1]:u;

    float x_ = 0.9*x + u - w;
    x_ = (x_<x_bound[0])?x_bound[0]:x_;
    x_ = (x_>x_bound[1])?x_bound[1]:x_;

    return x_;
}