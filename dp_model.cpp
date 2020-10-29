#include "phy_constraint.h"
#include "dp_model.h"

//initial function
DPModel::DPModel()
{
    // discretizing x, u, and w
    x_set.bound[0] = x_lower;
    x_set.bound[1] = x_upper;
    discretize(&x_set);

    w_set.bound[0] = w_lower;
    w_set.bound[1] = w_upper;
    discretize(&w_set);

    u_set.bound[0] = u_lower;
    u_set.bound[1] = u_upper;
    discretize(&u_set);

    xw_cnt = x_set.count * w_set.count;
    states_cnt = xw_cnt * N;
    
    std::cout << "total states: " << states_cnt << std::endl;

    value_table = new float[(N+1)*xw_cnt]();
    action_table = new int[states_cnt]();

    // TODO: create your distribution from w -> w_ here!
    //create_distribution();
    
    return;
}

int DPModel::discretize(Set *in)
{
    std::cout << "get variable" << std::endl;
    std::cout << "lower bound " << in->bound[0] << std::endl;
    std::cout << "upper bound " << in->bound[1] << std::endl;
    in->count = (int)(round((in->bound[1]-in->bound[0])*grain+1));
    in->list = new float[in->count];
    for (int i = 0;i < in->count; ++i)
        in->list[i] = in->bound[0] + 1.0/grain * i;

    std::cout << "number: " << in->count << std::endl;  
    std::cout << "list: ";
    for(int i = 0;i < in->count; ++i)
        std::cout << in->list[i] << ", ";
    std::cout << std::endl;

    return 0;
}


// 2D matrix <x, u>
// find the index of corresponding <x, u> pair
// the full state contains time step k
int DPModel::xw_idx(int xk, int wk)
{
    int idx = xk * w_set.count + wk;
    return idx;
}

int DPModel::state_idx(int k, int xk, int wk)
{
    int idx = k * xw_cnt + xw_idx(xk, wk);
    return idx;
    return 0;
}

// By given a value x, find the index of 
int DPModel::val_to_idx(float val, struct Set *ref)
{
    int idx = 0;
    idx = round((val - ref->bound[0])*grain);
    // make sure it will not be out of boundary because of float accuracy
    idx < ref->bound[0] ? idx = 0 : idx;
    idx > ref->count - 1 ? idx = ref->count -1 : idx;
    return idx;
}