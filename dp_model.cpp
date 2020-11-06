#include "dp_model.h"


//initial function
DPModel::DPModel(PHYModel * ptr_in, int grain_in)
{
    ptr_model = ptr_in;
    grain = grain_in;

    N = ptr_model->N;
    // discretizing x, u, and w
    x_set.bound[0] = ptr_model->x_bound[0];
    x_set.bound[1] = ptr_model->x_bound[1];
    discretize(&x_set);

    w_set.bound[0] = ptr_model->w_bound[0];
    w_set.bound[1] = ptr_model->w_bound[1];
    discretize(&w_set);

    u_set.bound[0] = ptr_model->u_bound[0];
    u_set.bound[1] = ptr_model->u_bound[1];
    discretize(&u_set);

    xw_cnt = x_set.count * w_set.count;
    states_cnt = xw_cnt * N;
    
    std::cout << "total states: " << states_cnt << std::endl;

    // value_table = new float[(N+1)*xw_cnt]();
    // action_table = new int[states_cnt]();

    // TODO: create your distribution from w -> w_ here!
    //create_distribution();

    // create <x,w> -u-> x' table here
    state_trans();

    // create transition probability matrix
    Prob_gen prob((int)10e5, w_set.count, 2.0/6.0, w_set.bound);
    prob_table = new float[w_set.count*w_set.count]{};
    memcpy(prob_table, prob.prob_mat, w_set.count*w_set.count*sizeof(float));
    
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
    // std::cout << "list: ";
    // for(int i = 0;i < in->count; ++i)
    //     std::cout << in->list[i] << ", ";
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

int DPModel::state_trans()
{
    s_trans_table = new int[x_set.count * w_set.count * u_set.count]{};
    for (int xk = 0; xk < x_set.count; ++xk)
    {
        for (int wk = 0; wk < w_set.count; ++wk)
        {
            for (int uk = 0; uk < u_set.count; ++uk)
            {
                float x = x_set.list[xk];
                float w = w_set.list[wk];
                float u = u_set.list[uk];

                float x_ = ptr_model->linear_model(x,w,u);
                int xk_ = val_to_idx(x_, &x_set);
                // std::cout << "x=" << x << ", w=" << w << ", u=" << u << ", x_=" << x_set.list[xk_] << " (" << xk_ << ")" << std::endl; 
                int idx = xk*(w_set.count*u_set.count) + wk*u_set.count + uk;
                s_trans_table[idx] = xk_;
            }
        }
    }
    return 0;
}