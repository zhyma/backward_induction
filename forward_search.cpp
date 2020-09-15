//#include "phy_model.h"
#include "forward_search.h"

//initial function
DPModel::DPModel(PHYModel * ptr_in, int sample_rate, bool write2file)
{
    ptr_model = ptr_in;
    N = ptr_model->N;
    gran = sample_rate;
    save_transition = write2file;

    // discretizing x, u, and w
    x_cnt = (int)(round((ptr_model->x_bound[1]-ptr_model->x_bound[0])*gran+1));
    x_list = new float[x_cnt];
    for(int i = 0;i < x_cnt; ++i)
        x_list[i] = ptr_model->x_bound[0] + 1.0/sample_rate * i;

    u_cnt = (int)(round((ptr_model->u_bound[1]-ptr_model->u_bound[0])*gran+1));
    u_list = new float[u_cnt];
    for(int i = 0;i < u_cnt; ++i)
        u_list[i] = ptr_model->u_bound[0] + 1.0/sample_rate *i;

    temp_search = new float[(N * x_cnt) * u_cnt];
    // initializing all counter as 0
    cnter_table = new int[(N * x_cnt) * u_cnt * (x_cnt + 1)] ();
    prob_table = new float[(N * x_cnt) * u_cnt * x_cnt];
    
    return;
}

// By given k, x, u, find the corresponding index (1D dynamic array for convenience)
// For control problem considering the horizon, <k,x> is the state. You may have the same x at different step k.
int DPModel::kxu2index(int k, int x, int u)
{
    int idx = k * (x_cnt * u_cnt);
    idx += x * u_cnt + u;
    return idx;
}

// By given <s/k*x, a/u>, find next state s_/(k+1)*x_
int DPModel::x2x_cnt(float x)
{
    int idx = 0;
    idx = round((x - ptr_model->x_bound[0])*gran);
    idx < 0 ? idx = 0 : idx;
    idx > x_cnt - 1 ? idx = x_cnt -1 : idx;
    return idx;
}

// Search forward one time. Once you search for multiple times, you can get a "grid" model for DP
// Return a matrix. The index is a tuple <x_k, a>, the value corresponding to the tuple is the x_{k+1}
// There are two types of searching:
// 1) from a given x_0, provide one input, get one output, move to the next time step and that the output as the new state
// 2) from all possible x_0, try all input, save all output/next state, then move on to the next step (applying this method here)
// x_k is a combination (tuple) of <k, x>
int DPModel::global_forward_once(float x0)
{
    float delta = 1.0/gran;
    int idx = 0;
    for (int k = 0; k < N; ++k)
    {
        for (int xk = 0;xk < x_cnt; ++xk)
        {
            for (int uk = 0; uk < u_cnt; ++uk)
            {
                float x = ptr_model->linear_model(k, x_list[xk], u_list[uk], 0.5);
                idx = kxu2index(k, xk, uk);
                temp_search[idx] = x;
            }
        }
    }
    return 0;
}

int DPModel::estimate_model(int iter)
{
    
    int idx = 0;
    int x_ = 0;
    for(int i = 0;i < iter;++i)
    {
        // search once
        global_forward_once(0);
        // count the state transition for each <x_k, u> pair
        for(int k = 0; k < N; k++)
        {
            for(int xk = 0; xk < x_cnt; xk++)
            {
                for(int uk = 0; uk < u_cnt; uk++)
                {
                    idx = kxu2index(k, xk, uk);
                    x_ = x2x_cnt(temp_search[idx]);
                    cnter_table[idx * (x_cnt+1) + (x_)] += 1;
                    cnter_table[idx * (x_cnt+1) + x_cnt] += 1;
                }
            }
        }
        if((i*100)/iter%5==0)
        {
            cout << (i*100)/iter << "%...";
        }
    }
    cout << endl;

    // now get the estimated model
    for(int k = 0;k < N;k++)
    {
        for(int xk =0; xk < x_cnt; ++xk)
        {
            for(int uk=0; uk < u_cnt; ++uk)
            {
                idx = kxu2index(k, xk, uk);
                float all_cnt = (float) cnter_table[idx*(x_cnt+1)+x_cnt];
                for(int x_= 0; x_ < x_cnt; ++x_)
                {
                    // the number of transit to a certain state divided by the number of all transition
                    float state_cnt = (float) cnter_table[idx*(x_cnt+1) + x_];
                    float prob = state_cnt/all_cnt;
                    prob_table[idx*x_cnt + x_] = prob;
                }
            }
        }
    }

    cout << "Model estimation done." << endl;

    // save to csv for test
    // Don't do that for large state-action space (will hang forever)
    if(save_transition)
    {
        ofstream out_prob;
        out_prob.open("../prob.csv", ios::out);
        for(int k = 0;k < N; ++k)
        {
            for(int xk = 0; xk < x_cnt; ++xk)
            {
                for(int uk = 0; uk < u_cnt; ++uk)
                {
                    idx = kxu2index(k, xk, uk);
                    for(int x_=0; x_ < x_cnt; ++x_)
                    {
                        out_prob << prob_table[idx*x_cnt + x_] << ',';
                    }
                    out_prob << endl;
                }
            }
        }
        out_prob.close();
        cout << "Save estimated model to file." << endl;
    }

    return 0;
}