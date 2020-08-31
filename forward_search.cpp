#include "model.h"
#include "forward_search.h"

//inital function
DPModel::DPModel(int time_step, float * state_con, float * action_con, int sample_rate)
{
    N = time_step;
    gran = sample_rate;
    
    // initial x upper and lower bounds.
    x_con[0] = state_con[0];
    x_con[1] = state_con[1];

    x_cnt = (int)(round((x_con[1]-x_con[0])*gran+1));
    x_list = new float[x_cnt];
    for(int i = 0;i < x_cnt; ++i)
        x_list[i] = x_con[0] + 1.0/sample_rate * i;

    // initial u upper and lower bounds.
    u_con[0] = action_con[0];
    u_con[1] = action_con[1];

    u_cnt = (int)(round((u_con[1]-u_con[0])*gran+1));
    u_list = new float[u_cnt];
    for(int i = 0;i < u_cnt; ++i)
        u_list[i] = u_con[0] + 1.0/sample_rate *i;

    temp_search = new float[(N * x_cnt) * u_cnt];
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
    idx = round((x - x_con[0])*gran);
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
int DPModel::forward_search_once(float x0)
{
    float delta = 1.0/gran;
    int idx = 0;
    for(int k = 0; k < N; ++k)
    {
        for(int xk = 0;xk < x_cnt; ++xk)
        {
            for(int uk = 0; uk < u_cnt; ++uk)
            {
                float x = linear_model(k, x_list[xk], u_list[uk]);
                idx = kxu2index(k, xk, uk);
                temp_search[idx] = x;
            }
        }
    }
    return 0;
}

int DPModel::estimate_model(int i)
{
    
    int idx = 0;
    int x_ = 0;
    for(;i>0;i--)
    {
        // search once
        forward_search_once(0);
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
    }

    // now ger the estimated model
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
                //outFile << all_cnt;
            }
        }
    }

    // save to csv for test
    if(true)
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
                        out_prob << prob_table[idx*x_cnt + x_] << ",";
                    }
                    out_prob << endl;
                }
            }
        }
        out_prob.close();
    }

    return 0;
}

// int main()
// {
//     DPModel model;
//     model.estimate_model(100);

//     cout << "done" << endl;

//     return 0;
// }