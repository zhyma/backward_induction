#include "forward_search.h"
#include "model.h"

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
    for(int k = 0; k < N; k++)
    {
        int x_cnter = 0;
        for(float xk = x_con[0]; xk < x_con[1] + delta; xk += delta)
        {
            int u_cnter = 0;
            for(float uk=u_con[0]; uk < u_con[1] + delta; uk += delta)
            {
                float x = linear_model(k, xk, uk);
                idx = kxu2index(k, x_cnter, u_cnter);
                temp_search[idx] = x;
                u_cnter += 1;
            }
            x_cnter += 1;
        }
    }
    return 0;
}

int DPModel::estimate_model(int i)
{
    // save to csv for test
    ofstream outFile;
    outFile.open("../prob.csv", ios::out);
    int idx = 0;
    int x_ = 0;
    for(;i>0;i--)
    {
        // search once
        forward_search_once(0);
        // count the state transition for each <x_k, u> pair
        for(int k = 0; k < N; k++)
        {
            for(int x = 0; x < x_cnt; x++)
            {
                for(int u = 0; u < u_cnt; u++)
                {
                    idx = kxu2index(k, x, u);
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
        for(int x =0; x< x_cnt;x++)
        {
            for(int u=0;u<u_cnt;u++)
            {
                idx = kxu2index(k,x,u);
                float all_cnt = (float) cnter_table[idx*(x_cnt+1)+x_cnt];
                for(int x_= 0; x_ < x_cnt; x_++)
                {
                    // the number of transit to a certain state divided by the number of all transition
                    float state_cnt = (float) cnter_table[idx*(x_cnt+1) + x_];
                    float prob = state_cnt/all_cnt;
                    outFile << prob << ',';
                }
                //outFile << all_cnt;
                outFile << endl;
            }
        }
    }

    outFile.close();
    return 0;
}

// int main()
// {
//     DPModel model;
//     model.estimate_model(100);

//     cout << "done" << endl;

//     return 0;
// }