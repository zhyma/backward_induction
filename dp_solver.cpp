#include "model.h"
#include "forward_search.h"
#include <iostream>
#include <cmath>
#include <fstream>
using namespace std;

struct Min_index
{
    int index;
    float value;
};

int find_min(float *u, int cnt, struct Min_index *min)
{
    int index = 0;
    float value = u[0];
    for(int i = 0;i < cnt; ++i)
    {
        if(u[i] > value)
        {
            value = u[i];
            index = i;
        }
        // cout << u[i] << ",";
    }
    // cout << endl;
    min->index = index;
    min->value = value;
    return 0;
}

// all possible x and u
int solver(DPModel &model)
{
    float *value_table = new float[(model.N+1)*model.x_cnt]{};
    int *action_table = new int[model.N*model.x_cnt]{};
    // calculate the termianl cost at N=10
    // initial value for V_N is V_N(x)=J_f(x), final cost
    for(int x = 0; x < model.x_cnt; ++x)
    {
        float v = pow(1-model.x_list[x],2);
        value_table[(model.N-1)*model.x_cnt+x] = v;
        cout << v << ",";
    }
    cout << endl;
    // calculate the running cost
    // searching backward
    // a temporary buffer to save all the result of executing different u for a given xk
    float *u_z_temp = new float[model.u_cnt];
    for(int k = model.N-1; k >= 0; k--)
    {
        for(int xk = 0; xk < model.x_cnt; ++xk)
        {
            for(int uk = 0; uk < model.u_cnt; ++uk)
            {
                // for each <x, u> (q in RL): l(x,u)+\sum P(z|x,u)V(z)
                // l(x) = x^2+u^2
                float x = model.x_list[xk];
                float u = model.u_list[uk];
                float l = x*x + u*u;
                float sum = 0;
                // z, or x_/x'
                for(int x_ = 0; x_ < model.x_cnt; ++x_)
                {
                    //<k, x_k> --u_k--> <k+1,x_k+1>
                    int idx = model.kxu2index(k, xk, uk);
                    float p_z = model.prob_table[idx*model.x_cnt + x_];
                    float v_z = model.x_list[x_];
                    sum += p_z*v_z;
                }
                u_z_temp[uk] = l+sum;
            }
            // v = min[l(x,u)+\sum P(z|x,u)V(z)]
            // find the minimium now.
            Min_index min;
            find_min(u_z_temp, model.u_cnt, &min);
            value_table[k*model.x_cnt + xk] = min.value;
            action_table[k*model.x_cnt + xk] = min.index;
        }
    }
    if(true)
    {
        ofstream out_value;
        ofstream out_action;
        out_value.open("../value.csv", ios::out);
        out_action.open("../action.csv", ios::out);
        for(int k = 0; k < model.N; k++)
        {
            for(int xk = 0; xk < model.x_cnt; xk++)
            {
                out_value << value_table[k*model.x_cnt + xk] << ",";
                out_action << action_table[k*model.x_cnt + xk] << ",";
            }
            out_value << endl;
            out_action << endl;
        }
    }
    
    return 0;
}

int main()
{
    int N = 10;
    float x_con[2] = {-2.0, 2.0};
    float u_con[2] = {0.2, 1.6};
    int gran = 10;
    DPModel model(N, x_con, u_con, gran);
    model.estimate_model(100);
    solver(model);
    return 0;
}