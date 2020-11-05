#include "cpu_solver.h"

CPUSolver::CPUSolver(DPModel * ptr_in)
{
    model = ptr_in;
    return;
}

int CPUSolver::find_min(float *u, int cnt, struct Min_index *min)
{
    int index = 0;
    float value = u[0];
    for(int i = 0;i < cnt; ++i)
    {
        if(u[i] < value)
        {
            value = u[i];
            index = i;
        }
    }
    min->index = index;
    min->value = value;
    return 0;
}

// 2D matrix <x, u>
// find the index of corresponding <x, u> pair
// the full state contains time step k
int CPUSolver::xw_idx(int xk, int wk)
{
    int idx = xk * model->w_set.count + wk;
    return idx;
}

int CPUSolver::state_idx(int k, int xk, int wk)
{
    int idx = k * xw_cnt + xw_idx(xk, wk);
    return idx;
    return 0;
}

float CPUSolver::calc_q(int k, int xk, int wk, int uk)
{
    float x_ = 0;
    float w_ = 0;
    int  xk_ = 0;
    int  wk_ = 0;
    int  idx = 0;
    float next[2] = {};
    float sum = 0;


    ptr_model->linear_model(k, x_set.list[xk], w_set.list[wk], u_set.list[uk], next);
    x_ = next[0];
    w_ = next[1];
    xk_ = val_to_idx(x_, &x_set);
    wk_ = val_to_idx(w_, &w_set);

    float *prob_table = new float[w_set.count]{};
    get_distribution(wk, prob_table);

    for (int i = 0; i < w_set.count; ++i)
    {
        // p*V_{k+1}
        sum += prob_table[i] * value_table[state_idx(k+1, xk_, i)];
    }
    
    delete [] prob_table;
    return sum;
}

float CPUSolver::estimate_one_step(int k)
{
    clock_t start,end;
    if (k==N)
    {
        // calculate the termianl cost at N=10
        // initial value for V_N is V_N(x)=J_f(x), final cost
        // J_f(x) = (1-x_N)^2
        // final step, no simulation/data is needed
        start = clock();

        for(int xk = 0; xk < x_set.count; ++xk)
        {
            for (int wk = 0; wk < w_set.count; ++wk)
            {
                float v = pow(1-x_set.list[xk],2);
                value_table[state_idx(N, xk, wk)] = v;
            }
        }

        end = clock();
    }
    else if((k >= 0) and (k < N))
    {
        // calculate the running cost
        // searching backward, search for the transition probability one step before, then calculate the min
        // generate probability estimation for intermediate steps

        start = clock();
        // a temporary buffer to save all the result of executing different u for a given xk, wk
        float *u_z_temp = new float[u_set.count]{};
        for (int xk = 0; xk < x_set.count; ++xk)
        {
            for (int wk = 0; wk < w_set.count; ++wk)
            {
                // get a <x, w> pair first
                for (int uk = 0; uk < u_set.count; ++uk)
                {
                    // for each <x, u> (q in RL): l(x,u)+\sum P(z|x,u)V(z)
                    // l(x) = x^2+u^2
                    float x = x_set.list[xk];
                    float u = u_set.list[uk];
                    float l = x*x + u*u;
                    float sum = 0;
                    //get the transition probability here
                    if (prob_type == MONTECARLO)
                    {
                        prob_table = new float[xw_cnt]();

                        // from step k to k+1
                        cout << "searching for step " << k << endl;
                        // TODO: NEED TO RE-WORK ON THIS
                        mc_one_stateaction(k, xk, wk, uk);
                        for (int xk_ = 0; xk_ < x_set.count; ++xk_)
                        {
                            for (int wk_ = 0; wk_ < w_set.count; ++wk_)
                            {
                                //<k, x_k> --u_k--> <k+1,x_k+1>
                                int idx = xw_idx(xk_, wk_);
                                float p_z = prob_table[idx];
                                float v_z = value_table[state_idx(k+1, xk_, wk_)];
                                sum += p_z*v_z;
                            }
                        }
                        delete [] prob_table;
                    }
                    else
                    {
                        //cout << "TODO: algebraic way" << endl;
                        sum = calc_q(k, xk, wk, uk);
                    }
                    // z, or x_/x'
                    
                    u_z_temp[uk] = l+sum;

                }
                // v = min[l(x,u)+\sum P(z|x,u)V(z)]
                // find the minimium now.
                Min_index min;
                find_min(u_z_temp, u_set.count, &min);
                value_table[state_idx(k, xk, wk)] = min.value;
                action_table[state_idx(k, xk, wk)] = u_set.list[min.index];
            }
        }
        end = clock();
    }
    else
    {
        cout << "Error! k="<< k <<" is out of the boundary!" << endl;
    }
    
    // return: time
    return (float) (end-start)/CLOCKS_PER_SEC;
}