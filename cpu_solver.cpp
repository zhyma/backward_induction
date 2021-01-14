#include "cpu_solver.h"

CPUSolver::CPUSolver(DPModel * ptr_in, std::atomic<int>* busy_p_mat)
{
    model = ptr_in;
    busy_mat_ptr = busy_p_mat;

    N = model->N;
    n_x = model->x.n;
    n_w = model->w.n;
    n_u = model->u.n;
    value = new float[(N+1)*n_x*n_w];
    action = new int[N*n_x*n_w];

    return;
}

int CPUSolver::find_min(float *q, int cnt, struct Min_index *min)
{
    int index = 0;
    float value = q[0];
    for(int i = 0;i < cnt; ++i)
    {
        if(q[i] < value)
        {
            index = i;
            value = q[i];
        }
    }
    min->index = index;
    min->value = value;
    return 0;
}

float CPUSolver::calc_q(int k, int xk, int wk, int uk)
{
    int  xk_ = 0;
    int  wk_ = 0;
    float sum = 0;

    int idx = xk*n_w*n_u + wk*n_u + uk;
    xk_ = model->s_trans_table[idx];

    for (int wk_ = 0; wk_ < n_w; ++wk_)
    {
        // p*V_{k+1}
        int p_idx = k*n_w*n_w + wk*n_w + wk_;
        float p = model->prob_table[*busy_mat_ptr][p_idx];
        int v_idx = (k+1)*(n_x*n_w) + xk_*n_w + wk_;
        float v = value[v_idx];
        sum += p*v;
    }
    float l = model->running_cost[k*n_x*n_w*n_u + xk*n_w*n_u + wk*n_u + uk];
    
    return l + sum;
}

int CPUSolver::estimate_one_step(int k)
{
    if (k==N)
    {
        // calculate the terminal cost at N=10
        // initial value for V_N is V_N(x)=J_f(x), final cost
        // J_f(x) = (1-x_N)^2
        // final step, no simulation/data is needed

        for(int xk = 0; xk < n_x; ++xk)
        {
            for (int wk = 0; wk < n_w; ++wk)
            {
                int idx = xk*n_w + wk;
                value[k*n_x*n_w + xk*n_w + wk] = model->t_cost[idx];
            }
        }
    }
    else if((k >= 0) and (k < N))
    {
        // calculate the running cost
        // searching backward, search for the transition probability one step before, then calculate the min
        // generate probability estimation for intermediate steps

        // a temporary buffer to save all the result of executing different u for a given xk, wk
        std::cout << "working on step " << k << std::endl;
        float *q = new float[n_u]{};
        for (int xk = 0; xk < n_x; ++xk)
        {
            for (int wk = 0; wk < n_w; ++wk)
            {
                // get a <x, w> pair first
                for (int uk = 0; uk < n_u; ++uk)
                {
                    // for each <x, u> (q in RL): l(x,u)+\sum P(z|x,u)V(z)
                    // l(x) = x^2+u^2
                    q[uk] = calc_q(k, xk, wk, uk);
                }
                // v = min[l(x,u)+\sum P(z|x,u)V(z)]
                // find the minimium now.
                Min_index min;
                find_min(q, n_u, &min);
                value[k*n_x*n_w + xk*n_w + wk] = min.value;
                //action[k*n_x*n_w + xk*n_w + wk] = model->u_set.list[min.index];
                action[k*n_x*n_w + xk*n_w + wk] = min.index;
            }
        }
    }
    else
    {
        std::cout << "Error! k="<< k <<" is out of the boundary!" << std::endl;
    }
    return 0;
}

int CPUSolver::solve()
{
    std::cout << "CPU using p_mat buffer #" << *busy_mat_ptr << std::endl;
    for (int k = N; k >= 0; k--)
        estimate_one_step(k);
    
    return 0;
}