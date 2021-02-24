#include "cpu_solver.h"
#include <cstring>
#include <algorithm>

CPUSolver::CPUSolver(DPModel * ptr_in)
{
    model = ptr_in;

    N = model->N_pred;
    n_v = model->v.n;
    n_t = model->n_t; // 128

    n_x = n_t * n_v; // 128*32
    n_x_s = (n_t-model->max_last_step) * n_v; // 115*32
    n_w = n_t * 2; // 128*2
    n_w_s = (n_t-model->max_last_step) * 2; // 115*2
    n_u = model->u.n;

    n_p = model->n_p; // 28

    value = new float[(N+1)*n_x*n_w]{};
    // for (int i = 0; i < (N+1)*n_x*n_w; ++i)
    //     value[i] = 1e20;
    action = new int[N*n_x_s*n_w_s]{};
    // for (int i = 0; i < N*n_x_s*n_w_s; ++i)
    //     action[i] = -1;
    return;
}

CPUSolver::~CPUSolver()
{
    // if (r_cost)
    //     delete [] r_cost;
    // if (r_mask)
    //     delete [] r_mask;
    // if (t_cost)
    //     delete [] t_cost;
    // if (trans)
    //     delete [] trans;
    // if (prob)
    //     delete [] prob;
}

int CPUSolver::find_min(float *q, int cnt)
{
    // int index = -1;
    // float value = 1e20;
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
    return index;
}

float CPUSolver::calc_q(int k0, int k, int xk, int wk, int uk)
{
    int xk_ = 0;
    int wk_ = 0;
    float sum = 0;

    int idx = xk*n_u + uk;
    xk_ = trans[idx];

    for (int dwk = 0; dwk < n_p; ++dwk)
    {
        // p*V_{k+1}
        int p_idx = k*n_w_s*n_p + wk*n_p + dwk;
        float p = prob[p_idx];
        int v_idx = (k+1)*(n_x*n_w) + xk_*n_w + (wk+dwk);
        float v = value[v_idx];
        sum += p*v;
    }
    float l  = r_cost[xk*n_w_s*n_u + wk*n_u + uk];
    l += float(r_mask[xk*n_w_s*n_u + wk*n_u + uk] & 1<<(k0+k)) * 1e30;
    
    return l + sum;
}

int CPUSolver::estimate_one_step(int k0, int k)
{
    if (k==N)
    {
        // calculate the terminal cost at N=10
        // initial value for V_N is V_N(x)=J_f(x), final cost
        // final step, no simulation/data is needed

        for(int xk = 0; xk < n_x; ++xk)
        {
            for (int wk = 0; wk < n_w; ++wk)
            {
                int idx = k*n_x*n_w + xk*n_w + wk;
                value[idx] = t_cost[xk];
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
        for (int xk = 0; xk < n_x_s; ++xk)
        {
            for (int wk = 0; wk < n_w_s; ++wk)
            {
                // get a <x, w> pair first
                for (int uk = 0; uk < n_u; ++uk)
                {
                    // for each <x, u> (q in RL): l(x,u)+\sum P(z|x,u)V(z)
                    q[uk] = calc_q(k0, k, xk, wk, uk);
                }
                // v = min[l(x,u)+\sum P(z|x,u)V(z)]
                // find the minimium now.
                int idx_min = find_min(q, n_u);
                if (idx_min >= 0)
                {
                    value[k*n_x*n_w + xk*n_w + wk] = q[idx_min];
                    //action[k*n_x*n_w + xk*n_w + wk] = model->u_set.list[min.index];
                    action[k*n_x_s*n_w_s + xk*n_w_s + wk] = idx_min;
                }
            }
        }
    }
    else
    {
        std::cout << "Error! k="<< k <<" is out of the boundary!" << std::endl;
    }
    return 0;
}

int CPUSolver::solve(int k0, int dk0, int dck0)
{
    get_subset(k0, dk0, dck0);
    std::cout << "extract subset done." << std::endl;
    for (int k = N; k >= 0; k--)
        estimate_one_step(k0, k);

    return 0;
}

int CPUSolver::get_subset(int k0, int dk0, int dck0)
{
    r_cost = new float[n_x_s*n_w_s*n_u]{};
    r_mask = new unsigned long long int[n_x_s*n_w_s*n_u]{};
    t_cost = new float[n_x]{};
    trans = new int[n_x_s*n_u]{};
    prob = new float[N*n_w_s*n_p]{};

    // long long int idx = 0;
    int idx = 0;
    int solver_idx = 0;

    //slicing state transition (x,w,u)
    for (int dxk = 0; dxk < n_x_s; ++dxk)
    {
        for (int uk =0; uk < n_u; ++uk)
        {
            idx =  (dk0*n_v + dxk) * model->u.n + uk;
            solver_idx = dxk*n_u + uk;
            // //before shift:
            // trans[solver_idx] = model->s_trans_table[idx];
            // //relation:
            // //xk = dk*v.n + vk = (dk0+ddk)*n_v + vk ==>> 
            // //after shift
            trans[solver_idx] = model->s_trans_table[idx] - dk0*n_v;
        }
    }
    if(false)
    {
        std::string filename = "partial_trans_cpu";
        int dim[] = {1, n_x_s, n_u};
        mat_to_file(filename, sizeof(dim)/sizeof(dim[0]), dim, trans);
    }
    std::cout << "extract subset from state transition" << std::endl;

    //slicing transition probability (k,w,w')
    // k = k0+dk
    std::cout << "get transition probability starts here" << std::endl;
    
    for (int dk = 0; dk < N; ++dk)
    {
        for (int dwk = 0; dwk < n_w_s; ++dwk)
        {
            for (int dwk_ = 0; dwk_ < n_p; ++dwk_)
            {
                idx = (k0+dk) * model->w.n * model->n_p + (dck0*2+dwk) * model->n_p + dwk_;
                solver_idx = dk*n_w_s*n_p + dwk*n_p + dwk_;
                prob[solver_idx] = model->prob_table[idx];
            }
        }
    }
    if(false)
    {
        std::string filename = "partial_prob_cpu";
        int dim[] = {N, n_w_s, n_p};
        mat_to_file(filename, sizeof(dim)/sizeof(dim[0]), dim, prob);
    }
    std::cout << "extract subset from transition probability" << std::endl;


    //slicing running_cost (k,x,w,u)
    // k = k0+dk
    // xk = xk0 + dxk = dk0*n_v + dxk
    for (int dxk = 0; dxk < n_x_s; ++dxk)
    {
        // wk = 
        for (int dwk = 0; dwk < n_w_s; ++dwk)
        {
            for (int uk =0; uk < n_u; ++uk)
            {
                // xk dimension: (d0+ddk)*n_v
                // wk dimension: (dck0+ddck)*2
                int temp1 = (dk0 * model->v.n + dxk) * model->w.n * model->u.n;
                // uk dimension: uk
                int temp2 = (dck0*2 + dwk) * model->u.n;
                int temp3 = uk;
                idx =  temp1 + temp2 + temp3;
                if (idx < 0)
                {
                    std::cout << "In CPU solver, running cost index out of range! idx=" << idx << std::endl;
                    std::cout << "ERROR!" << std::endl;
                }
                solver_idx = dxk*n_w_s*n_u + dwk*n_u + uk;
                r_cost[solver_idx] = model->r_cost[idx];
                r_mask[solver_idx] = model->r_mask[idx];
            }
        }
    }
    std::cout << "extract subset from running cost" << std::endl;

    // generate terminal cost
    for (int dk = 0; dk < n_t; ++dk)
    {
        for (int vk = 0; vk < n_v; ++vk)
        {
            idx = (dk*n_v+vk);
            t_cost[idx] = model->terminal_cost(dk0, dk, vk);
        }
    }
    std::cout << "generate terminal cost" << std::endl;

    return 0;
}