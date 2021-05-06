#include "cpu_solver.h"
#include "utility.h"
#include <cstring>
#include <algorithm>

CPUSolver::CPUSolver(DPModel * ptr_in, bool save)
{
    model = ptr_in;

    N = model->N_pred;
    n_v = model->v.n;
    n_d = model->n_d; // 128
    n_dc = model->n_dc; // 185

    n_x = n_d * n_v; // 128*32
    n_x_s = (n_d-model->max_last_step) * n_v; // 115*32
    n_w = n_dc * 2; // 185*2
    n_w_s = (n_dc-model->max_last_step) * 2; // (185-13)*2
    n_u = model->u.n;

    n_p = model->n_p; // 28

    value_buffer = new float[2*n_x*n_w]{};
    // for (int i = 0; i < (N+1)*n_x*n_w; ++i)
    //     value[i] = 1e30;
    action = new int[N*n_x_s*n_w_s]{};
    // for (int i = 0; i < N*n_x_s*n_w_s; ++i)
    //     action[i] = -1;

    // if (save==true)
    // {
    //     save_v = true;
    //     value = new float[(N+1)*n_x*n_w];
    // }
    // else
    //     save_v = false;
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
    int idx = cnt-1;
    float val = q[cnt-1];
    for(int i = cnt-2;i >= 0; --i)
    {
        if(q[i] < val)
        {
            idx = i;
            val = q[i];
        }
    }
    return idx;
}

long CPUSolver::calc_q(int k0, int k, long xk, long wk, int uk)
{
    long xk_ = 0;
    long wk_ = 0;
    float sum = 0;

    long idx = xk*n_u + uk;
    xk_ = trans[idx];

    for (int dwk = 0; dwk < n_p; ++dwk)
    {
        // p*V_{k+1}
        int p_idx = k*n_w_s*n_p + wk*n_p + dwk;
        // apply the offset
        int v_idx = ((k+1)%2)*(n_x*n_w) + xk_*n_w + (wk+dwk-1);
        sum += value_buffer[v_idx]*prob[p_idx];

        // if (k==1 && prob[p_idx]>0 && xk==148)
        // {
        //     std::cout << "xk': " << xk_;
        //     std::cout << ", wk: " << wk;
        //     std::cout << ", wk': " << wk+dwk-1;
        //     std::cout << ", uk: " << uk << std::endl;
        //     // std::cout << "p_idx: " << p_idx;
        //     // std::cout << ", v_idx: " << v_idx << std::endl;
        //     std::cout << "value: " << value_buffer[v_idx];
        //     std::cout << ", v+1: " << sum << std::endl;
        //     std::cout << "====" << std::endl;
        // }
    }
    float l = r_cost[xk*n_w_s*n_u + wk*n_u + uk];

    if ( (r_mask[xk*n_w_s*n_u + wk*n_u + uk] & (1<<(k0+k))) > 0)
        l = PENALTY;

    // if (k==0 && xk==0 && wk==77)
    // {
    //     std::cout << "xk': " << xk_;
    //     // std::cout << ", wk': " << wk+dwk-1;
    //     std::cout << ", uk: " << uk << std::endl;
    //     std::cout << "r: " << l;
    //     std::cout << ", v+1: " << sum;
    //     std::cout << ", total: " << l+sum << std::endl;
    //     std::cout << "====" << std::endl;
    // }

    return l + sum;
}

int CPUSolver::estimate_one_step(int k0, int k)
{
    // std::ofstream out_file;
    // out_file.open("output/one_step_test.csv", std::ios_base::app);
    // out_file << std::setiosflags(std::ios::fixed) << std::setprecision(2);

    if (k==N)
    {
        // calculate the terminal cost at N=10
        // initial value for V_N is V_N(x)=J_f(x), final cost
        // final step, no simulation/data is needed

        for(long xk = 0; xk < n_x; ++xk)
        {
            for (long wk = 0; wk < n_w; ++wk)
            {
                value_buffer[(k%2)*n_x*n_w + xk*n_w + wk] = t_cost[xk*n_w+wk];
            }
        }
        if (save_v==true)
        {
            for(long i = 0; i < n_x*n_w; ++i)
                value[k*n_x*n_w + i] = value_buffer[(k%2)*n_x*n_w + i];
        }
    }
    else if((k >= 0) and (k < N))
    {
        // out_file << "k=" << k << std::endl;

        // calculate the running cost
        // searching backward, search for the transition probability one step before, then calculate the min
        // generate probability estimation for intermediate steps

        // a temporary buffer to save all the result of executing different u for a given xk, wk
        // std::cout << "working on step " << k << std::endl;
        float *q = new float [n_u]{};
        for (long xk = 0; xk < n_x_s; ++xk)
        {
            for (long wk = 0; wk < n_w_s; ++wk)
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

                // if (k==0 && xk==0 && wk==77)
                // {
                //     std::cout << "====" << std::endl;
                //     std::cout << "xk: " << xk;
                //     std::cout << ", uk: " << idx_min << std::endl;
                //     std::cout << "====" << std::endl;
                // }

                value_buffer[(k%2)*n_x*n_w + xk*n_w + wk] = q[idx_min];
                action[k*n_x_s*n_w_s + xk*n_w_s + wk] = idx_min;

            }
        }

        // if (save_v==true)
        // {
        //     for (long xk = 0; xk < n_x_s; ++xk)
        //     {
        //         for (long wk = 0; wk < n_w_s; ++wk)
        //             value[k*n_x*n_w + xk*n_w + wk] = value_buffer[(k%2)*n_x*n_w + xk*n_w + wk];
        //     }
        // }
    }
    else
    {
        std::cout << "Error! k="<< k <<" is out of the boundary!" << std::endl;
    }
    // out_file.close();
    return 0;
}

int CPUSolver::solve(int k0, float d0, float v0, float dc0, int intention)
{
    // std::cout << "solver started" << std::endl;
    int dk0 = model->get_dist_idx(d0);
    int vk0 = model->get_velc_idx(v0);
    int dck0 = model->get_dist_idx(dc0);
    // std::cout << "dk0 is: " << dk0 << std::endl;
    // std::cout << "dck0 is: " << dck0 << std::endl;
    get_subset(k0, dk0, dck0);
    // std::cout << "extract subset done." << std::endl;
    for (int k = N; k >= 0; k--)
    // for (int k = N; k >= 8; k--)
    {
        // std::cout << "solve step " << k << std::endl;
        estimate_one_step(k0, k);
    }
    
    long idx = (dk0*model->v.n + vk0)*n_w_s + (0+intention);
    int ak = action[idx];

    // std:: cout << "solver output: " << model->a.list[ak] << std::endl;
    // apply action filter here
    float a = model->action_filter(v0, ak);

    return a;
}

int CPUSolver::get_subset(int k0, int dk0, int dck0)
{
    r_cost = new long [n_x_s*n_w_s*n_u]{};
    r_mask = new long [n_x_s*n_w_s*n_u]{};
    t_cost = new long [n_x*n_w]{};
    trans = new long[n_x_s*n_u]{};
    prob = new float[N*n_w_s*n_p]{};

    // for debug only
    xk0_debug = dk0*n_v;
    wk0_debug = dck0*2;

    // long long int idx = 0;
    long idx = 0;
    long solver_idx = 0;

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
    if (debug)
    {
        std::string filename = "cpu_tran_part";
        long dim[] = {1, n_x_s, n_u};
        mat_to_file(filename, sizeof(dim)/sizeof(dim[0]), dim, trans);
    }
    // std::cout << "extract subset from state transition" << std::endl;

    // slicing transition probability (k,w,w')
    // k = k0+dk
    // std::cout << "get transition probability starts here" << std::endl;
    
    for (int k = 0; k < N; ++k)
    {
        for (long dwk = 0; dwk < n_w_s; ++dwk)
        {
            for (long dwk_ = 0; dwk_ < n_p; ++dwk_)
            {
                idx = (k0+k) * model->w.n * model->n_p + (dck0*2+dwk) * model->n_p + dwk_;
                solver_idx = k*n_w_s*n_p + dwk*n_p + dwk_;
                prob[solver_idx] = model->prob_table[idx];
            }
        }
    }
    if (debug)
    {
        std::string filename = "cpu_prob_part";
        long dim[] = {N, n_w_s, n_p};
        mat_to_file(filename, sizeof(dim)/sizeof(dim[0]), dim, prob);
    }
    // std::cout << "extract subset from transition probability" << std::endl;


    // slicing running_cost (k,x,w,u)
    // k = k0+dk
    // xk = xk0 + dxk = dk0*n_v + dxk
    for (long dxk = 0; dxk < n_x_s; ++dxk)
    {
        // wk = 
        for (long dwk = 0; dwk < n_w_s; ++dwk)
        {
            for (int uk =0; uk < n_u; ++uk)
            {
                // xk dimension: (d0+ddk)*n_v
                // wk dimension: (dck0+ddck)*2
                long temp1 = (dk0 * model->v.n + dxk) * model->w.n * model->u.n;
                // uk dimension: uk
                long temp2 = (dck0*2 + dwk) * model->u.n;
                long temp3 = uk;
                idx =  temp1 + temp2 + temp3;

                solver_idx = dxk*n_w_s*n_u + dwk*n_u + uk;
                r_cost[solver_idx] = model->r_cost[idx];
                r_mask[solver_idx] = model->r_mask[idx];
            }
        }
    }
    // std::cout << "extract subset from running cost" << std::endl;

    // generate terminal cost
    for (long xk = 0; xk < n_x; ++xk)
    {
        for (long wk = 0; wk < n_w; ++wk)
        {
            idx = (xk*n_w + wk);
            t_cost[idx] = model->terminal_cost(xk, wk);
        }
    }

    if (debug)
    {
        std::string filename = "t_cost_dv";
        long dim[] = {1, n_x, n_w};
        mat_to_file(filename, sizeof(dim)/sizeof(dim[0]), dim, t_cost);
    }
    // std::cout << "generate terminal cost" << std::endl;

    return 0;
}