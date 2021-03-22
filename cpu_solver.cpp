#include "cpu_solver.h"
#include "utility.h"
#include <cstring>
#include <algorithm>

CPUSolver::CPUSolver(DPModel * ptr_in)
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

    value = new long[(N+1)*n_x*n_w]{};
    // for (int i = 0; i < (N+1)*n_x*n_w; ++i)
    //     value[i] = 1e30;
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

int CPUSolver::find_min(long *q, int cnt)
{
    // int idx = -1;
    // float val = 1e20;

    // int idx = 0;
    // float val = q[0];
    // for(int i = 0;i < cnt; ++i)
    // {
    //     if(q[i] < val)
    //     {
    //         idx = i;
    //         val = q[i];
    //     }
    // }
    // return idx;

    int idx = cnt-1;
    long val = q[cnt-1];
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

long CPUSolver::calc_q(int k0, int k, int xk, int wk, int uk)
{
    int xk_ = 0;
    int wk_ = 0;
    long sum = 0;

    int idx = xk*n_u + uk;
    xk_ = trans[idx];

    for (int dwk = 0; dwk < n_p; ++dwk)
    {
        // p*V_{k+1}
        int p_idx = k*n_w_s*n_p + wk*n_p + dwk;
        double p = prob[p_idx];
        int v_idx = (k+1)*(n_x*n_w) + xk_*n_w + (wk+dwk);
        long v = value[v_idx];
        double temp = double(v)*p;
        sum += long (temp);

        // if (k0 == 0 && k==1 && xk == 39 && p>0 && uk==28)
        // if ((k0 == 0 && k==3 && xk == 303 && p>0 && uk==28) || \
        //     (k0 == 0 && k==2 && xk == 139 && p>0 && uk==28) || \
        //     (k0 == 0 && k==1 && xk == 303 && p>0 && uk==29) || \
        //     (k0 == 0 && k==0 && xk == 303 && p>0 && uk==30))
        // if (k0 == 0 && k==7 && xk == 68 && p>0 && uk==24)
        // {
        //     std::cout << "u=" << uk << " (" << model->a.list[uk] << ")";
        //     std::cout << "xk_=" << xk_ << " (" << model->d.list[xk_/n_v] << "," << model->v.list[xk_%n_v] << ")";
        //     std::cout << ", wk=" << wk;
        //     std::cout << ", dwk=" << dwk;
        //     std::cout << ", p=" << p;
        //     std::cout << ", v=" << v << std::endl;
        // }


        // // to test transition probability (use deterministic data)
        // if (uk == 4 && xk == 50 && p > 0)
        // {
        //     std::cout << "at k: " << k << " detected trans: " << wk0_debug + wk << " to " << wk0_debug + wk + dwk <<std::endl;
        // }
        // if (p > 0 && ((k == 8 && xk == 143)||(k == 9 && xk == 192)))
        // {
        //     // std::cout << "at k: " << k << " detected trans: " << wk0_debug + wk << " to " << wk0_debug + wk + dwk <<std::endl;
        //     // std::cout << "at k: " << k << " detected dc': " << model->d.list[(wk0_debug+wk+dwk)/2] << ", i: " << (wk+dwk)%2;
        //     std::cout << "at k: " << k;
        //     std::cout << "xk=" << xk0_debug + xk;
        //     std::cout << ", wk=" << wk0_debug + wk;
        //     std::cout << ", uk=" << uk;
        //     std::cout << ", xk_=" << xk0_debug + xk_;
        //     std::cout << ", wk_=" << wk0_debug+wk+dwk;
        //     std::cout << ", p=" << p << ", v=" << v << "," << std::endl;
        //     // std::cout << ", d'=" << model->d.list[xk_/n_v];
        //     // std::cout << ", v'=" << model->v.list[xk_%n_v] << std::endl;
        // }
    }
    long l  = r_cost[xk*n_w_s*n_u + wk*n_u + uk];

    // if (k0 == 0 && k==0 && xk == 0 && uk==24)
    // // // // if ((k0 == 0 && k==3 && xk == 303 && wk==39 && uk==28) || \
    // // // //     (k0 == 0 && k==2 && xk == 139 && wk==33 && uk==28) || \
    // // // //     (k0 == 0 && k==1 && xk == 38 && wk==25 && uk==29) || \
    // // // //     (k0 == 0 && k==0 && xk == 0 && wk==19 && uk==30))
    // {
    //     std::cout << "xk'=" << xk_ << ", ";
    //     std::cout << ", d'=" << model->d.list[xk_/n_v];
    //     std::cout << ", v'=" << model->v.list[xk_%n_v] << std::endl;
    //     std::cout << "u=" << uk;
    //     std::cout << ", l=" << l;
    //     std::cout << ", sum=" << sum << std::endl;
    //     std::cout << "mask: " << r_mask[xk*n_w_s*n_u + wk*n_u + uk] << std:: endl;
    //     std::cout << ", cost to go=" << l+sum << std::endl << std::endl;
    // }

    // if (k == 6 && xk == 32 && wk == 185 && uk==28)
    // {
    //     std::cout << "k=" << k;
    //     std::cout << ", d=" << model->d.list[xk/n_v];
    //     std::cout << ", v=" << model->v.list[xk%n_v];
    //     std::cout << ", a=" << model->a.list[uk];
    //     std::cout << ", uk=" << uk; 
    //     std::cout << ", d'=" << model->d.list[xk_/n_v];
    //     std::cout << ", v'=" << model->v.list[xk_%n_v] << std::endl;
    //     std::cout << "running cost is: " << l;
    //     std::cout << ", rmask is: " << r_mask[xk*n_w_s*n_u + wk*n_u + uk];
    //     std::cout << ", sum is: " << sum;
    //     std::cout << ", q is: " << l+sum << std::endl << std::endl;
    // }

    if ( (r_mask[xk*n_w_s*n_u + wk*n_u + uk] & (1<<(k0+k))) > 0)
        l = 1e15;

    // // // to test state transition
    // if (k0 == 0 && k>7 && xk == 143 && wk==191 && uk ==3)
    // {
    //     std::cout << "d=" << model->d.list[xk/n_v];
    //     std::cout << ", v=" << model->v.list[xk%n_v];
    //     std::cout << ", a=" << model->a.list[uk];
    //     std::cout << ", d'=" << model->d.list[xk_/n_v];
    //     std::cout << ", v'=" << model->v.list[xk_%n_v] << std::endl;
    // }

    // to test running_cost
    // if (k0 == 0 && k>8 && xk == 143 && wk==191)
    // if (k == 6 && xk == 32 && wk == 185 && uk==28)
    // {
    //     std::cout << "k=" << k;
    //     std::cout << ", d=" << model->d.list[xk/n_v];
    //     std::cout << ", v=" << model->v.list[xk%n_v];
    //     std::cout << ", a=" << model->a.list[uk];
    //     std::cout << ", uk=" << uk; 
    //     std::cout << ", d'=" << model->d.list[xk_/n_v];
    //     std::cout << ", v'=" << model->v.list[xk_%n_v] << std::endl;
    //     std::cout << "running cost is: " << l;
    //     std::cout << ", sum is: " << sum;
    //     std::cout << ", q is: " << l+sum << std::endl << std::endl;
    // }

    // if (uk == 20 && xk == 3200 && k0 == 0 && wk == 2)
    // {
    //     std::cout << "r_mask: " << l << std::endl;
    // }

    // // // examine the running cost
    // if (k0 == 0 && k==3 && xk==139 && wk==39)
    // {
    //     std::cout << "v=" << model->v.list[xk%n_v];
    //     std::cout << ", a=" << model->a.list[uk];
    //     std::cout << ", sum=" << sum;
    //     std::cout << ", running_cost=" << l;
    //     std::cout << ", q=" << l+sum << std::endl;
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
        // out_file << "k=" << k << std::endl;

        // calculate the running cost
        // searching backward, search for the transition probability one step before, then calculate the min
        // generate probability estimation for intermediate steps

        // a temporary buffer to save all the result of executing different u for a given xk, wk
        // std::cout << "working on step " << k << std::endl;
        long *q = new long [n_u]{};
        for (int xk = 0; xk < n_x_s; ++xk)
        {
            for (int wk = 0; wk < n_w_s; ++wk)
            {
                // get a <x, w> pair first
                for (int uk = 0; uk < n_u; ++uk)
                {
                    // for each <x, u> (q in RL): l(x,u)+\sum P(z|x,u)V(z)
                    q[uk] = calc_q(k0, k, xk, wk, uk);
                    // if (xk==192 && wk0_debug + wk==247)
                    //     std::cout << "q is: " << q[uk] << std::endl<< std::endl;
                }
                // v = min[l(x,u)+\sum P(z|x,u)V(z)]
                // find the minimium now.
                int idx_min = find_min(q, n_u);
                if (idx_min >= 0)
                {
                    value[k*n_x*n_w + xk*n_w + wk] = q[idx_min];
                    action[k*n_x_s*n_w_s + xk*n_w_s + wk] = idx_min;
                }
                else 
                    std::cout << "idx_min error?";
                
                // if (xk==192 && wk0_debug + wk==247)
                // {
                //     std::cout << "value is: " << q[idx_min] << ", action is " << idx_min << std::endl;
                // }
                // if (wk == 1)
                // {
                //     out_file << "d=" << model->d.list[xk/n_v];
                //     out_file << " v=" << model->v.list[xk%n_v] << std::endl;
                //     out_file << " q=";
                //     for (int i =0; i < n_u; ++i)
                //         out_file << q[i] << ", ";
                //     out_file << std::endl << " min idx=" << idx_min << std::endl;
                // }
            }
        }
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
    std::cout << "dk0 is: " << dk0 << std::endl;
    std::cout << "dck0 is: " << dck0 << std::endl;
    get_subset(k0, dk0, dck0);
    // std::cout << "extract subset done." << std::endl;
    for (int k = N; k >= 0; k--)
    // for (int k = N; k >= 8; k--)
    {
        std::cout << "solve step " << k << std::endl;
        estimate_one_step(k0, k);
    }

    // std::cout << "xk'=" << 563 << ", ";
    // std::cout << ", d'=" << model->d.list[563/n_v];
    // std::cout << ", v'=" << model->v.list[563%n_v] << std::endl;
        
    
    int idx = (dk0*model->v.n + vk0)*n_w_s + (0+intention);
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
    t_cost = new long [n_x]{};
    trans = new int[n_x_s*n_u]{};
    prob = new float[N*n_w_s*n_p]{};

    // for debug only
    xk0_debug = dk0*n_v;
    wk0_debug = dck0*2;

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
    if (debug)
    {
        std::string filename = "cpu_tran_part";
        int dim[] = {1, n_x_s, n_u};
        mat_to_file(filename, sizeof(dim)/sizeof(dim[0]), dim, trans);
    }
    // std::cout << "extract subset from state transition" << std::endl;

    // slicing transition probability (k,w,w')
    // k = k0+dk
    // std::cout << "get transition probability starts here" << std::endl;
    
    for (int k = 0; k < N; ++k)
    {
        for (int dwk = 0; dwk < n_w_s; ++dwk)
        {
            for (int dwk_ = 0; dwk_ < n_p; ++dwk_)
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
        int dim[] = {N, n_w_s, n_p};
        mat_to_file(filename, sizeof(dim)/sizeof(dim[0]), dim, prob);
    }
    // std::cout << "extract subset from transition probability" << std::endl;


    // slicing running_cost (k,x,w,u)
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

                solver_idx = dxk*n_w_s*n_u + dwk*n_u + uk;
                r_cost[solver_idx] = model->r_cost[idx];
                r_mask[solver_idx] = model->r_mask[idx];
            }
        }
    }
    // std::cout << "extract subset from running cost" << std::endl;

    // generate terminal cost
    for (int dk = 0; dk < n_d; ++dk)
    {
        for (int vk = 0; vk < n_v; ++vk)
        {
            idx = (dk*n_v+vk);
            t_cost[idx] = model->terminal_cost(dk0, dk, vk);
        }
    }

    if (debug)
    {
        std::string filename = "t_cost_dv";
        int dim[] = {1, n_d, n_v};
        mat_to_file(filename, sizeof(dim)/sizeof(dim[0]), dim, t_cost);
    }
    // std::cout << "generate terminal cost" << std::endl;

    return 0;
}