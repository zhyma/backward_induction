#include "gpu_solver.h"
#include "gpu_kernel.h"

GPUSolver::GPUSolver(DPModel * ptr_in, int block_size_in)
{
    model = ptr_in;
    block_size = block_size_in;

    N = model->N_pred;
    n_v = model->v.n;
    n_d = model->n_d;
    n_dc = model->n_dc;

    // n_p_default = (max_last_step+1)*2
    // this n_p is not optimizing the GPU process, need a new one instead
    n_p_default = model->n_p; // for n_d=121, n_p=(12+1)*2

    if (n_p_default < 32)
        n_p = 32;
    else if (n_p_default < 64)
        n_p = 64;
    else if (n_p_default < 128)
        n_p = 128;

    std::cout << "For GPU, n_p is: " << n_p << std::endl;

    int n_d_s  = n_d  - model->max_last_step;
    int n_dc_s = n_dc - model->max_last_step;
    // s stands for searching range
    n_x = n_d * n_v; // 128*32
    n_x_s = n_d_s  * n_v;
    n_w_s = n_dc_s * 2;
    n_w   = (n_dc_s + (n_p/2-1))*2;

    n_u = model->u.n;
    if (n_u < 32)
        n_u_expand = 32;
    else if (n_u < 64)
        n_u_expand = 64;
    std::cout << n_dc_s << std::endl;
    std::cout << "n_x, " << n_x << std::endl;
    std::cout << "n_w, " << n_w << std::endl;
    std::cout << "n_u, " << n_u << std::endl;
    
    u_expand.init(n_u_expand);
    int i = 0;
    for (; i < n_u; ++i)
        u_expand.cpu[i] = i;
    for (; i < n_u_expand; ++i)
        u_expand.cpu[i] = n_u - 1;

    value.init(N+1, n_x, n_w);
    q.init(n_x_s, n_w_s, n_u);
    action.init(N, n_x_s, n_w_s);
    trans.init(n_x_s, n_u);
    prob.init(N, n_w_s, n_p);
    r_cost.init(n_x_s, n_w_s, n_u);
    r_mask.init(n_x_s, n_w_s, n_u);

    // Allocate memory
    cudaMalloc(&u_expand.gpu, u_expand.size_b);
    // You can do (N+1) for the whole value table, or 2 as a ping-pong buffer
    // The whole value table size will be (N+1)*N_x*N_w
    // Ping-pong buffer type will be 2*N_x*N_w
    cudaMalloc(&value.gpu, value.size_b);
    cudaMalloc(&q.gpu, q.size_b);
    cudaMalloc(&action.gpu, action.size_b);
    // terminal cost NxNw
    // cudaMalloc(&t_cost_gpu, t_cost_size);
    // state transition matrix Nx*Nw*Nu
    cudaMalloc(&trans.gpu, trans.size_b);
    // transition probability matrix size: N*Nw*Nw
    cudaMalloc(&prob.gpu, prob.size_b);
    // running cost N*Nx*Nw*Nu
    cudaMalloc(&r_cost.gpu, r_cost.size_b);
    cudaMalloc(&r_mask.gpu, r_mask.size_b);

    cudaMemcpy(u_expand.gpu, u_expand.cpu, u_expand.size_b, cudaMemcpyHostToDevice);

    return;
}

GPUSolver::~GPUSolver()
{
    // Free memory
    cudaFree(u_expand.gpu);
    cudaFree(value.gpu);
    cudaFree(q.gpu);
    cudaFree(action.gpu);
    cudaFree(trans.gpu);
    cudaFree(prob.gpu);
    cudaFree(r_cost.gpu);
    cudaFree(r_mask.gpu);
}

int GPUSolver::solve(int k0, float d0, float v0, float dc0, int intention)
{
    int dk0 = model->get_dist_idx(d0);
    int vk0 = model->get_velc_idx(v0);
    int dck0 = model->get_dist_idx(dc0);

    get_subset(k0, dk0, dck0);
    // std::cout << "extract subset done." << std::endl;

    // For finding the minimum from a sort of q-value
    dim3 grid(n_x_s, n_w_s);
    int block = 32;

    for (int k = N-1; k >= 0; k--)
    {
        switch(n_d)
        {
            case 121:
                bi_kernel<<<grid, block>>> \
                (k0, k, n_d, n_v, n_u, u_expand.gpu, \
                 r_cost.gpu, r_mask.gpu, trans.gpu, prob.gpu, \
                 value.gpu, action.gpu);
        }
    }

    // Backup data before it's too late
    cudaMemcpy(value.cpu, value.gpu, value.size_b, cudaMemcpyDeviceToHost);
    cudaMemcpy(action.cpu, action.gpu, action.size_b, cudaMemcpyDeviceToHost);

    return 0;
}

int GPUSolver::get_subset(int k0, int dk0, int dck0)
{
    // std::cout << "get subset" << std::endl;

    // long long int idx = 0;
    long idx = 0;
    long solver_idx = 0;

    //slicing state transition (x,w,u)
    for (long dxk = 0; dxk < n_x_s; ++dxk)
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
            trans.cpu[solver_idx] = model->s_trans_table[idx] - dk0*n_v;
        }
    }
    if(false)
    {
        std::string filename = "partial_trans_gpu";
        long dim[] = {1, n_x_s, n_u};
        mat_to_file(filename, sizeof(dim)/sizeof(dim[0]), dim, trans.cpu);
    }
    // std::cout << "extract subset from state transition" << std::endl;

    //slicing transition probability (k,w,w')
    // k = k0+dk
    for (int dk = 0; dk < N; ++dk)
    {
        for (int dwk = 0; dwk < n_w_s; ++dwk)
        {
            // i = ddwk+1, ddwk is [-1, n_p-1]
            for (int i = 0; i < n_p; ++i)
            {
                solver_idx = dk*n_w_s*n_p + dwk*n_p + i;
                if (i < n_p_default)
                {
                    // n_p_default = 28 accessible next states
                    idx = (k0+dk) * model->w.n * model->n_p + (dck0*2+dwk) * model->n_p + i;
                    prob.cpu[solver_idx] = model->prob_table[idx];
                }
                else
                {
                    // the rest are tiled for GPU
                    prob.cpu[solver_idx] = 0;
                }
            }
        }
    }
    if(false)
    {
        std::string filename = "partial_prob_gpu";
        long dim[] = {N, n_w_s, n_p};
        mat_to_file(filename, sizeof(dim)/sizeof(dim[0]), dim, prob.cpu);
    }
    // std::cout << "extract subset from transition probability" << std::endl;


    //slicing running_cost (k,x,w,u)
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
                int temp1 = (dk0 * model->v.n + dxk) * model->w.n * model->u.n;
                // uk dimension: uk
                int temp2 = (dck0*2 + dwk) * model->u.n;
                int temp3 = uk;
                idx =  temp1 + temp2 + temp3;

                solver_idx = dxk*n_w_s*n_u + dwk*n_u + uk;
                r_cost.cpu[solver_idx] = model->r_cost[idx];
                r_mask.cpu[solver_idx] = model->r_mask[idx];
            }
        }
    }
    // std::cout << "extract subset from running cost" << std::endl;

    // generate terminal cost
    for (long xk = 0; xk < n_x; ++xk)
    {
        for (int wk = 0; wk < n_dc*2; ++wk)
        {
            idx = N*n_x*n_w + xk * n_w + wk;
            float t_cost = model->terminal_cost(xk, wk);
            // if (wk < n_w_s)
            value.cpu[idx] = t_cost;
            // else
            //     value[idx] = 0;
        }
    }
    // std::cout << "place terminal cost into the value table" << std::endl;

    cudaMemcpy(value.gpu, value.cpu, value.size_b, cudaMemcpyHostToDevice);
    // Initialize "index" transition matrix <x,w> -u-> x'
    cudaMemcpy(trans.gpu, trans.cpu, trans.size_b, cudaMemcpyHostToDevice);
    // Initialize transition probability matrix w -> w'
    cudaMemcpy(prob.gpu, prob.cpu, prob.size_b, cudaMemcpyHostToDevice);
    // initialize x, w, and u value to GPU for reference arrays on the host
    cudaMemcpy(r_cost.gpu, r_cost.cpu, r_cost.size_b, cudaMemcpyHostToDevice);
    cudaMemcpy(r_mask.gpu, r_mask.cpu, r_mask.size_b, cudaMemcpyHostToDevice);
    
    return 0;
}