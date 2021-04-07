#include "gpu_solver.h"
#include "gpu_kernel.h"

GPUSolver::GPUSolver(DPModel * ptr_in, int block_size_in)
{
    debug = true;
    
    model = ptr_in;
    block_size = block_size_in;

    N = model->N_pred;
    n_v = model->v.n;
    n_d = model->n_d; // 128
    n_dc = model->n_dc;

    // s stands for searching range
    n_x = n_d * n_v; // 128*32
    n_x_s = (n_d-model->max_last_step) * n_v; // 115*32
    // n_w = (n_d - model->max_last_step+15)*2; //130*2
    // needs to pad n_w so that the following state is n*32
    n_w = (n_dc - model->max_last_step + 15)*2;
    n_w_s = (n_w - 15) * 2; //
    n_u = model->u.n;

    n_p_default = model->n_p; // 28, offset 13
    n_p = 32; // offset 15

    value.init(N+1, n_x, n_w);
    action.init(N, n_x_s, n_w_s);
    q.init(n_x_s, n_w_s, n_u);

    cudaMalloc(&value.gpu, value.size_b);
    cudaMalloc(&q.gpu, q.size_b);
    cudaMalloc(&action.gpu, action.size_b);
    
    return;
}

GPUSolver::~GPUSolver()
{
    // Free memory
    cudaFree(r_cost.gpu);
    cudaFree(r_mask.gpu);
    cudaFree(trans.gpu);
    cudaFree(prob.gpu);
    cudaFree(value.gpu);
    cudaFree(q.gpu);
    cudaFree(action.gpu);
}

int GPUSolver::solve(int k0, float d0, float v0, float dc0, int intention)
{
    // just for find out the range (distance), so v and i are ignored.
    int dk0 = model->get_dist_idx(d0);
    int vk0 = model->get_velc_idx(v0);
    int dck0 = model->get_dist_idx(dc0);
    get_subset(k0, dk0, dck0);
    std::cout << "extract subset done." << std::endl;

    // std::cout << value.size_b << std::endl; 

    // Set up parameters for parallel computing
    // For calculating q-value
    dim3 q_grid(n_x_s, n_w_s, n_u);
    int q_block = n_p; // which is 32

    // For finding the minimum from a sort of q-value
    dim3 s_grid(n_x_s, n_w_s);
    int s_block = n_u;

    for (int k = N-1; k >= 0; k--)
    {
        switch(q_block)
        {
            case 1024:
                bi_q_kernel<1024><<<q_grid, 1024>>> \
                (k0, k, n_v, r_cost.gpu, r_mask.gpu, trans.gpu, prob.gpu, value.gpu, q.gpu);
                break;
            case 512:
                bi_q_kernel< 512><<<q_grid,  512>>> \
                (k0, k, n_v, r_cost.gpu, r_mask.gpu, trans.gpu, prob.gpu, value.gpu, q.gpu);
                break;
            case 256:
                bi_q_kernel< 256><<<q_grid,  256>>> \
                (k0, k, n_v, r_cost.gpu, r_mask.gpu, trans.gpu, prob.gpu, value.gpu, q.gpu);
                break;
            case 128:
                bi_q_kernel< 128><<<q_grid,  128>>> \
                (k0, k, n_v, r_cost.gpu, r_mask.gpu, trans.gpu, prob.gpu, value.gpu, q.gpu);
                break;
            case 64:
                bi_q_kernel<  64><<<q_grid,   64>>> \
                (k0, k, n_v, r_cost.gpu, r_mask.gpu, trans.gpu, prob.gpu, value.gpu, q.gpu);
                break;
            case 32:
                bi_q_kernel<  32><<<q_grid,   32>>> \
                (k0, k, n_v, r_cost.gpu, r_mask.gpu, trans.gpu, prob.gpu, value.gpu, q.gpu);
                break;
            case 16:
                bi_q_kernel<  16><<<q_grid,   16>>> \
                (k0, k, n_v, r_cost.gpu, r_mask.gpu, trans.gpu, prob.gpu, value.gpu, q.gpu);
                break;
            case  8:
                bi_q_kernel<   8><<<q_grid,    8>>> \
                (k0, k, n_v, r_cost.gpu, r_mask.gpu, trans.gpu, prob.gpu, value.gpu, q.gpu);
                break;
            case  4:
                bi_q_kernel<   4><<<q_grid,    4>>> \
                (k0, k, n_v, r_cost.gpu, r_mask.gpu, trans.gpu, prob.gpu, value.gpu, q.gpu);
                break;
            case  2:
                bi_q_kernel<   2><<<q_grid,    2>>> \
                (k0, k, n_v, r_cost.gpu, r_mask.gpu, trans.gpu, prob.gpu, value.gpu, q.gpu);
                break;
            case  1:
                bi_q_kernel<   1><<<q_grid,    1>>> \
                (k0, k, n_v, r_cost.gpu, r_mask.gpu, trans.gpu, prob.gpu, value.gpu, q.gpu);
                break;
        }
        bi_min_kernel<<<s_grid, s_block>>>(k, n_v, n_u, value.gpu, q.gpu, action.gpu);
    }

    // Backup data before it's too late
    cudaMemcpy(value.cpu, value.gpu, value.size_b, cudaMemcpyDeviceToHost);
    cudaMemcpy(action.cpu, action.gpu, action.size_b, cudaMemcpyDeviceToHost);

    int idx = ((dk0*n_v)+vk0) * n_w_s + (dck0*2+intention);

    return action.cpu[idx];
}

int GPUSolver::get_subset(int k0, int dk0, int dck0)
{
    int idx = 0;
    int solver_idx = 0;

    //slicing state transition (x,w,u)
    // Initialize "index" transition matrix <x,w> -u-> x'
    trans.init(n_x_s, n_u);
    // state transition matrix Nx*Nw*Nu
    cudaMalloc(&trans.gpu, trans.size_b);
    for (int dxk = 0; dxk < n_x_s; ++dxk)
    {
        for (int uk =0; uk < n_u; ++uk)
        {
            idx =  (dk0*n_v + dxk) * model->u.n + uk;
            solver_idx = dxk*n_u + uk;
            trans.cpu[solver_idx] = model->s_trans_table[idx] - dk0*n_v;
        }
    }
    cudaMemcpy(trans.gpu, trans.cpu, trans.size_b, cudaMemcpyHostToDevice);
    if(false)
    {
        std::string filename = "partial_trans_gpu";
        int dim[] = {1, n_x_s, n_u};
        mat_to_file(filename, sizeof(dim)/sizeof(dim[0]), dim, trans.cpu);
    }
    // std::cout << "extract subset from state transition" << std::endl;

    //slicing transition probability (k,w,w')
    // k = k0+dk
    // Initialize transition probability matrix w -> w'
    prob.init(N, n_w_s, n_p);
    // transition probability matrix size: N*Nw*Nw
    cudaMalloc(&prob.gpu, prob.size_b);
    for (int k = 0; k < N; ++k)
    {
        for (int dwk = 0; dwk < n_w_s; ++dwk)
        {
            for (int dwk_ = 0; dwk_ < n_p; ++dwk_)
            {
                idx = (k0+k) * model->w.n * model->n_p + (dck0*2+dwk) * model->n_p + dwk_;
                solver_idx = k*n_w_s*n_p + dwk*n_p + dwk_;
                prob.cpu[solver_idx] = model->prob_table[idx];
            }
        }
    }
    cudaMemcpy(prob.gpu, prob.cpu, prob.size_b, cudaMemcpyHostToDevice);
    if(false)
    {
        std::string filename = "partial_prob_gpu";
        int dim[] = {N, n_w_s, n_p};
        mat_to_file(filename, sizeof(dim)/sizeof(dim[0]), dim, prob.cpu);
    }
    // std::cout << "extract subset from transition probability" << std::endl;


    //slicing running_cost (k,x,w,u)
    // k = k0+dk
    // xk = xk0 + dxk = dk0*n_v + dxk
    // initialize x, w, and u value to GPU for reference arrays on the host
    r_cost.init(n_x_s, n_w_s, n_u);
    r_mask.init(n_x_s, n_w_s, n_u);
    // Allocate memory
    // running cost N*Nx*Nw*Nu
    cudaMalloc(&r_cost.gpu, r_cost.size_b);
    cudaMalloc(&r_mask.gpu, r_mask.size_b);
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
                r_cost.cpu[solver_idx] = model->r_cost[idx];
                r_mask.cpu[solver_idx] = model->r_mask[idx];
            }
        }
    }
    cudaMemcpy(r_cost.gpu, r_cost.cpu, r_cost.size_b, cudaMemcpyHostToDevice);
    cudaMemcpy(r_mask.gpu, r_mask.cpu, r_mask.size_b, cudaMemcpyHostToDevice);
    // std::cout << "extract subset from running cost" << std::endl;

    // generate terminal cost, directly apply to the value matrix at step N
    for (int xk = 0; xk < n_x; ++xk)
    {
        // wk cannot reach n_w because of padding
        for (int wk = 0; wk < n_dc*2; ++wk)
        {
            idx = N*n_x*n_w + xk * n_w + wk;
            value.cpu[idx] = model->terminal_cost(xk, wk);
        }
    }
    cudaMemcpy(value.gpu, value.cpu, value.size_b, cudaMemcpyHostToDevice);

    // std::cout << "place terminal cost into the value table" << std::endl;

    return 0;
}