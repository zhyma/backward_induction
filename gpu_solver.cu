#include <iostream>
#include <ctime>

#include <atomic>
#include <chrono>
#include <thread>

#include "dp_model.h"
#include "gpu_share.h"

// Kernel function to calculate the control/action cost
template <unsigned int blockSize>
__global__ void bi_q_kernel(int k, float *running_cost, int *t, float *p, float *v, float *q)
{
  //max number of thread possible, some will not be used
  __shared__ float sdata_sum[1024];

  // <x, w> -u-> <x2, w2>
  // grid: 3D, <x,w,u>
  //  gridDim.x: N_x;    gridDim.y: N_w;    gridDim.z: N_u
  // blockIdx.x: xk; blockIdx.y: wk; blockIdx.z: uk
  // block: 1D <w>, dimension is not N_w
  //  blockDim.x: 2^m, greater than N_w
  // threadIdx.x: wk_
  int xk = blockIdx.x;
  int wk = blockIdx.y;
  int uk = blockIdx.z;
  int n_x = gridDim.x;
  int n_w = gridDim.y;
  int n_u = gridDim.z;
  int tid = threadIdx.x;

  // STEP 1: find the following x_ by given k, x, w, u and the model
  // prepare transition matrix <xk, wk> -uk-> x'_idx
  // input k, xk, wk, uk, output x'_idx
  int xk_ = t[xk*(n_w*n_u) + wk*n_u + uk];

  sdata_sum[tid] = 0;
  int i = 0;
  int p_offset = k*n_w*n_w + wk * n_w;
  int v_offset = (k+1)*(n_x*n_w) + xk_*n_w;
  while (i < n_w)
  {
    sdata_sum[tid] += p[p_offset+i+tid]*v[v_offset+i+tid] + p[p_offset+i+blockSize+tid]*v[v_offset+i+blockSize+tid];
    i += blockSize * 2;
  }
  __syncthreads();

  if (blockSize >= 1024){
    if (tid < 512) {sdata_sum[tid] += sdata_sum[tid+512];} __syncthreads();}
  if (blockSize >= 512){
      if (tid < 256) {sdata_sum[tid] += sdata_sum[tid+256];} __syncthreads();}
  if (blockSize >= 256){
      if (tid < 128) {sdata_sum[tid] += sdata_sum[tid+128];} __syncthreads();}
  if (blockSize >= 128){
      if (tid < 64) {sdata_sum[tid] += sdata_sum[tid+64];} __syncthreads();}
  if (tid < 32) warpReduce<blockSize>(sdata_sum, tid);

  // STEP 4: calculate q = l(k,x,u) + sum(pv), write q to global memory
  if (tid == 0)
  {
    int q_idx = xk*n_w*n_u + wk*n_u + uk;
    //q[q_idx] = x[xk]*x[xk] + u[uk]*u[uk] + sdata_sum[0];
    q[q_idx] = running_cost[q_idx] + sdata_sum[0];
  }
}

// int gpu_main(DPModel * model, int block_size, float *v_out, int *a_out)
int gpu_main(DPModel * model, int block_size, float *v_out, int *a_out, std::atomic<int>* busy_p_mat, std::atomic<bool>* running)
{
  // int max_threads = 1024;
  std::clock_t start;
  double gpu_duration = 0;
  int prev_busy_mat = 0;

  int n_x = model->x.n;
  int n_w = model->w.n;
  int n_u = model->u.n;
  int N = model->N;

  float *running_cost, *t_cost;
  int *t;
  float *p, *v;
  float *q;
  int *a;

  // Allocate memory
  cudaMalloc(&running_cost, n_x*n_w*n_u*sizeof(float));
  cudaMalloc(&t_cost,  n_x*n_w*sizeof(float));
  
  cudaMalloc(&t, n_x * n_w * n_u*sizeof(int));
  // transition probability matrix size: Nw*Nw
  cudaMalloc(&p, N*n_w*n_w*sizeof(float));
  // You can do (N+1) for the whole value table, or 2 as a ping-pong buffer
  // The whole value table size will be (N+1)*N_x*N_w
  // Ping-pong buffer type will be 2*N_x*N_w
  cudaMalloc(&v, (N+1)*n_x*n_w*sizeof(float));
  cudaMalloc(&q, n_x*n_w*n_u*sizeof(float));
  // You may only need 
  cudaMalloc(&a, N*n_x*n_w*sizeof(int));

  // initialize x, w, and u value to GPU for reference arrays on the host
  cudaMemcpy(running_cost, model->running_cost, n_x*n_w*n_u*sizeof(float), cudaMemcpyHostToDevice);
  cudaMemcpy(t_cost, model->t_cost, n_x*n_w*sizeof(float), cudaMemcpyHostToDevice);

  // Initialize "index" transition matrix <x,w> -u-> x'
  cudaMemcpy(t, model->s_trans_table, n_x*n_w*n_u*sizeof(int), cudaMemcpyHostToDevice);
  // Initialize transition probability matrix w -> w'
  cudaMemcpy(p, model->prob_table[*busy_p_mat], N*n_w*n_w*sizeof(float), cudaMemcpyHostToDevice);

  // Set up parameters for parallel computing
  // For calculating q-value
  dim3 q_grid(n_x, n_w, n_u);

  int w_pow2 = 1;
  while (w_pow2 < n_w)
    w_pow2 <<= 1;
  int q_block = w_pow2;

  // For finding the minimum from a sort of q-value
  dim3 s_grid(n_x, n_w);
  int u_pow2 = 1;
  while (u_pow2 < n_u)
    u_pow2 <<= 1;
  int s_block = u_pow2;

  while(*running)
  {
    if (prev_busy_mat != *busy_p_mat)
    {
      cudaMemcpy(p, model->prob_table[*busy_p_mat], N*n_w*n_w*sizeof(float), cudaMemcpyHostToDevice);
      prev_busy_mat = *busy_p_mat;
    }
    std::cout << "GPU using p_mat buffer #" << (*busy_p_mat) << std::endl;

    start = std::clock();

    // Here k = N, the last step
    bi_terminal_kernel<<<n_x, q_block>>>(n_w, N, t_cost, v);
    // Wait for GPU to finish before accessing on host
    // cudaDeviceSynchronize();

    if (block_size >= n_w)
      q_block = n_w/2;
    else
      q_block = block_size;
    for (int k = N-1; k >= 0; k--)
    {
      switch(q_block)
      {
        case 1024:
          bi_q_kernel<1024><<<q_grid, 1024>>>(k, running_cost, t, p, v, q);
          break;
        case 512:
          bi_q_kernel< 512><<<q_grid,  512>>>(k, running_cost, t, p, v, q);
          break;
        case 256:
          bi_q_kernel< 256><<<q_grid,  256>>>(k, running_cost, t, p, v, q);
          break;
        case 128:
          bi_q_kernel< 128><<<q_grid,  128>>>(k, running_cost, t, p, v, q);
          break;
        case 64:
          bi_q_kernel<  64><<<q_grid,   64>>>(k, running_cost, t, p, v, q);
          break;
        case 32:
          bi_q_kernel<  32><<<q_grid,   32>>>(k, running_cost, t, p, v, q);
          break;
        case 16:
          bi_q_kernel<  16><<<q_grid,   16>>>(k, running_cost, t, p, v, q);
          break;
        case  8:
          bi_q_kernel<   8><<<q_grid,    8>>>(k, running_cost, t, p, v, q);
          break;
        case  4:
          bi_q_kernel<   4><<<q_grid,    4>>>(k, running_cost, t, p, v, q);
          break;
        case  2:
          bi_q_kernel<   2><<<q_grid,    2>>>(k, running_cost, t, p, v, q);
          break;
        case  1:
          bi_q_kernel<   1><<<q_grid,    1>>>(k, running_cost, t, p, v, q);
          break;
      }

      bi_min_kernel<<<s_grid, s_block>>>(n_u, k, v, q, a);
    }

    // Backup data before it's too late
    cudaMemcpy(v_out, v, (N+1)*n_x*n_w*sizeof(float), cudaMemcpyDeviceToHost);
    cudaMemcpy(a_out, a, N*n_x*n_w*sizeof(int), cudaMemcpyDeviceToHost);

    gpu_duration = (std::clock() - start) / (double)CLOCKS_PER_SEC;
    std::cout << "GPU time: " << gpu_duration << " s" << std::endl;

    std::this_thread::sleep_for(std::chrono::seconds(2));
  }

  // Free memory
  cudaFree(running_cost);
  cudaFree(t_cost);
  cudaFree(t);
  cudaFree(p);
  cudaFree(v);
  cudaFree(q);
  cudaFree(a);

  std::cout << "Exiting GPU solver" << std::endl;

  return 0;
}