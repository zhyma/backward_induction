#include <iostream>
#include "dp_model.h"

struct q_info
{
    int index;
    float value;
};

__device__ void warpReduce(volatile float* sdata_sum, int tid)
{
    sdata_sum[tid] += sdata_sum[tid + 32];
    sdata_sum[tid] += sdata_sum[tid + 16];
    sdata_sum[tid] += sdata_sum[tid + 8];
    sdata_sum[tid] += sdata_sum[tid + 4];
    sdata_sum[tid] += sdata_sum[tid + 2];
    sdata_sum[tid] += sdata_sum[tid + 1];
}

// Kernel function to calculate the control/action cost
template <unsigned int blockSize>
__global__ void bi_q_kernel(int k, float *x, float *w, float *u, int *t, float *p, float *v, float *q)
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
  int wk_ = threadIdx.x;

  // STEP 1: find the following x_ by given k, x, w, u and the model
  // prepare transition matrix <xk, wk> -uk-> x'_idx
  // input k, xk, wk, uk, output x'_idx
  int xk_ = t[xk*(n_w*n_u) + wk*n_u + uk];

  // STEP 2: by given transition probability matrix, calculate the p*v
  // find p(w -> w') 
  int p1_idx = wk * n_w + wk_;
  // find v(<x',w'>)
  int v1_idx = (k+1)*(n_x*n_w) + xk_*n_w + wk_;

  int p2_idx = wk * n_w + wk_ + n_w/2;
  int v2_idx = (k+1)*(n_x*n_w) + xk_*n_w + wk_ + n_w/2;

  int tid = threadIdx.x;
  // STEP 3: do the sum reduction here
  // initialize each element with corresponding pv 
  sdata_sum[tid] = (tid < n_w)?p[p1_idx]*v[v1_idx]:0;
  if (tid + n_w/2 < n_w) sdata_sum[tid] += p[p2_idx]*v[v2_idx];
  __syncthreads();

	// for (unsigned int s = blockDim.x/2; s > 32; s >>=1)
	// {
	// 	if (tid < s)
	// 		sdata_sum[tid] += sdata_sum[tid+s];
	// 	__syncthreads();
  // }
  if (blockSize >= 1024){
    if (tid < 512) {sdata_sum[tid] += sdata_sum[tid+512];} __syncthreads();}
  if (blockSize >= 512){
      if (tid < 256) {sdata_sum[tid] += sdata_sum[tid+256];} __syncthreads();}
  if (blockSize >= 256){
      if (tid < 128) {sdata_sum[tid] += sdata_sum[tid+128];} __syncthreads();}
  if (blockSize >= 128){
      if (tid < 64) {sdata_sum[tid] += sdata_sum[tid+64];} __syncthreads();}
  if (tid < 32) warpReduce(sdata_sum, tid);

  // STEP 4: calculate q = l(k,x,u) + sum(pv), write q to global memory
  if (tid == 0)
  {
    int q_idx = xk*n_w*n_u + wk*n_u + uk;
    q[q_idx] = x[xk]*x[xk] + u[uk]*u[uk] + sdata_sum[0];
  }
}

// Kernel function to find the control/action with the lowest cost (q-value)
__global__ void bi_min_kernel(int n_u, int k, float *x, float *w, float *u, int *t, float *p, float *v, float *q, int *a)
{
  __shared__ q_info sdata_q[1024];

  // <x, w> -u->
  // grid: 2D, <x,w>
  //  gridDim.x: N_x;    gridDim.y: N_w;
  // blockIdx.x: xk; blockIdx.y: wk;
  // block: 1D <u>, dimension is not N_u
  //  blockDim.x: 2^m, greater than N_u
  // threadIdx.x: uk
  int xk = blockIdx.x;
  int wk = blockIdx.y;
  int n_x = gridDim.x;
  int n_w = gridDim.y;
  int uk = threadIdx.x;

  int tid = threadIdx.x;
  // STEP 1: 
  // initialize each element with
  if (tid < n_u)
  {
    sdata_q[tid].index = tid;
    sdata_q[tid].value = q[xk*n_w*n_u + wk*n_u + uk];
  }
  else
  {
    sdata_q[tid].index = 0;
    sdata_q[tid].value = q[xk*n_w*n_u + wk*n_u];
  }
  
	__syncthreads();

	for (unsigned int s = blockDim.x/2; s > 0; s >>=1)
	{
    if (tid < s)
    {
      //find the min value and its idx.
      if (sdata_q[tid].value > sdata_q[tid+s].value)
      {
        sdata_q[tid].index = sdata_q[tid+s].index;
        sdata_q[tid].value = sdata_q[tid+s].value;
      }
    }

		__syncthreads();
	}

  // STEP 
  if (tid == 0)
  {
    int s_idx = k*n_x*n_w + xk*n_w + wk;
    v[s_idx] = sdata_q[0].value;
    a[s_idx] = sdata_q[0].index;
  }

}

// Kernel function to calculate the final cost/value at the last step
__global__ void bi_terminal_kernel(int n_w, int k, float *x, float *w, float *u, int *t, float *p, float *v, int *a)
{
  // <x, w> -u-> <x2, w2>
  // grid: 3D, <x,w,u>
  //  gridDim.x: N_x;    gridDim.y: N_w;    gridDim.z: N_u
  // blockIdx.x: xk; blockIdx.y: wk; blockIdx.z: uk
  // block: 1D <w>, dimension is not N_w
  //  blockDim.x: 2^m, greater than N_w
  // threadIdx.x: wk_

  int xk = blockIdx.x;
  int n_x = gridDim.x;
  int wk = threadIdx.x;

  if (wk < n_w)
  {
    int v_idx = k*(n_x*n_w) + xk * n_w + wk;
    v[v_idx] = (1-x[xk])*(1-x[xk]);
  }
}

int gpu_main(DPModel * model, float *v_out, int *a_out)
{
  int max_threads = 1024;
  int n_x = model->x_set.count;
  int n_w = model->w_set.count;
  int n_u = model->u_set.count;
  int N = model->N;

  float *x, *w, *u;
  int *t;
  float *p, *v;
  float *q;
  int *a;

  // Allocate memory
  cudaMalloc(&x, n_x*sizeof(float));
  cudaMalloc(&w, n_w*sizeof(float));
  cudaMalloc(&u, n_u*sizeof(float));
  
  cudaMalloc(&t, n_x * n_w * n_u*sizeof(int));
  // transition probability matrix size: Nw*Nw
  cudaMalloc(&p, n_w*n_w*sizeof(float));
  // You can do (N+1) for the whole value table, or 2 as a ping-pong buffer
  // The whole value table size will be (N+1)*N_x*N_w
  // Ping-pong buffer type will be 2*N_x*N_w
  cudaMalloc(&v, (N+1)*n_x*n_w*sizeof(float));
  cudaMalloc(&q, n_x*n_w*n_u*sizeof(float));
  // You may only need 
  cudaMalloc(&a, N*n_x*n_w*sizeof(int));

  // initialize x, w, and u value to GPU for reference arrays on the host
  cudaMemcpy(x, model->x_set.list, n_x*sizeof(float), cudaMemcpyHostToDevice);
  cudaMemcpy(w, model->w_set.list, n_w*sizeof(float), cudaMemcpyHostToDevice);
  cudaMemcpy(u, model->u_set.list, n_u*sizeof(float), cudaMemcpyHostToDevice);

  // Initialize "index" transition matrix <x,w> -u-> x'
  cudaMemcpy(t, model->s_trans_table, n_x*n_w*n_u*sizeof(int), cudaMemcpyHostToDevice);
  // Initialize transition probability matrix w -> w'
  cudaMemcpy(p, model->prob_table, n_w*n_w*sizeof(float), cudaMemcpyHostToDevice);

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

  // Here k = N, the last step
  bi_terminal_kernel<<<n_x, q_block>>>(n_w, N, x, w, u, t, p, v, a);
  // Wait for GPU to finish before accessing on host
  // cudaDeviceSynchronize();

  for (int k = N-1; k >= 0; k--)
  {
    switch(q_block)
    {
      case 1024:
        bi_q_kernel<1024/2><<<q_grid, q_block/2>>>(k, x, w, u, t, p, v, q);
        break;
      case 512:
        bi_q_kernel< 512/2><<<q_grid, q_block/2>>>(k, x, w, u, t, p, v, q);
        break;
      case 256:
        bi_q_kernel< 256/2><<<q_grid, q_block/2>>>(k, x, w, u, t, p, v, q);
        break;
      case 128:
        bi_q_kernel< 128/2><<<q_grid, q_block/2>>>(k, x, w, u, t, p, v, q);
        break;
      case 64:
        bi_q_kernel< 64/2><<<q_grid, q_block/2>>>(k, x, w, u, t, p, v, q);
        break;
      case 32:
        bi_q_kernel< 32/2><<<q_grid, q_block/2>>>(k, x, w, u, t, p, v, q);
        break;
    }
    
    // cudaDeviceSynchronize();

    bi_min_kernel<<<s_grid, s_block>>>(n_u, k, x, w, u, t, p, v, q, a);
    // cudaDeviceSynchronize();
  }
  // cudaDeviceSynchronize();

  // Backup data before it's too late
  cudaMemcpy(v_out, v, (N+1)*n_x*n_w*sizeof(float), cudaMemcpyDeviceToHost);
  cudaMemcpy(a_out, a, N*n_x*n_w*sizeof(int), cudaMemcpyDeviceToHost);

  // Free memory
  cudaFree(x);
  cudaFree(w);
  cudaFree(u);
  cudaFree(t);
  cudaFree(p);
  cudaFree(v);
  cudaFree(q);
  cudaFree(a);

  // std::cout << "optimal actions found" << std::endl;

  return 0;
}