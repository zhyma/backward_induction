#include <iostream>
#include "dp_model.h"

__device__ struct q_info
{
    int index;
    float value;
};

__device__ void index2xwu(int i, int *dims)
{
    // from index, get x, w, u
    // dim[0, 1, 2] is dim[x, w, u]
    // i = x_idx * (Nw*Nu) + w_idx * (Nu) + u_idx
    // try to avoid using "%"

    // int x_idx = i/(dims[1]*dim[2]);
    // int i_ = i-x_idx*dim[1]*dim[2];
    // int w_idx = i_/dims[2];
    // int u_idx = (i_ - w_idx*dims[2]);
}

// TODO: modify this kernel
// Kernel function to calculate the control/action cost
__global__ void bi_q_kernel(int k, float *x, float *w, float *u, int *t, float *p, float *v, q_info *q)
{
  //max number of thread possible, some will not be used
  extern __shared__ float sdata_sum[];

  // <x, w> -u-> <x2, w2>
  // grid: 3D, <x,w,u>
  //  gridDim.x: N_x;    gridDim.y: N_w;    gridDim.z: N_u
  // blockIdx.x: x_idx; blockIdx.y: w_idx; blockIdx.z: u_idx
  // block: 1D <w>, dimension is not N_w
  //  blockDim.x: 2^m, greater than N_w
  // threadIdx.x: w2_idx
  int x_idx = blockIdx.x;
  int w_idx = blockIdx.y;
  int u_idx = blockIdx.z;
  int n_x = gridDim.x;
  int n_w = gridDim.y;
  int n_u = gridDim.z;
  int w2_idx = threadIdx.x;

  // STEP 1: find the following x_ by given k, x, w, u and the model
  // prepare transition matrix <x_idx, w_idx> -u_idx-> x'_idx
  // input k, x_idx, w_idx, u_idx, output x'_idx
  int x2_idx = t[x_idx*(n_w*n_u)+w_idx*n_u+n_u];

  // STEP 2: by given transition probability matrix, calculate the p*v
  // find p(w -> w') 
  int p_idx = w_idx * n_w + w2_idx;
  // find v(<x',w'>)
  int v_idx = k*(n_x*n_w) + x2_idx * n_w + w2_idx;

  int tid = threadIdx.x;
  // STEP 3: do the sum reduction here
  // initialize each element with corresponding pv 
  sdata_sum[tid] = (tid < n_w)?p[p_idx]*v[v_idx]:0;
	__syncthreads();


	for (unsigned int s = blockDim.x/2; s > 0; s >>=1)
	{
		if (tid < s)
			sdata_sum[tid] += sdata_sum[tid+s];
		__syncthreads();
	}

  // STEP 4: calculate q = l(k,x,u) + sum(pv), write q to global memory
  if (tid == 0)
  {
    int q_idx = k*n_x*n_w*n_u + x_idx*n_w*n_u + w_idx*n_u + u_idx;
    q[q_idx].index = u_idx;
    q[q_idx].value = x[x_idx]*x[x_idx] + u[u_idx]*u[u_idx] + sdata_sum[0];
  }

}

// TODO: modify this kernel
// Kernel function to find the control/action with the lowest cost (q-value)
__global__ void bi_min_kernel(int n_u, int k, float *x, float *w, float *u, int *t, float *p, float *v, q_info *q, int *a)
{
  extern __shared__ q_info sdata_q[];

  q_info max_q;
  max_q.index = 0;
  max_q.value = 10e6;

  // <x, w> -u->
  // grid: 2D, <x,w>
  //  gridDim.x: N_x;    gridDim.y: N_w;
  // blockIdx.x: x_idx; blockIdx.y: w_idx;
  // block: 1D <u>, dimension is not N_u
  //  blockDim.x: 2^m, greater than N_u
  // threadIdx.x: u_idx
  int x_idx = blockIdx.x;
  int w_idx = blockIdx.y;
  int n_x = gridDim.x;
  int n_w = gridDim.y;
  int u_idx = threadIdx.x;

  int tid = threadIdx.x;
  int q_idx = k*n_x*n_w*n_u + x_idx*n_w*n_u + w_idx*n_u + u_idx;
  // STEP 1: 
  // initialize each element with  
  sdata_q[tid] = (tid < n_u)?q[q_idx]:max_q;
	__syncthreads();

	for (unsigned int s = blockDim.x/2; s > 0; s >>=1)
	{
    if (tid < s)
      //find the min value and its idx.
			sdata_q[tid] = (sdata_q[tid].value<sdata_q[tid+s].value)?sdata_q[tid]:sdata_q[tid+s];
		__syncthreads();
	}

  // STEP 
  if (tid == 0)
  {
    int s_idx = k*n_x*n_w + x_idx*n_w + w_idx;
    v[s_idx] = sdata_q[0].value;
    a[s_idx] = sdata_q[0].index;
  }

}

// Kernel function to calculate the final cost/value at the last step
__global__ void bi_terminal_kernel(int k, float *x, float *w, float *u, int *t, float *p, float *v, int *a)
{
  // <x, w> -u-> <x2, w2>
  // grid: 3D, <x,w,u>
  //  gridDim.x: N_x;    gridDim.y: N_w;    gridDim.z: N_u
  // blockIdx.x: x_idx; blockIdx.y: w_idx; blockIdx.z: u_idx
  // block: 1D <w>, dimension is not N_w
  //  blockDim.x: 2^m, greater than N_w
  // threadIdx.x: w2_idx

  int x_idx = blockIdx.x;
  int w_idx = blockIdx.y;
  int u_idx = blockIdx.z;
  int n_x = gridDim.x;
  int n_w = gridDim.y;
  int n_u = gridDim.z;

  int v_idx = k*(n_x*n_w) + x_idx * n_w + w_idx;
  v[v_idx] = (1-x[x_idx])*(1-x[x_idx]);
}

int gpu_main(DPModel * model, float *v_out, int *a_out)
{
  int max_threads = 1024;
  int n_x = model->x_set.count;
  int n_w = model->w_set.count;
  int n_u = model->u_set.count;
  int N = model->N;

  int len = n_x * n_w * n_u;
  float *x, *w, *u;
  int *t;
  float *p, *v;
  q_info *q;
  int *a;

  // Allocate Unified Memory . accessible from CPU or GPU
  cudaMallocManaged(&x, n_x*sizeof(float));
  cudaMallocManaged(&w, n_w*sizeof(float));
  cudaMallocManaged(&u, n_u*sizeof(float));
  
  cudaMallocManaged(&t, len*sizeof(int));
  // transition probability matrix size: Nw*Nw
  cudaMallocManaged(&p, n_w*n_w*sizeof(float));
  // You can do (N+1) for the whole value table, or 2 as a ping-pong buffer
  // The whole value table size will be (N+1)*N_x*N_w
  // Ping-pong buffer type will be 2*N_x*N_w
  cudaMallocManaged(&v, (N+1)*n_x*n_w*sizeof(float));
  cudaMallocManaged(&q, (N+1)*n_x*n_w*n_u*sizeof(q_info));
  // You may only need 
  cudaMallocManaged(&a, N*n_x*n_w*sizeof(int));

  // initialize x, w, and u value to GPU for reference arrays on the host
  memcpy(x, model->x_set.list, n_x*sizeof(float));
  memcpy(w, model->w_set.list, n_w*sizeof(float));
  memcpy(u, model->u_set.list, n_u*sizeof(float));

  // Initialize "index" transition matrix <x,w> -u-> x'
  memcpy(t, model->s_trans_table, n_x*n_w*n_u*sizeof(int));
  // Initialize transition probability matrix w -> w'
  memcpy(p, model->prob_table, n_w*n_w*sizeof(float));
  // for (int i = 0; i < n_w; ++i)
  // {
  //   for (int j=0; j < n_w; ++j)
  //     p[i*n_w + j] = model->prob->prob_mat[i*n_w + j]
  // }

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
  bi_terminal_kernel<<<q_grid, q_block>>>(N, x, w, u, t, p, v, a);
  // Wait for GPU to finish before accessing on host
  cudaDeviceSynchronize();

  
  for (int k = N-1; k >= 0; k--)
  {
    //bi_q_kernel<<<grid, blockSize, pow2>>>(k, x, w, u, t, p, v, a);
    //bi_q_kernel<<<q_grid, q_block, q_block*sizeof(float)>>>(k, x, w, u, t, p, v, q, a);
    bi_q_kernel<<<q_grid, q_block, max_threads*sizeof(q_info)>>>(k, x, w, u, t, p, v, q);
    cudaDeviceSynchronize();
    //bi_min_kernel<<<s_grid, s_block, s_block*sizeof(float)>>>(k, x, w, u, t, p, v, q, a);
    bi_min_kernel<<<s_grid, s_block, max_threads*sizeof(q_info)>>>(n_u, k, x, w, u, t, p, v, q, a);
  }

  // Backup data before it's too late
  memcpy(v_out, v, (N+1)*n_x*n_w*sizeof(float));
  memcpy(a_out, a, N*n_x*n_w*sizeof(int));

  // Free memory
  cudaFree(x);
  cudaFree(w);
  cudaFree(u);
  cudaFree(t);
  cudaFree(p);
  cudaFree(v);
  cudaFree(q);
  cudaFree(a);

  std::cout << "optimal actions found" << std::endl;

  return 0;
}