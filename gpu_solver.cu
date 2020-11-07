#include <iostream>
#include "dp_model.h"

struct q_info
{
    int index;
    float value;
};

int find_min(float *q, int cnt, q_info *min)
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

// Kernel function to calculate the control/action cost
__global__ void bi_q_kernel(int k, float *x, float *w, float *u, int *t, float *p, float *v, float *q, float *test_table)
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
  int p_idx = wk * n_w + wk_;
  // find v(<x',w'>)
  int v_idx = (k+1)*(n_x*n_w) + xk_*n_w + wk_;

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

int gpu_main(DPModel * model, float *v_out, int *a_out, float *q_out)
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
  float *test_table;

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

  cudaMalloc(&test_table, N*n_x*n_w*n_u*sizeof(float));

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
  cudaDeviceSynchronize();

  // cudaMemcpy(v_out, v, (N+1)*n_x*n_w*sizeof(float), cudaMemcpyDeviceToHost);

  for (int k = N-1; k >= 0; k--)
  {
    //bi_q_kernel<<<grid, blockSize, pow2>>>(k, x, w, u, t, p, v, a);
    //bi_q_kernel<<<q_grid, q_block, q_block*sizeof(float)>>>(k, x, w, u, t, p, v, q, a);
    bi_q_kernel<<<q_grid, q_block>>>(k, x, w, u, t, p, v, q, test_table);
    cudaDeviceSynchronize();

    cudaMemcpy(q_out, q, n_x*n_w*n_u*sizeof(float), cudaMemcpyDeviceToHost);
    //bi_min_kernel<<<s_grid, s_block, s_block*sizeof(float)>>>(k, x, w, u, t, p, v, q, a);
    bi_min_kernel<<<s_grid, s_block>>>(n_u, k, x, w, u, t, p, v, q, a);
    cudaDeviceSynchronize();
    // break;
    // for (int xk = 0; xk < n_x; ++xk)
    // {
    //   for (int wk = 0; wk < n_w; ++wk)
    //   {
    //     q_info *min = new q_info[1];
    //     find_min(&q_out[xk*n_w*n_u + wk*n_u], n_u, min);
    //     v_out[k*n_x*n_w + xk*n_w + wk] = min[0].value;
    //     a_out[k*n_x*n_w + xk*n_w + wk] = min[0].index;
    //   }
    // }
    // cudaMemcpy(v, v_out, (N+1)*n_x*n_w*sizeof(float), cudaMemcpyHostToDevice);
  }

  // Backup data before it's too late
  cudaMemcpy(v_out, v, (N+1)*n_x*n_w*sizeof(float), cudaMemcpyDeviceToHost);
  cudaMemcpy(a_out, a, N*n_x*n_w*sizeof(int), cudaMemcpyDeviceToHost);
  cudaMemcpy(q_out, test_table, N*n_x*n_w*n_u*sizeof(float), cudaMemcpyDeviceToHost);

  // Free memory
  cudaFree(x);
  cudaFree(w);
  cudaFree(u);
  cudaFree(t);
  cudaFree(p);
  cudaFree(v);
  cudaFree(q);
  cudaFree(a);
  cudaFree(test_table);

  std::cout << "optimal actions found" << std::endl;

  return 0;
}