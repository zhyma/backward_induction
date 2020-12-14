#ifndef GPU_SHARE_H_
#define GPU_SHARE_H_

struct q_info
{
    int index;
    float value;
};

template <unsigned int blockSize>
__device__ void warpReduce(volatile float* sdata_sum, int tid)
{
  if (blockSize>=64) sdata_sum[tid] += sdata_sum[tid + 32];
  if (blockSize>=32) sdata_sum[tid] += sdata_sum[tid + 16];
  if (blockSize>=16) sdata_sum[tid] += sdata_sum[tid + 8];
  if (blockSize>= 8) sdata_sum[tid] += sdata_sum[tid + 4];
  if (blockSize>= 4) sdata_sum[tid] += sdata_sum[tid + 2];
  if (blockSize>= 2) sdata_sum[tid] += sdata_sum[tid + 1];
}

// Kernel function to find the control/action with the lowest cost (q-value)
__global__ void bi_min_kernel(int n_u, int k, float *v, float *q, int *a)
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
__global__ void bi_terminal_kernel(int n_w, int k, float *t_cost, float *v)
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
    int g_idx = xk*n_w + wk;
    v[v_idx] = t_cost[g_idx];
  }
}


#endif //GPU_SHARE_H_