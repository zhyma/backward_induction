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
};

// Kernel function to calculate the control/action cost
template <unsigned int blockSize>
__global__ void bi_q_kernel(int k0, int k, int n_v, float *r_cost, long int *r_mask, int *t, float *p, float *v, float *q)
{
  //max number of thread possible, some will not be used
  __shared__ float sdata_sum[32];

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
  int n_x_s = gridDim.x;
  int n_w_s = gridDim.y;
  int n_u = gridDim.z;
  int n_p = blockDim.x;
  int tid = threadIdx.x;

  // Not a magic number, just too lazy to pass the "model->max_last_step"
  // and the padded w next
  int n_x = n_x_s + 13*n_v;
  int n_w = n_w_s + 15*2;

  // STEP 1: find the following x_ by given k, x, w, u and the model
  // prepare transition matrix <xk, wk> -uk-> x'_idx
  // input k, xk, wk, uk, output x'_idx

  int xk_ = t[xk*(n_u) + uk];

  sdata_sum[tid] = 0;
  int i = 0;
  int p_offset = k*n_w_s*n_p + wk * n_p;
  int v_offset = (k+1)*(n_x*n_w) + xk_*n_w + wk; // dwk is your offset

  while (i < n_p)
  {
    sdata_sum[tid] += p[p_offset+i+tid]*v[v_offset+i+tid] ;
    i += blockSize;
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
    //indexes for q and running cost are accidentally the same
    int idx = xk*n_w_s*n_u + wk*n_u + uk;

    if(r_mask[idx] & 1<<(k0+k))
      q[idx] = 1e15;
    else
      q[idx] = r_cost[idx] + sdata_sum[0];
  }
};

// Kernel function to find the control/action with the lowest cost (q-value)
__global__ void bi_min_kernel(int k, int n_v, int n_u, float *v, float *q, int *a)
{
  __shared__ q_info sdata_q[32];

  // <x, w> -u->
  // grid: 2D, <x,w>
  //  gridDim.x: N_x;    gridDim.y: N_w;
  // blockIdx.x: xk; blockIdx.y: wk;
  // block: 1D <u>, dimension is not N_u
  //  blockDim.x: 2^m, greater than N_u
  // threadIdx.x: uk
  int xk = blockIdx.x;
  int wk = blockIdx.y;
  int n_x_s = gridDim.x;
  int n_w_s = gridDim.y;
  int uk = threadIdx.x;
  int n_x = n_x_s + 13*n_v;
  int n_w = n_w_s + 15*2;

  int tid = threadIdx.x;
  // STEP 1: 
  // initialize each element with

  if (tid < n_u)
  {
    sdata_q[tid].index = tid;
    sdata_q[tid].value = q[xk*n_w_s*n_u + wk*n_u + uk];
  }
  else
  {
    sdata_q[tid].index = 0;
    sdata_q[tid].value = q[xk*n_w_s*n_u + wk*n_u];
  }
  
	__syncthreads();

	for (unsigned int s = blockDim.x/2; s > 0; s >>=1)
	{
    if (tid < s)
    {
      //find the min value and its idx.
      if (sdata_q[tid].value >= sdata_q[tid+s].value)
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
    v[k*n_x*n_w + xk*n_w + wk] = sdata_q[0].value;
    a[k*n_x_s*n_w_s + xk*n_w_s + wk] = sdata_q[0].index;
  }

};

#endif