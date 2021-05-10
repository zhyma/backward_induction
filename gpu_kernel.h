#ifndef GPU_SHARE_H_
#define GPU_SHARE_H_

struct q_info
{
    int idx;
    float val;
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
  __shared__ float sdata_sum[blockSize];

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
  long n_x_s = gridDim.x;
  long n_w_s = gridDim.y;
  int n_u = gridDim.z;
  int n_p = blockDim.x;
  int tid = threadIdx.x;

  // Not a magic number, just too lazy to pass the "model->max_last_step"
  // and the padded w next
  long n_x = n_x_s + 12*n_v;
  long n_w = n_w_s + 15*2;

  if (blockSize >= 64)
  {
    n_x += 12*n_v;
    n_w += 16*2;
  }

  // STEP 1: find the following x_ by given k, x, w, u and the model
  // prepare transition matrix <xk, wk> -uk-> x'_idx
  // input k, xk, wk, uk, output x'_idx

  long xk_ = t[xk*(n_u) + uk];

  sdata_sum[tid] = 0;
  int i = 0;
  long p_offset = k*n_w_s*n_p + wk * n_p;
  // dwk is offseted by 1 (ref to dp_model)
  long v_offset = (k+1)*(n_x*n_w) + xk_*n_w + (wk - 1); // dwk is your offset

  while (i < n_p)
  {
    sdata_sum[tid] += p[p_offset+i+tid]*v[v_offset+i+tid];
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
    long idx = xk*n_w_s*n_u + wk*n_u + uk;

    if(r_mask[idx] & 1<<(k0+k))
      q[idx] = 1e15;
    else
      q[idx] = r_cost[idx] + sdata_sum[0];
  }
};

// Kernel function to find the control/action with the lowest cost (q-value)

__global__ void bi_min_kernel \
(int k0, int k, int n_d, int n_v, int n_u, int *u_expand, \
  float *r_cost, long int *r_mask, int *t, float *p, \
  float *v, int *a)
{
  // __shared__ int sdata_idx[32];
  // __shared__ float sdata_val[32];
  __shared__ q_info sdata[32];

  // <x, w> -u->
  // grid: 2D, <x,w>
  //  gridDim.x: N_x;    gridDim.y: N_w;
  // blockIdx.x: xk; blockIdx.y: wk;
  // block: 1D <u>, dimension is not N_u
  //  blockDim.x: 2^m, greater than N_u
  // threadIdx.x: uk
  long xk = blockIdx.x;
  long wk = blockIdx.y;
  long n_x_s = gridDim.x;
  long n_w_s = gridDim.y;

  int tid = threadIdx.x;
  int uk = u_expand[tid];

  long n_x = n_x_s + 12*n_v;
  long n_w = n_w_s + 15*2;

  int n_p = 32;

  if (n_d >= 241)
  {
    n_x += 12*n_v;
    n_w += 16*2;
  }

  long xk_ = t[xk*(n_u) + uk];
  long p_offset = k*n_w_s*n_p + wk * n_p;
  // dwk is offseted by 1 (ref to dp_model)
  long v_offset = (k+1)*(n_x*n_w) + xk_*n_w + (wk - 1); // dwk is your offset

  long idx = xk*n_w_s*n_u + wk*n_u + uk;

  sdata[tid].idx = uk;
  sdata[tid].val = p[p_offset+ 0]*v[v_offset+ 0] + p[p_offset+ 1]*v[v_offset+ 1] \
                     + p[p_offset+ 2]*v[v_offset+ 2] + p[p_offset+ 3]*v[v_offset+ 3] \
                     + p[p_offset+ 4]*v[v_offset+ 4] + p[p_offset+ 5]*v[v_offset+ 5] \
                     + p[p_offset+ 6]*v[v_offset+ 6] + p[p_offset+ 7]*v[v_offset+ 7] \
                     + p[p_offset+ 8]*v[v_offset+ 8] + p[p_offset+ 9]*v[v_offset+ 9] \
                     + p[p_offset+10]*v[v_offset+10] + p[p_offset+11]*v[v_offset+11] \
                     + p[p_offset+12]*v[v_offset+12] + p[p_offset+13]*v[v_offset+13] \
                     + p[p_offset+14]*v[v_offset+14] + p[p_offset+15]*v[v_offset+15] \
                     + p[p_offset+16]*v[v_offset+16] + p[p_offset+17]*v[v_offset+17] \
                     + p[p_offset+18]*v[v_offset+18] + p[p_offset+19]*v[v_offset+19] \
                     + p[p_offset+20]*v[v_offset+20] + p[p_offset+21]*v[v_offset+21] \
                     + p[p_offset+22]*v[v_offset+22] + p[p_offset+23]*v[v_offset+23] \
                     + p[p_offset+24]*v[v_offset+24] + p[p_offset+25]*v[v_offset+25] \
                     + (r_cost[idx] + ((r_mask[idx]>>(k0+k)) && 1)*1e15);

	__syncthreads();

	for (unsigned int s = blockDim.x/2; s > 0; s >>=1)
	{
    if (tid < s)
    {

      //find the min value and its idx.
      if (sdata[tid].val >= sdata[tid+s].val)
      {
        sdata[tid].idx = sdata[tid+s].idx;
        sdata[tid].val = sdata[tid+s].val;
      }
    }

		__syncthreads();
	}

  // STEP 
  if (tid == 0)
  {
    v[k*n_x*n_w + xk*n_w + wk] = sdata[0].val;
    a[k*n_x_s*n_w_s + xk*n_w_s + wk] = sdata[0].idx;
  }

};

#endif