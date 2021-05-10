#ifndef GPU_SHARE_H_
#define GPU_SHARE_H_

struct q_info
{
    int idx;
    float val;
};

__global__ void bi_kernel \
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
  sdata[tid].val = (r_cost[idx] + ((r_mask[idx]>>(k0+k)) && 1)*1e15)
                  + p[p_offset+ 0]*v[v_offset+ 0] + p[p_offset+ 1]*v[v_offset+ 1] \
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
                  + p[p_offset+24]*v[v_offset+24] + p[p_offset+25]*v[v_offset+25];

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