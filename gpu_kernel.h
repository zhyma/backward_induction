#ifndef GPU_SHARE_H_
#define GPU_SHARE_H_

struct q_info
{
    int idx;
    float val;
};

template <unsigned int n_d>
__global__ void bi_kernel \
(int k0, int k, int n_v, int n_u, int max_last_step, int *u_expand, \
  float *r_cost, long int *r_mask, int *t, float *p, \
  float *v, int *a, long k_offset, long kn_offset)
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

  long n_x = n_x_s + max_last_step*n_v;
  long n_w = n_w_s + max_last_step*2;
  int n_p = (max_last_step+1)*2;


  long xk_ = t[xk*(n_u) + uk];
  long p_offset = k*n_w_s*n_p + wk * n_p;
  // dwk is offseted by 1 (ref to dp_model)
  long v_offset = (k+1)*(n_x*n_w) + xk_*n_w + (wk - 1); // dwk is your offset
  // long v_offset = kn_offset + xk_*n_w + (wk - 1);

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

  if (n_d > 121)
    sdata[tid].val += p[p_offset+26]*v[v_offset+26] + p[p_offset+27]*v[v_offset+27] \
                    + p[p_offset+28]*v[v_offset+28] + p[p_offset+29]*v[v_offset+29] \
                    + p[p_offset+30]*v[v_offset+30] + p[p_offset+31]*v[v_offset+31] \
                    + p[p_offset+32]*v[v_offset+32] + p[p_offset+33]*v[v_offset+33] \
                    + p[p_offset+34]*v[v_offset+34] + p[p_offset+35]*v[v_offset+35] \
                    + p[p_offset+36]*v[v_offset+36] + p[p_offset+37]*v[v_offset+37] \
                    + p[p_offset+38]*v[v_offset+38] + p[p_offset+39]*v[v_offset+39] \
                    + p[p_offset+40]*v[v_offset+40] + p[p_offset+41]*v[v_offset+41] \
                    + p[p_offset+42]*v[v_offset+42] + p[p_offset+43]*v[v_offset+43] \
                    + p[p_offset+44]*v[v_offset+44] + p[p_offset+45]*v[v_offset+45] \
                    + p[p_offset+46]*v[v_offset+46] + p[p_offset+47]*v[v_offset+47] \
                    + p[p_offset+48]*v[v_offset+48] + p[p_offset+49]*v[v_offset+49];

  // if (n_d > 241)
  //   sdata[tid].val += p[p_offset+50]*v[v_offset+50] + p[p_offset+51]*v[v_offset+51] \
  //                   + p[p_offset+52]*v[v_offset+52] + p[p_offset+53]*v[v_offset+53] \
  //                   + p[p_offset+54]*v[v_offset+54] + p[p_offset+55]*v[v_offset+55] \
  //                   + p[p_offset+56]*v[v_offset+56] + p[p_offset+57]*v[v_offset+57] \
  //                   + p[p_offset+58]*v[v_offset+58] + p[p_offset+59]*v[v_offset+59] \
  //                   + p[p_offset+60]*v[v_offset+60] + p[p_offset+61]*v[v_offset+61] \
  //                   + p[p_offset+62]*v[v_offset+62] + p[p_offset+63]*v[v_offset+63] \
  //                   + p[p_offset+64]*v[v_offset+64] + p[p_offset+65]*v[v_offset+65] \
  //                   + p[p_offset+66]*v[v_offset+66] + p[p_offset+67]*v[v_offset+67] \
  //                   + p[p_offset+68]*v[v_offset+68] + p[p_offset+69]*v[v_offset+69] \
  //                   + p[p_offset+70]*v[v_offset+70] + p[p_offset+71]*v[v_offset+71] \
  //                   + p[p_offset+72]*v[v_offset+72] + p[p_offset+73]*v[v_offset+73];

  // if (n_d > 361)
  //   sdata[tid].val += p[p_offset+74]*v[v_offset+74] + p[p_offset+75]*v[v_offset+75] \
  //                   + p[p_offset+76]*v[v_offset+76] + p[p_offset+77]*v[v_offset+77] \
  //                   + p[p_offset+78]*v[v_offset+78] + p[p_offset+79]*v[v_offset+79] \
  //                   + p[p_offset+80]*v[v_offset+80] + p[p_offset+81]*v[v_offset+81] \
  //                   + p[p_offset+82]*v[v_offset+82] + p[p_offset+83]*v[v_offset+83] \
  //                   + p[p_offset+84]*v[v_offset+84] + p[p_offset+85]*v[v_offset+85] \
  //                   + p[p_offset+86]*v[v_offset+86] + p[p_offset+87]*v[v_offset+87] \
  //                   + p[p_offset+88]*v[v_offset+88] + p[p_offset+89]*v[v_offset+89] \
  //                   + p[p_offset+90]*v[v_offset+90] + p[p_offset+91]*v[v_offset+91] \
  //                   + p[p_offset+92]*v[v_offset+92] + p[p_offset+93]*v[v_offset+93] \
  //                   + p[p_offset+94]*v[v_offset+94] + p[p_offset+95]*v[v_offset+95] \
  //                   + p[p_offset+96]*v[v_offset+96] + p[p_offset+97]*v[v_offset+97];

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
    // v[k_offset + xk*n_w + wk] = sdata[0].val;
    a[k*n_x_s*n_w_s + xk*n_w_s + wk] = sdata[0].idx;
  }

};

#endif