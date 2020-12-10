#include "dp_model.h"
#include "gpu_share.h"

template <unsigned int blockSize, unsigned int calcSize>
__device__ void blockReduce(float *p, float *v, int p_offset, int v_offset, volatile float* sdata_sum, int tid)
{
  // equal to ```sdata_sum[tid] += p[p_offset+i+tid]*v[v_offset+i+tid] + p[p_offset+i+blockSize+tid]*v[v_offset+i+blockSize+tid];```
  if (blockSize==64 && calcSize >=1024) sdata_sum[tid] += p[p_offset+896+tid]*v[v_offset+896+tid] + p[p_offset+960+tid]*v[v_offset+960+tid];
  if (blockSize==64 && calcSize >= 896) sdata_sum[tid] += p[p_offset+768+tid]*v[v_offset+768+tid] + p[p_offset+832+tid]*v[v_offset+832+tid];
  if (blockSize==64 && calcSize >= 768) sdata_sum[tid] += p[p_offset+640+tid]*v[v_offset+640+tid] + p[p_offset+704+tid]*v[v_offset+704+tid];
  if (blockSize==64 && calcSize >= 640) sdata_sum[tid] += p[p_offset+512+tid]*v[v_offset+512+tid] + p[p_offset+576+tid]*v[v_offset+576+tid];
  if (blockSize==64 && calcSize >= 512) sdata_sum[tid] += p[p_offset+384+tid]*v[v_offset+384+tid] + p[p_offset+448+tid]*v[v_offset+448+tid];
  if (blockSize==64 && calcSize >= 384) sdata_sum[tid] += p[p_offset+256+tid]*v[v_offset+256+tid] + p[p_offset+320+tid]*v[v_offset+320+tid];
  if (blockSize==64 && calcSize >= 256) sdata_sum[tid] += p[p_offset+128+tid]*v[v_offset+128+tid] + p[p_offset+192+tid]*v[v_offset+192+tid];
  if (blockSize==64 && calcSize >= 128) sdata_sum[tid] += p[p_offset+  0+tid]*v[v_offset+  0+tid] + p[p_offset+ 64+tid]*v[v_offset+ 64+tid];

  if (blockSize==32 && calcSize >=1024) sdata_sum[tid] += p[p_offset+960+tid]*v[v_offset+960+tid] + p[p_offset+992+tid]*v[v_offset+992+tid];
  if (blockSize==32 && calcSize >= 960) sdata_sum[tid] += p[p_offset+896+tid]*v[v_offset+896+tid] + p[p_offset+928+tid]*v[v_offset+928+tid];
  if (blockSize==32 && calcSize >= 896) sdata_sum[tid] += p[p_offset+832+tid]*v[v_offset+832+tid] + p[p_offset+864+tid]*v[v_offset+864+tid];
  if (blockSize==32 && calcSize >= 832) sdata_sum[tid] += p[p_offset+768+tid]*v[v_offset+768+tid] + p[p_offset+800+tid]*v[v_offset+800+tid];
  if (blockSize==32 && calcSize >= 768) sdata_sum[tid] += p[p_offset+704+tid]*v[v_offset+704+tid] + p[p_offset+736+tid]*v[v_offset+736+tid];
  if (blockSize==32 && calcSize >= 704) sdata_sum[tid] += p[p_offset+640+tid]*v[v_offset+640+tid] + p[p_offset+672+tid]*v[v_offset+672+tid];
  if (blockSize==32 && calcSize >= 640) sdata_sum[tid] += p[p_offset+576+tid]*v[v_offset+576+tid] + p[p_offset+608+tid]*v[v_offset+608+tid];
  if (blockSize==32 && calcSize >= 576) sdata_sum[tid] += p[p_offset+512+tid]*v[v_offset+512+tid] + p[p_offset+544+tid]*v[v_offset+544+tid];
  if (blockSize==32 && calcSize >= 512) sdata_sum[tid] += p[p_offset+448+tid]*v[v_offset+448+tid] + p[p_offset+480+tid]*v[v_offset+480+tid];
  if (blockSize==32 && calcSize >= 448) sdata_sum[tid] += p[p_offset+384+tid]*v[v_offset+384+tid] + p[p_offset+416+tid]*v[v_offset+416+tid];
  if (blockSize==32 && calcSize >= 384) sdata_sum[tid] += p[p_offset+320+tid]*v[v_offset+320+tid] + p[p_offset+352+tid]*v[v_offset+352+tid];
  if (blockSize==32 && calcSize >= 320) sdata_sum[tid] += p[p_offset+256+tid]*v[v_offset+256+tid] + p[p_offset+288+tid]*v[v_offset+288+tid];
  if (blockSize==32 && calcSize >= 256) sdata_sum[tid] += p[p_offset+192+tid]*v[v_offset+192+tid] + p[p_offset+224+tid]*v[v_offset+224+tid];
  if (blockSize==32 && calcSize >= 192) sdata_sum[tid] += p[p_offset+128+tid]*v[v_offset+128+tid] + p[p_offset+160+tid]*v[v_offset+160+tid];
  if (blockSize==32 && calcSize >= 128) sdata_sum[tid] += p[p_offset+ 64+tid]*v[v_offset+ 64+tid] + p[p_offset+ 96+tid]*v[v_offset+ 96+tid];
  if (blockSize==32 && calcSize >=  64) sdata_sum[tid] += p[p_offset+  0+tid]*v[v_offset+  0+tid] + p[p_offset+ 32+tid]*v[v_offset+ 32+tid];
}

// Kernel function to calculate the control/action cost
template <unsigned int blockSize, unsigned int calcSize>
__global__ void bi_q_kernel_beta(int k, float *x, float *w, float *u, int *t, float *p, float *v, float *q)
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
  int p_offset = k*n_w*n_w + wk * n_w;
  int v_offset = (k+1)*(n_x*n_w) + xk_*n_w;

  // int i = 0;
  // while (i < n_w)
  // {
  //   sdata_sum[tid] += p[p_offset+i+tid]*v[v_offset+i+tid] + p[p_offset+i+blockSize+tid]*v[v_offset+i+blockSize+tid];
  //   i += blockSize * 2;
  // }
  
  blockReduce<blockSize, calcSize>(p, v, p_offset, v_offset, sdata_sum, tid);
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
    q[q_idx] = x[xk]*x[xk] + u[uk]*u[uk] + sdata_sum[0];
  }
}

int gpu_main_beta(DPModel * model, int block_size, float *v_out, int *a_out)
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
  cudaMalloc(&p, N*n_w*n_w*sizeof(float));
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
  cudaMemcpy(p, model->prob_table, N*n_w*n_w*sizeof(float), cudaMemcpyHostToDevice);

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

  if (block_size >= n_w)
    q_block = n_w/2;
  else
    q_block = block_size;
  for (int k = N-1; k >= 0; k--)
  {
    if (q_block == 32)
    {
      switch(n_w)
      {
        case 1024:
          bi_q_kernel_beta<32, 1024><<<q_grid, 32>>>(k, x, w, u, t, p, v, q);
          break;
        case 512:
          bi_q_kernel_beta<32, 512><<<q_grid,  32>>>(k, x, w, u, t, p, v, q);
          break;
        case 256:
          bi_q_kernel_beta<32, 256><<<q_grid,  32>>>(k, x, w, u, t, p, v, q);
          break;
        case 128:
          bi_q_kernel_beta<32, 128><<<q_grid,  32>>>(k, x, w, u, t, p, v, q);
          break;
      }
    }
    if (q_block == 64)
    {
      switch(n_w)
      {
        case 1024:
          bi_q_kernel_beta<64, 1024><<<q_grid, 64>>>(k, x, w, u, t, p, v, q);
          break;
        case 512:
          bi_q_kernel_beta<64, 512><<<q_grid,  64>>>(k, x, w, u, t, p, v, q);
          break;
        case 256:
          bi_q_kernel_beta<64, 256><<<q_grid,  64>>>(k, x, w, u, t, p, v, q);
          break;
        case 128:
          bi_q_kernel_beta<64, 128><<<q_grid,  64>>>(k, x, w, u, t, p, v, q);
          break;
      }
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