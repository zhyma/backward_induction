#include <iostream>
#include "phy_shared.h"
#include "dp_model.h"

// Kernel function to add the elements of two arrays
__global__ void bi_kernel(int len, int *dims, int k, float *x, float *w, float *u, float *p, float *v, int *a)
{
  //dims[0]: x, dims[1]: w, dims[2]:u
  int index = blockIdx.x * blockDim.x + threadIdx.x;
  int stride = blockDim.x * gridDim.x;
  //printf("gridDim %d, blockDim %d, blockIdx %d, threadIdx %d\n", gridDim.x, blockDim.x, blockIdx.x, threadIdx.x);
  for (int i = index; i < len; i += stride)
  {
    // from index, get x, w, u
    int x_idx = i/(dims[0]);
    int w_idx = (i%dims[0])/dims[1];
    int u_idx = (i%dims[0])%dims[1];
    // input k, x, w, u
    float x_ = shared_linear_model(x[x_idx], w[w_idx], u[u_idx]);
    //printf("blockIdx %d, threadIdx %d, %d\n", blockIdx.x, threadIdx.x, i);
    //break;
  }
}

int gpu_main(DPModel * model)
{
  int len = model->x_set.count * model->w_set.count * model->u_set.count;
  int *dims;
  float *x, *w, *u;
  float *p, *v;
  int *a;

  // Allocate Unified Memory . accessible from CPU or GPU
  cudaMallocManaged(&dims, 3*sizeof(float));
  dims[0] = model->x_set.count;
  dims[1] = model->w_set.count;
  dims[2] = model->u_set.count;

  cudaMallocManaged(&x, model->x_set.count*sizeof(float));
  cudaMallocManaged(&w, model->w_set.count*sizeof(float));
  cudaMallocManaged(&u, model->u_set.count*sizeof(float));
  
  cudaMallocManaged(&p, model->w_set.count*model->w_set.count*sizeof(float));
  cudaMallocManaged(&v, 2*model->x_set.count*model->w_set.count*sizeof(float));
  cudaMallocManaged(&a, model->u_set.count*sizeof(int));

  dims[0] = model->x_set.count;
  dims[1] = model->w_set.count;
  dims[2] = model->u_set.count;
  // initialize x and y arrays on the host
  for (int i = 0; i < model->x_set.count; ++i)
  {
    x[i] = model->x_set.list[i];
  }
  for (int i = 0; i < model->w_set.count; ++i)
  {
    w[i] = model->w_set.list[i];
  }
  for (int i = 0; i < model->u_set.count; ++i)
  {
    u[i] = model->u_set.list[i];
  }

  for (int i = 0; i < model->w_set.count; ++i)
  {
    for (int j=0; j < model->w_set.count; ++j)
    {
      if (i==j)
        p[i*model->w_set.count + j] = 1;
      else
        p[i*model->w_set.count + j] = 0;
    }
  }

  // Run kernel on 1M elements on the GPU
  int blockSize = 256;
  int numBlocks = (len + blockSize - 1) / blockSize;
  bi_kernel<<<numBlocks, blockSize>>>(len, dims, 1, x, w, u, p, v, a);

  // Wait for GPU to finish before accessing on host
  cudaDeviceSynchronize();

  // // Check for errors (all values should be 3.0f)
  // float maxError = 0.0f;
  // for (int i = 0; i < N; i++)
  //   maxError = fmax(maxError, fabs(y[i]-3.0f));
  // std::cout << "Max error: " << maxError << std::endl;

  // Free memory
  cudaFree(x);
  cudaFree(w);
  cudaFree(u);
  cudaFree(p);
  cudaFree(v);
  cudaFree(a);

  return 0;
}