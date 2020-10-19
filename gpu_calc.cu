#include "gpu_calc.h"
#include <iostream>
#include <math.h>


// Kernel function calculate the terminal cost/value
__global__ void terminal_kernel(int n, float *s, float *j)
{
  int index = blockIdx.x * blockDim.x + threadIdx.x;
  int stride = blockDim.x * gridDim.x;
  
  for (int i = index; i < n; i += stride)
  {
    j[i] = (1-s[i])*(1-s[i]);
  }
}

void terminal_value(Set x_set, Set w_set, float *input, float *output)
{
  int N = (x_set.count*w_set.count);
  //state, j(terminal cost)
  float *s, *j;
  // Allocate Unified Memory – accessible from CPU or GPU
  cudaMallocManaged(&s, N*sizeof(float));
  cudaMallocManaged(&j, N*sizeof(float));

  // prepare input data
  cudaMemcpy(s, input, N*sizeof(float), cudaMemcpyHostToDevice);

  // Run kernel
  int blockSize = 256;
  int numBlocks = (N + blockSize - 1) / blockSize;
  terminal_kernel<<<numBlocks, blockSize>>>(N, s, j);

  // Wait for GPU to finish before accessing on host
  cudaDeviceSynchronize();

  cudaMemcpy(output, j, N*sizeof(float), cudaMemcpyDeviceToHost);
  printf("\n CUDA part done\n");
  // Free memory
  cudaFree(s);
  cudaFree(j);
  return;
}

const int blockSize = 256;
// Kernel function to 
__global__ void intermediate_kernel(int N, float *p, float *v, float *sum)
{
  __shared__ float cache[blockSize];
  int tid = threadIdx.x + blockIdx.x * blockDim.x;

  int cacheIndex = threadIdx.x;
  float temp = 0;
  while (tid < N)
  {
    temp += p[tid] * v[tid];
    //printf("at i=%d, prob is: %f, , value is: %f\n", tid, p[tid], v[tid]);
    tid += blockDim.x * gridDim.x;
  }
  cache[cacheIndex] = temp;

  __syncthreads();

  int i = blockDim.x/2;
  while (i != 0) {
    if (cacheIndex < i)
      cache[cacheIndex] += cache[cacheIndex + i];
    __syncthreads();
    i /= 2;
  }
  if (cacheIndex == 0)
  {
    sum[0] = cache[0];
    //printf("sum is %f\n", sum[0]);
  }
}

void intermediate_value(Set x_set, Set w_set, float *prob, float *value, float *output)
{
  int N = (w_set.count);
  //state, j(terminal cost)
  float *p, *v, *sum;
  // Allocate Unified Memory – accessible from CPU or GPU
  cudaMallocManaged(&p, N*sizeof(float));
  cudaMallocManaged(&v, N*sizeof(float));
  cudaMallocManaged(&sum, sizeof(float));

  // prepare input data
  cudaMemcpy(p, prob, N*sizeof(float), cudaMemcpyHostToDevice);
  cudaMemcpy(v, value, N*sizeof(float), cudaMemcpyHostToDevice);

  // Run kernel on 1M elements on the GPU
  int numBlocks = (N + blockSize - 1) / blockSize;
  intermediate_kernel<<<numBlocks, blockSize>>>(N, p, v, sum);

  // Wait for GPU to finish before accessing on host
  cudaDeviceSynchronize();

  cudaMemcpy(output, sum, sizeof(float), cudaMemcpyDeviceToHost);
  // Free memory
  cudaFree(p);
  cudaFree(v);
  return;
}