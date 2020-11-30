#ifndef GPU_SOLVER_H_
#define GPU_SOLVER_H_

int gpu_main(DPModel * model, int block_size, float *v_out, int *a_out);

#endif //GPU_SOLVER_H_