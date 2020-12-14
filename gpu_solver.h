#ifndef GPU_SOLVER_H_
#define GPU_SOLVER_H_

int gpu_main(DPModel * model, int block_size, float *v_out, int *a_out, std::atomic<int>* busy_p_mat, std::atomic<bool>* running);

#endif //GPU_SOLVER_H_