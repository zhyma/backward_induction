#ifndef CPU_SOLVER_H_
#define CPU_SOLVER_H_
#include <thread>
#include <atomic>

#include "dp_model.h"

struct Min_index
{
    int index;
    float value;
};

class CPUSolver
{
    public:
        DPModel * model;
        CPUSolver(DPModel * ptr_in, std::atomic<int>* busy_p_mat);
        int solve();
        float * value;
        int * action;
    private:
        std::atomic<int>* busy_mat_ptr;
        int N;
        int n_x,n_w,n_u;

        int find_min(float *q, int cnt, struct Min_index *min);
        float calc_q(int k, int xk, int wk, int uk);
        int estimate_one_step(int k);
};

#endif // CPU_SOLVER_H_