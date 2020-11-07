#ifndef CPU_SOLVER_H_
#define CPU_SOLVER_H_
#include "dp_model.h"

class CPUSolver
{
    public:
        DPModel * model;
        CPUSolver(DPModel * ptr_in);
        float * value;
        int * action;
        float * test_table;
    private:
        int N;
        int n_x,n_w,n_u;

        int xw_idx(int xk, int wk);
        int state_idx(int k, int xk, int wk);
        int find_min(float *q, int cnt, struct Min_index *min);
        float calc_q(int k, int xk, int wk, int uk);
        int estimate_one_step(int k);
};

#endif // CPU_SOLVER_H_