#ifndef CPU_SOLVER_H_
#define CPU_SOLVER_H_
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
        CPUSolver(DPModel * ptr_in);
    private:
        int xw_idx(int xk, int wk);
        int state_idx(int k, int xk, int wk);
        int find_min(float *u, int cnt, struct Min_index *min);
        float calc_q(int k, int xk, int wk, int uk);
        float estimate_one_step(int k);
};

#endif // CPU_SOLVER_H_