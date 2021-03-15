#ifndef CPU_SOLVER_H_
#define CPU_SOLVER_H_
#include <thread>
#include <atomic>

#include "dp_model.h"
#include "utility.h"

struct Min_index
{
    int index;
    float value;
};

class CPUSolver
{
    public:
        CPUSolver(DPModel * ptr_in);
        ~CPUSolver();
        
        DPModel * model;
        int N;
        int n_x,n_w,n_u;
        int n_x_s, n_w_s;
        
        int solve(int k0, float d0, float v0, float dc0, int intention);
        long * value;
        int * action;
        bool debug = false;
        
    private:   
        int n_v;

        int n_p;
        int n_t;

        // subset
        long *r_cost = NULL;
        unsigned long long int *r_mask = NULL;
        long *t_cost = NULL;
        int *trans = NULL;
        float *prob = NULL;

        int find_min(long *q, int cnt);
        long calc_q(int k0, int k, int xk, int wk, int uk);
        int estimate_one_step(int k0, int k);
        int get_subset(int k0, int dk0, int dck0);

        // for debug only
        int wk0_debug = 0;
};

#endif // CPU_SOLVER_H_