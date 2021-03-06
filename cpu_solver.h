#ifndef CPU_SOLVER_H_
#define CPU_SOLVER_H_
#include <thread>
#include <atomic>

#include "dp_model.h"
#include "utility.h"

#define PINGPONG 0
#define FULL     1

struct Min_index
{
    int index;
    float value;
};

class CPUSolver
{
    public:
        CPUSolver(DPModel * ptr_in, bool save = false);
        ~CPUSolver();
        
        DPModel * model;
        int N;
        long n_x,n_w,n_u;
        long n_x_s, n_w_s;
        
        int solve(int k0, float d0, float v0, float dc0, int intention);
        float * value;
        float * value_buffer;
        int * action;
        bool debug = true;
        
    private:
        bool save_v = false;

        int n_v;

        int n_p;
        int n_d;
        int n_dc;

        // subset
        long *r_cost = NULL;
        long *r_mask = NULL;
        long *t_cost = NULL;
        long *trans = NULL;
        float *prob = NULL;

        int find_min(float *q, int cnt);
        long calc_q(int k0, int k, long xk, long wk, int uk);
        int estimate_one_step(int k0, int k);
        int get_subset(int k0, int dk0, int dck0);

        // for debug only
        long xk0_debug = 0;
        long wk0_debug = 0;
};

#endif // CPU_SOLVER_H_