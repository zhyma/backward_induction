#ifndef DP_SOLVER_H_
#define DP_SOLVER_H_

#define NO_NOISE 0
#define MC_NOISE 1
#define FIX_NOISE 2

#include <iostream>
#include <cmath>
#include <math.h>
#include <fstream>
#include <stdlib.h>
#include <time.h>
#include "phy_model.h"
using namespace std;

struct Min_index
{
    int index;
    float value;
};

class DPSolver
{
    public:
        PHYModel * ptr_model;
        int N;                          // time steps, same as the model
        int gran;                       // granularity.
        bool save_transition;

        // total number of x and u (after discretized)
        int x_cnt;
        int u_cnt;

        // all possible x and u
        float *x_list;
        float *u_list;

        float *value_table;
        float *action_table;

        DPSolver(PHYModel * ptr_in, int sample_rate);


        int kxu2index(int k, int x, int u);
        int x2x_cnt(float x);
        int global_forward_once(float x0);
        int estimate_model(int iter);
        int solve_whole_model();

        int one_step_backward(int step);
        float solve_one_step(int k);

        int write_to_file();

    private:
        // Running the forward search once, you will get a
        float *temp_search;
        // A counter for all next states that appears. The extra one (last one, [x_cnt]) is for the sum of one current state.
        int *cnter_table;
        // The final model you get. The probability of transiting from one state to another.
        float *prob_table;

        //
        int find_min(float *u, int cnt, struct Min_index *min);
        int xu2index(int xk, int uk, int x_);
        int search_one_step(int k, int iter);

};
#endif // DP_SOLVER_H_