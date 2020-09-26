#ifndef DP_SOLVER_H_
#define DP_SOLVER_H_

#define NO_NOISE 0
#define MC_NOISE 1
#define FIX_NOISE 2

#define MONTECARLO 0
#define ALGEBRAIC 1

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

// move to physical model
typedef struct Set
{
    int count;
    float *list;
    float bound[2];
} Set;

class DPSolver
{
    public:
        PHYModel * ptr_model;
        int prob_type;

        int N;                          // time steps, same as the model
        int gran;                       // granularity.
        bool save_transition;
        int iter;

        Set x_set;
        Set u_set;
        Set w_set;

        int xw_cnt;
        int states_cnt;
        
        float *value_table;
        float *action_table;

        DPSolver(PHYModel * ptr_in, int prob, int sample_rate, int number_of_trials);

        int one_step_backward(int step);
        float estimate_one_step(int k);

        int create_distribution();
        int get_distribution(int wk, float * prob_table);

        int write_to_file();

    private:
        // Running the forward search once, you will get a
        float *temp_search;
        // A counter for all next states that appears. The extra one (last one, [x_cnt]) is for the sum of one current state.
        int *cnter_table;
        // The final model you get. The probability of transiting from one state to another.
        float *prob_table;

        float *center_distribution;

        //
        
        int find_min(float *u, int cnt, struct Min_index *min);
        int discretize(Set *in);

        int val_to_idx(float val, Set *ref);
        int xw_idx(int xk, int wk);
        int state_idx(int k, int xk, int wk);
        int sas2idx(int xk, int wk, int uk, int xk_, int wk_);

        int mc_one_stateaction(int k, int xk, int wk, int uk);
        int calc_one_step(int k);

        // for test
        int write_array_to_csv(int k, float * table);
};
#endif // DP_SOLVER_H_